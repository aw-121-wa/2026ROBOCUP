#include "path_ports.h"
#include "path_mission.h"
#include "rdk_link.h"
#include "turntable_link.h"
#include "chassis_control.h"
#include "pin_config.h"
#include "disc_task_config.h"
#include <string.h>
static PathMission mission;
static RdkLink rdk;
static TurntableLink turn;
static uint8_t turn_buffer[16];
static unsigned turn_issued;
static bool turn_enabled;
static bool turn_transmit(void *ctx, const uint8_t *data, size_t size)
{
    (void)ctx;
    if (size > sizeof(turn_buffer) || PINCFG_TURNTABLE_UART->gState != HAL_UART_STATE_READY)
        return false;
    memcpy(turn_buffer, data, size);
    return HAL_UART_Transmit_IT(PINCFG_TURNTABLE_UART, turn_buffer, (uint16_t)size) == HAL_OK;
}
static void cancel_turn(void)
{
    turn_enabled = false;
    if (turn.pending && !turn.stopping) Turn_Stop(&turn, HAL_GetTick());
}
static void service_turn(uint32_t now)
{
    if (!Chassis_GetState()->armed || Chassis_GetState()->fault || mission.result >= PATH_CANCELED)
        cancel_turn();
    if (turn_enabled && !turn.pending && turn.reply != PATH_FAILED && turn_issued < mission.id_count)
    {
        if (Turn_Start(&turn, false, now)) ++turn_issued;
    }
    Turn_Tick(&turn, now, PINCFG_TURNTABLE_UART->gState == HAL_UART_STATE_READY);
    if (turn.reply == PATH_FAILED) turn_enabled = false;
}
volatile PathDiagnostics path_diagnostics;
static bool ready, initialized, motion_pending, verified, test_ping, stationary;
static uint32_t motion_since, motion_timeout, last_rx;
static volatile uint32_t io_fault;
static volatile uint32_t rfid_fault; /* Record-only diagnostics, never a motion gate. */
static uint8_t rx_byte, tx_buffer[80];
static volatile uint8_t ring[128];
static volatile unsigned head, tail;
/* Added UART7 capture; ZHY UART4 wire protocol remains unchanged. */
static uint8_t rfid_byte;
static volatile uint8_t rfid_ring[128];
static volatile unsigned rfid_head, rfid_tail;
static volatile bool rfid_capture;
static uint8_t rfid_frame[28], rfid_length;
/* Accept auto UID, auto UID+block, and A1 read-UID replies only.
 * Sliding resynchronization also handles noise and corrupt/partial frames. */
static void rfid_feed(uint8_t byte)
{
    rfid_frame[rfid_length++] = byte;
    while (rfid_length)
    {
        uint8_t *f = rfid_frame;
        bool valid = f[0] == 4 || f[0] == 1 || f[0] == 3;
        if (valid && rfid_length < 2) return;
        if (valid) valid = f[1] == 8 || f[1] == 12 ||
                           (f[0] == 4 && (f[1] == 22 || f[1] == 28));
        if (valid && rfid_length < 3) return;
        if (valid) valid = f[1] == 8 || (f[0] == 4 && ((f[1] == 12 && f[2] == 2) ||
                                         (f[1] == 22 && f[2] == 3) ||
                                         (f[1] == 28 && f[2] == 4))) ||
                           (f[0] == 1 && f[1] == 12 && f[2] == 0xa1);
        if (valid && rfid_length < 4) return;
        if (valid) valid = f[3] == 0x20;
        if (valid && rfid_length < 5) return;
        if (valid && rfid_length < f[1]) return;
        if (valid)
        {
            uint8_t checksum = 0;
            for (unsigned i = 0; i < f[1]; ++i) checksum ^= f[i];
            if (checksum == 0xff)
            {
                if (f[4] == 0 && (f[1] == 12 || f[1] == 28))
                    Path_RecordId(&mission, ((uint32_t)f[7] << 24) |
                              ((uint32_t)f[8] << 16) | ((uint32_t)f[9] << 8) | f[10]);
                unsigned consumed = f[1];
                rfid_length -= consumed;
                memmove(f, f + consumed, rfid_length);
                continue;
            }
        }
        --rfid_length;
        memmove(f, f + 1, rfid_length);
    }
}
size_t PathPorts_CopyIds(uint32_t *out, size_t capacity)
{
    if (!out || !capacity) return 0;
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    size_t count = mission.id_count < capacity ? mission.id_count : capacity;
    memcpy(out, mission.id_list, count * sizeof(*out));
    __set_PRIMASK(mask);
    return count;
}
static void record_pending_ids(void)
{
    unsigned id_budget = sizeof(rfid_ring);
    while (rfid_tail != rfid_head && id_budget--)
    {
        uint8_t id = rfid_ring[rfid_tail];
        rfid_tail = (rfid_tail + 1U) % sizeof(rfid_ring);
        rfid_feed(id);
    }
}
static bool transmit(void *ctx, const char *s, size_t n)
{
    (void)ctx;
    if (n > sizeof(tx_buffer) || PINCFG_RDK_UART->gState != HAL_UART_STATE_READY)
        return false;
    memcpy(tx_buffer, s, n);
    return HAL_UART_Transmit_IT(PINCFG_RDK_UART, tx_buffer, (uint16_t)n) == HAL_OK;
}
static bool send(void *ctx, const PathCommand *c)
{
    (void)ctx;
    uint32_t now = HAL_GetTick();
    float scale = 2.0f * 3.141592654f * chassis_config.wheel_radius_mm / 60.0f;
    switch (c->kind)
    {
    case PC_MOVE:
        if (!Chassis_Move(c->x, c->y, c->speed * scale, 550, 550))
            return false;
        motion_pending = true;
        motion_since = now;
        motion_timeout = c->timeout_ms;
        return true;
    case PC_ROTATE:
        if (!Chassis_Rotate(c->x))
            return false;
        motion_pending = true;
        motion_since = now;
        motion_timeout = c->timeout_ms;
        return true;
    case PC_BODY:
        if (!Chassis_Body(c->x * scale, c->y * scale,
                          c->speed * scale /
                              (chassis_config.half_track_mm + chassis_config.half_wheelbase_mm)))
            return false;
        if (!motion_pending)
        {
            motion_since = now;
            motion_timeout = c->timeout_ms ? c->timeout_ms : 5000;
        }
        motion_pending = true;
        return true;
    case PC_HOLD:
        Chassis_Hold();
        motion_pending = false;
        return true;
    case PC_HELLO:
        return Rdk_Begin(&rdk, "HELLO", 0, now, 2000);
    case PC_DISC:
        if (!Chassis_IsSettled())
            return false;
        if (!Rdk_Begin(&rdk, "DISC", 0, now, c->timeout_ms))
            return false;
        {
            uint32_t mask = __get_PRIMASK();
            __disable_irq();
            rfid_head = rfid_tail = 0;
            rfid_length = 0;
            rfid_capture = true;
            __set_PRIMASK(mask);
        }
        return true;
    case PC_CANCEL:
        cancel_turn();
        rfid_capture = false;
        record_pending_ids();
        Chassis_Hold();
        motion_pending = false;
        verified = false;
        return Rdk_Begin(&rdk, "STOP", 0, now, 1);
    default:
        return false; /* No GROUP/VISION/turntable commands in this scope. */
    }
}
void PathPorts_Init(void)
{
    Turn_Init(&turn, turn_transmit, 0);
    Rdk_Init(&rdk, 0, transmit, 0);
    Path_Init(&mission, send, 0);
    initialized = true;
    ready = HAL_UART_Receive_IT(PINCFG_RDK_UART, &rx_byte, 1) == HAL_OK;
    if (HAL_UART_Receive_IT(PINCFG_RFID_UART, &rfid_byte, 1) != HAL_OK)
        rfid_fault |= 1;
    if (!ready)
        io_fault |= 1;
}
bool PathPorts_Busy(void)
{
    return initialized && (mission.result == PATH_RUNNING || rdk.active || rdk.locked || turn.pending ||
                           (turn_enabled && turn_issued < mission.id_count));
}
bool PathPorts_Ping(void)
{
    if (!ready || io_fault || PathPorts_Busy() || !Chassis_IsSettled())
        return false;
    verified = false;
    test_ping = true;
    return Rdk_Begin(&rdk, "HELLO", 0, HAL_GetTick(), 2000);
}
bool PathPorts_Reset(void)
{
    if (PINCFG_RDK_UART->gState != HAL_UART_STATE_READY || Chassis_GetState()->armed || !Chassis_IsSettled() || mission.result == PATH_RUNNING ||
        rdk.active || turn.pending)
        return false;
    if (HAL_UART_AbortReceive(PINCFG_RDK_UART) != HAL_OK)
        return false;
    bool rfid_abort_ok = HAL_UART_AbortReceive(PINCFG_RFID_UART) == HAL_OK;
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    head = tail = rfid_head = rfid_tail = 0;
    rfid_length = 0;
    rfid_capture = false;
    io_fault = 0;
    rfid_fault = rfid_abort_ok ? 0U : 2U;
    __set_PRIMASK(mask);
    Rdk_Init(&rdk, 0, transmit, 0);
    /* Link recovery does not erase the RFID result; next accepted task does. */
    uint32_t saved_ids[64];
    uint8_t saved_count = mission.id_count;
    bool saved_overflow = mission.id_overflow;
    uint16_t saved_mask = mission.ids;
    memcpy(saved_ids, mission.id_list, sizeof(saved_ids));
    Path_Init(&mission, send, 0);
    memcpy(mission.id_list, saved_ids, sizeof(saved_ids));
    mission.id_count = saved_count;
    mission.id_overflow = saved_overflow;
    mission.ids = saved_mask;
    Turn_Init(&turn, turn_transmit, 0);
    turn_enabled = false;
    turn_issued = mission.id_count;
    verified = test_ping = stationary = motion_pending = false;
    ready = HAL_UART_Receive_IT(PINCFG_RDK_UART, &rx_byte, 1) == HAL_OK;
    if (HAL_UART_Receive_IT(PINCFG_RFID_UART, &rfid_byte, 1) != HAL_OK)
        rfid_fault |= 1;
    if (!ready)
        io_fault = 1;
    return ready;
}
static bool start(bool disc_only)
{
    if (!ready || io_fault || !verified || PathPorts_Busy() || !Chassis_IsSettled())
        return false;
    PathInput in = {.armed = Chassis_GetState()->armed,
                    .fault = Chassis_GetState()->fault != 0,
                    .settled = true};
    stationary = disc_only;
    test_ping = false;
    if (!Path_Start(&mission, HAL_GetTick(), &in)) return false;
    Turn_Init(&turn, turn_transmit, 0);
    turn_issued = 0;
    turn_enabled = true;
    return true;
}
bool PathPorts_Start(void)
{
    return start(false);
}
bool PathPorts_Disc(void)
{
    return start(true);
}
void PathPorts_Cancel(void)
{
    if (!initialized)
        return;
    cancel_turn();
    if (mission.result == PATH_RUNNING)
        Path_Cancel(&mission);
    else if (rdk.active && rdk.stage != 1)
    {
        (void)Rdk_Begin(&rdk, "STOP", 0, HAL_GetTick(), 1);
        verified = false;
    }
}
void PathPorts_RxComplete(UART_HandleTypeDef *u)
{
    if (u == PINCFG_RFID_UART)
    {
        if (rfid_capture)
        {
            unsigned next = (rfid_head + 1U) % sizeof(rfid_ring);
            if (next == rfid_tail)
                rfid_fault |= 64;
            else
            {
                rfid_ring[rfid_head] = rfid_byte;
                rfid_head = next;
            }
        }
        if (HAL_UART_Receive_IT(u, &rfid_byte, 1) != HAL_OK)
            rfid_fault |= 8;
        return;
    }
    if (u != PINCFG_RDK_UART)
        return;
    unsigned next = (head + 1U) % sizeof(ring);
    if (next == tail)
        io_fault |= 2;
    else
    {
        ring[head] = rx_byte;
        head = next;
    }
    if (HAL_UART_Receive_IT(u, &rx_byte, 1) != HAL_OK)
        io_fault |= 4;
}
void PathPorts_Error(UART_HandleTypeDef *u)
{
    if (u == PINCFG_RDK_UART)
        io_fault |= 16;
    else if (u == PINCFG_RFID_UART)
        rfid_fault |= 16;
}
void PathPorts_Tick(void)
{
    if (!initialized)
        return;
    uint32_t now = HAL_GetTick();
    /* Expire before processing newly received replies; a late reply cannot revive a timeout. */
    Rdk_Tick(&rdk, now);
    if (rdk.length && (uint32_t)(now - last_rx) >= 500)
        rdk.overflow = true;
    unsigned budget = sizeof(ring);
    while (tail != head && budget--)
    {
        uint8_t b = ring[tail];
        tail = (tail + 1U) % sizeof(ring);
        Rdk_Feed(&rdk, b);
        last_rx = now;
    }
    record_pending_ids();
    if (io_fault)
    {
        rdk.locked = true;
        rdk.active = false;
        rdk.stage = 6;
        rdk.error = 4;
        rdk.reply = PATH_FAILED;
    }
    if (test_ping && rdk.stage == 2)
    {
        verified = true;
        test_ping = false;
    }
    if (rdk.locked)
        verified = false;
    if (motion_pending)
    {
        if ((uint32_t)(now - motion_since) >= motion_timeout)
        {
            io_fault |= 32;
            Chassis_Hold();
            motion_pending = false;
        }
        else if (!Chassis_MotionBusy())
        {
            Chassis_Hold();
            motion_pending = false;
        }
    }
    uint8_t gray = 0;
    if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_0) == GPIO_PIN_RESET)
        gray |= 4;
    if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_1) == GPIO_PIN_RESET)
        gray |= 2;
    bool disc_deadline = mission.result == PATH_RUNNING && mission.step == 3 &&
                         mission.phase == 1 &&
                         (uint32_t)(now - mission.entered) >= DISC_TASK_TIMEOUT_MS;
    uint32_t ir_raw = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_10) == GPIO_PIN_SET;
    PathInput in = {.armed = Chassis_GetState()->armed,
                    .fault = io_fault || Chassis_GetState()->fault ||
                             (rdk.locked && !(rdk.error == 1 && disc_deadline)),
                    .settled = Chassis_IsSettled(),
                    .gray = gray,
                    .yaw_deg = Chassis_ContinuousYaw() * 57.295779513f,
                    .ir = ir_raw == 0,
                    .reply = rdk.reply};
    PathResult previous = mission.result;
    unsigned phase = mission.phase;
    Path_Tick(&mission, now, &in);
    if (stationary && phase == 99 && mission.phase == 0 && mission.result == PATH_RUNNING)
    {
        mission.step = 3;
        mission.phase = 1;
        mission.entered = now;
        PathCommand c = {.kind = PC_DISC, .timeout_ms = DISC_TASK_TIMEOUT_MS};
        if (!send(0, &c))
            mission.result = PATH_ERROR;
    }
    if (!stationary && previous == PATH_RUNNING && mission.result == PATH_DONE && mission.step == 3)
    {
        mission.result = PATH_RUNNING;
        mission.step = 4;
        mission.phase = 0;
        mission.entered = now;
        mission.waiting = mission.stable = false;
    }
    if (mission.result != PATH_RUNNING || mission.step != 3)
        rfid_capture = false;
    if (previous == PATH_RUNNING && mission.result >= PATH_CANCELED)
    {
        Chassis_Hold();
        motion_pending = false;
        verified = false;
        if (!rdk.locked)
            (void)Rdk_Begin(&rdk, "STOP", 0, now, 1);
    }
    service_turn(now);
    path_diagnostics = (PathDiagnostics){.result = mission.result,
                                         .step = mission.step,
                                         .phase = mission.phase,
                                         .accepted_ids = verified, /* Preserve ZHY CH29. */
                                         .ids = mission.ids,
                                         .rfid_count = mission.id_count,
                                         .ir_raw = ir_raw,
                                         .settled = in.settled,
                                         .rfid_fault = rfid_fault | (mission.id_overflow ? 128U : 0U),
                                         .turn_reply = turn.reply,
                                         .link_reply = rdk.reply,
                                         .fault = io_fault,
                                         .sequence = rdk.sequence,
                                         .link_stage = rdk.stage,
                                         .link_error = rdk.error,
                                         .gray = gray};
}
