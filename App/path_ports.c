#include "path_chassis.h"
#include "path_policy.h"
#include "path_config.h"
#include "path_ports.h"
#include "path_mission.h"
#include "path_warehouse.h"
#include "rdk_link.h"
#include "turntable_link.h"
#include "chassis_control.h"
#include "pin_config.h"
#include "disc_task_config.h"
#include "ir_start.h"
#include <string.h>
#include <math.h>
static IrStart ir_start;
static PathMission mission;
static RdkLink rdk;
static TurntableLink turn;
static uint8_t turn_buffer[16];
static enum { TURN_IDLE, TURN_STORE, TURN_UNLOAD } turn_purpose;
static unsigned turn_issued;
static bool turn_enabled;
static uint32_t inventory_fault;
static volatile uint32_t io_fault;
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
    if (!PATH_SKIP_MATERIAL(&mission) && (mission.inventory.occupied || turn_purpose != TURN_IDLE ||
        (mission.result == PATH_RUNNING && rdk.active &&
         (rdk.stage == 4 || rdk.stage == 10 || rdk.stage == 12))))
        mission.inventory.uncertain = true;
    turn_enabled = false;
    turn_issued = mission.inventory.collected;
    turn_purpose = TURN_IDLE;
    if (turn.pending && !turn.stopping) Turn_Stop(&turn, HAL_GetTick());
}
static void service_turn(uint32_t now)
{
    if (!Chassis_GetState()->armed || Chassis_GetState()->fault || mission.result >= PATH_CANCELED)
        cancel_turn();
    /* Advance once per captured slot, including a timed-out unknown ID. */
    if (turn_enabled && !turn.pending && turn.reply != PATH_FAILED && turn_issued < mission.inventory.collected)
    {
        if (Turn_Start(&turn, false, now))
        {
            ++turn_issued;
            turn_purpose = TURN_STORE;
        }
    }
    turn.no_timeout=mission.result==PATH_RUNNING && PATH_WAREHOUSE_UNTIMED(&mission);
    Turn_Tick(&turn, now, PINCFG_TURNTABLE_UART->gState == HAL_UART_STATE_READY);
    if (turn_purpose != TURN_IDLE && !turn.pending)
    {
        if (turn.reply == PATH_OK)
            for (unsigned i=0;i<turn.steps;++i)
                BallInventory_Step(&mission.inventory, turn.direction != 0);
        else
        {
            inventory_fault |= INVENTORY_TURN_ERROR;
            mission.inventory.uncertain = true;
        }
        turn_purpose = TURN_IDLE;
    }
    if (turn.reply == PATH_FAILED) turn_enabled = false;
}
/* Capture slots and UID count are independent: an unread ball still occupies a slot. */
static void remember_ball(uint32_t uid, const uint8_t *block)
{
    BallInventory *b = &mission.inventory;
    unsigned slot = b->collected;
    if (slot >= BALL_SLOT_COUNT)
    {
        inventory_fault |= INVENTORY_FULL;
        b->uncertain = true;
        return;
    }
    uint8_t code = block ? BallInventory_Decode(block) : 0;
    if (!code) inventory_fault |= block ? INVENTORY_BAD_BLOCK : INVENTORY_NO_BLOCK;
    for (unsigned i=0;code && i<slot;i++)
        if (b->code[i] == code)
        {
            inventory_fault |= INVENTORY_CONFLICT;
            code = 0;
        }
    /* Collection order identifies the intended pocket even while prior turns are queued. */
    b->uid[slot] = uid;
    b->code[slot] = code;
    b->occupied |= (uint16_t)(1U << slot);
    ++b->collected;
}
volatile PathDiagnostics path_diagnostics;
static bool ready, initialized, motion_pending, motion_continuous, verified, test_ping, stationary;
/* 0 wait/retry PING, 1 handshake, 2 G0 pending, 3 ready, 4 stopped/failed. */
static unsigned boot;
static bool requested_blue, side_pending = PATH_VISION_ENABLE != 0, side_inflight;
static uint32_t boot_retry;
static volatile uint32_t rdk_uart_hal_error, rdk_uart_error_count;
static uint32_t motion_since, motion_timeout, last_rx;
static volatile uint32_t rfid_fault;
static uint8_t rx_byte, tx_buffer[80];
static volatile uint8_t ring[128];
static volatile unsigned head, tail;
/* Added UART7 capture; ZHY UART4 wire protocol remains unchanged. */
static uint8_t rfid_byte;
static volatile uint8_t rfid_ring[128];
static volatile unsigned rfid_head, rfid_tail;
static volatile bool rfid_capture;
static uint8_t rfid_frame[28], rfid_length;
static uint8_t disc_action_done_index, disc_rfid_confirmed_index;
static volatile uint8_t disc_waiting_rfid_index;
static uint8_t rfid_gate_base_count;
static uint32_t rfid_wait_since;
/* Accept auto UID, auto UID+block, and A1 read-UID replies only.
 * Sliding resynchronization also handles noise and corrupt/partial frames. */
static bool rfid_feed(uint8_t byte)
{
    rfid_frame[rfid_length++] = byte;
    while (rfid_length)
    {
        uint8_t *f = rfid_frame;
        bool valid = f[0] == 4 || f[0] == 1 || f[0] == 3;
        if (valid && rfid_length < 2) return false;
        if (valid) valid = f[1] == 8 || f[1] == 12 ||
                           (f[0] == 4 && (f[1] == 22 || f[1] == 28));
        if (valid && rfid_length < 3) return false;
        if (valid) valid = f[1] == 8 || (f[0] == 4 && ((f[1] == 12 && f[2] == 2) ||
                                         (f[1] == 22 && f[2] == 3) ||
                                         (f[1] == 28 && f[2] == 4))) ||
                           (f[0] == 1 && f[1] == 12 && f[2] == 0xa1);
        if (valid && rfid_length < 4) return false;
        if (valid) valid = f[3] == 0x20;
        if (valid && rfid_length < 5) return false;
        if (valid && rfid_length < f[1]) return false;
        if (valid)
        {
            uint8_t checksum = 0;
            for (unsigned i = 0; i < f[1]; ++i) checksum ^= f[i];
            if (checksum == 0xff)
            {
                uint8_t previous_count = mission.id_count;
                if (f[4] == 0 && (f[1] == 12 || f[1] == 28))
                {
                    uint32_t uid = ((uint32_t)f[7] << 24) |
                                   ((uint32_t)f[8] << 16) | ((uint32_t)f[9] << 8) | f[10];
                    Path_RecordId(&mission, uid);
                    if (mission.id_count > previous_count)
                        remember_ball(uid, f[1] == 28 ? f + 11 : NULL);
                }
                unsigned consumed = f[1];
                rfid_length -= consumed;
                memmove(f, f + consumed, rfid_length);
                if (disc_waiting_rfid_index && mission.id_count > previous_count &&
                    mission.id_count > rfid_gate_base_count)
                {
                    uint8_t index = disc_waiting_rfid_index;
                    if (Rdk_SendDiscRfidOk(&rdk, index))
                    {
                        disc_rfid_confirmed_index = index;
                        disc_waiting_rfid_index = 0;
                        return true;
                    }
                }
                continue;
            }
        }
        --rfid_length;
        memmove(f, f + 1, rfid_length);
    }
    return false;
}
void PathPorts_CopyInventory(BallInventory *out)
{
    if (!out) return;
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    *out = mission.inventory;
    __set_PRIMASK(mask);
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
        if (rfid_feed(id))
        {
            uint32_t mask = __get_PRIMASK();
            __disable_irq();
            rfid_capture = false;
            rfid_tail = rfid_head;
            rfid_length = 0;
            __set_PRIMASK(mask);
            break;
        }
    }
}
static void close_rfid_gate(void)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    rfid_capture = false;
    rfid_head = rfid_tail = 0;
    rfid_length = 0;
    disc_waiting_rfid_index = 0;
    __set_PRIMASK(mask);
}
static void open_rfid_gate(uint8_t index)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    rfid_capture = false;
    rfid_head = rfid_tail = 0;
    rfid_length = 0;
    rfid_gate_base_count = mission.id_count;
    rfid_wait_since = HAL_GetTick();
    disc_waiting_rfid_index = index;
    rfid_capture = true;
    __set_PRIMASK(mask);
}
static bool transmit(void *ctx, const char *s, size_t n)
{
    (void)ctx;
    if (n > sizeof(tx_buffer) || PINCFG_RDK_UART->gState != HAL_UART_STATE_READY)
        return false;
    memcpy(tx_buffer, s, n);
    return HAL_UART_Transmit_IT(PINCFG_RDK_UART, tx_buffer, (uint16_t)n) == HAL_OK;
}
static bool motion_started(const PathCommand *command, uint32_t now, bool continuous)
{
    motion_pending=true;
    motion_continuous=continuous;
    motion_since=now;
    motion_timeout=command->timeout_ms;
    return true;
}
static bool send(void *ctx, const PathCommand *c)
{
    if (PATH_BLUE_WAREHOUSE_TEST &&
        (c->kind==PC_GROUP || c->kind==PC_TURN || c->kind==PC_DISC ||
         c->kind==PC_VISION || c->kind==PC_STAIR || c->kind==PC_STAIR_SCAN)) return false;
    (void)ctx;
    uint32_t now = HAL_GetTick();
    float speed_scale = PathPolicy_Chassis(mission.blue,mission.result,mission.step,mission.phase).travel_speed_scale;
    float boost = PathPolicy_CommandBoost(mission.step,c->kind);
    float accel_scale = speed_scale * fminf(boost,PATH_TRAVEL_BOOST);
    float transit_scale = PathPolicy_TransitScale(mission.step,c->kind);
    boost *= transit_scale;
    accel_scale *= transit_scale;
    float decel_scale = speed_scale * transit_scale;
    float scale = speed_scale * 2.0f * 3.141592654f * chassis_config.wheel_radius_mm / 60.0f;
    switch (c->kind)
    {
    case PC_ORBIT_ARC:
        if (!Chassis_ExitOrbitArc(c->x,c->angle,c->speed*scale*boost,PATH_MOVE_ACCEL_MM_S2*accel_scale,PATH_MOVE_DECEL_MM_S2*decel_scale)) return false;
        return motion_started(c,now,true);
    case PC_ORBIT_EXIT:
        if (!Chassis_ExitOrbit(c->x,c->y,c->angle,c->speed*scale*boost,c->end_speed*scale,PATH_MOVE_ACCEL_MM_S2*accel_scale,PATH_MOVE_DECEL_MM_S2*decel_scale)) return false;
        return motion_started(c,now,true);
    case PC_MOVE_ROTATE:
        if (c->continuous)
        {
            if (c->end_speed <= 0 ||
                !Chassis_MoveRotateBoundary(c->x, c->y, c->angle, c->speed * scale * boost, PATH_MOVE_ACCEL_MM_S2 * accel_scale, PATH_MOVE_DECEL_MM_S2 * decel_scale,
                                            c->start_speed * scale, c->end_speed * scale))
                return false;
        }
        else if (!Chassis_MoveRotate(c->x, c->y, c->angle, c->speed * scale * boost, PATH_MOVE_ACCEL_MM_S2 * accel_scale, PATH_MOVE_DECEL_MM_S2 * decel_scale))
            return false;
        return motion_started(c,now,c->continuous);
    case PC_HOME_ALIGN:
        if (!Chassis_AlignHome(c->x)) return false;
        return motion_started(c,now,false);
    case PC_RETURN_HOME:
        if (!Chassis_ReturnHome(c->argument)) return false;
        return motion_started(c,now,false);
    case PC_FINISH_FORWARD:
        if (!Chassis_FinishForward(c->x,c->speed*scale*boost,c->acceleration*accel_scale,c->deceleration*decel_scale)) return false;
        return motion_started(c,now,false);
    case PC_MOVE:
        if (!Chassis_MoveBoundary(c->x, c->y, c->speed * scale * boost,
                                  (c->acceleration > 0 ? c->acceleration : PATH_MOVE_ACCEL_MM_S2) * accel_scale,
                                  (c->deceleration > 0 ? c->deceleration : PATH_MOVE_DECEL_MM_S2) * decel_scale,
                                  c->start_speed * scale, c->end_speed * scale))
            return false;
        return motion_started(c,now,c->continuous);
    case PC_ARC:
        if (!Chassis_MoveArc(c->x, c->y, c->angle, c->speed * scale * boost, PATH_MOVE_ACCEL_MM_S2 * accel_scale, PATH_MOVE_DECEL_MM_S2 * decel_scale,
                             c->start_speed * scale, c->end_speed * scale))
            return false;
        return motion_started(c,now,c->continuous);
    case PC_LINE_CALIBRATE:
        return Chassis_CalibrateLine();
    case PC_LINE_SEARCH:
        if (!Chassis_LineSearch(c->y * scale, c->speed * scale /
                               (chassis_config.half_track_mm + chassis_config.half_wheelbase_mm)))
            return false;
        if (!motion_pending) { motion_since = now; motion_timeout = c->timeout_ms; }
        motion_pending = true;
        return true;
    case PC_LINE_REFERENCE:
        return Chassis_SetLineReference();
    case PC_MAP_SEARCH:
        if (!Chassis_MapSearch(c->y * speed_scale)) return false;
        if (!motion_pending) { motion_since = now; motion_timeout = c->timeout_ms; }
        motion_pending = true;
        return true;
    case PC_MAP_HEADING:
        return Chassis_SetMapHeading(c->x);
    case PC_MAP_AXIS:
    case PC_MAP_LATERAL:
        if (!(c->kind == PC_MAP_AXIS ? Chassis_AlignMapAxis() : Chassis_MapLateral(c->y)))
            return false;
        return motion_started(c,now,false);
    case PC_ALIGN_ZERO:
    case PC_ROTATE:
        if (!(c->kind == PC_ALIGN_ZERO ? Chassis_AlignZero() : Chassis_Rotate(c->x)))
            return false;
        return motion_started(c,now,false);
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
        if ((mission.step == 6 && (mission.phase == 2 || mission.phase == 3)) ||
            (mission.step == 9 && mission.phase == 22 && c->argument == 1))
            Chassis_HoldCapture();
        else Chassis_Hold();
        motion_pending = false;
        motion_continuous = false;
        return true;
    case PC_BLOCK_CHECK:
        return mission.step==13 && Chassis_IsSettled() &&
               c->argument>=1 && c->argument<=3 &&
               Rdk_BlockBeginCell(&rdk,(uint8_t)c->argument,mission.destack.column+1,now,c->timeout_ms);
    case PC_WAREHOUSE_DIGIT:
        return mission.step==13 && c->argument<=14 &&
               Rdk_WarehouseBegin(&rdk,(uint8_t)c->argument,now,c->timeout_ms);
    case PC_HELLO:
        return Rdk_Begin(&rdk, "HELLO", 0, now, 2000);
    case PC_TURN:
        if (mission.step != 13 || !Chassis_IsSettled() || rdk.active || turn.pending ||
            turn_purpose != TURN_IDLE || turn_issued < mission.inventory.collected || mission.inventory.uncertain ||
            c->argument > 1 || !isfinite(c->x) || c->x < 1 || c->x > BALL_SLOT_COUNT/2 ||
            c->x != (unsigned)c->x || !Turn_StartSteps(&turn, c->argument != 0, (uint8_t)c->x, now)) return false;
        turn_purpose = TURN_UNLOAD;
        return true;
    case PC_GROUP:
        return (mission.step != 13 || (!turn.pending && turn_purpose == TURN_IDLE &&
                turn_issued == mission.inventory.collected)) &&
               (Chassis_IsSettled() || ((c->argument == 2 || c->argument == 105) && mission.step == 8) ||
                (c->argument == 105 && mission.step == 9 && mission.phase == 0) ||
                
                (c->argument == 1 && mission.disc_depart_pending &&
                 (mission.step == 5 || mission.step == 6))) &&
               Rdk_Begin(&rdk, "GROUP", c->argument, now, c->timeout_ms);
    case PC_STAIR:
        if (!Chassis_IsSettled() || mission.id_count >= 64 ||
            !Rdk_Begin(&rdk, "STAIR", c->argument, now, c->timeout_ms)) return false;
        close_rfid_gate();
        disc_action_done_index = disc_rfid_confirmed_index = 0;
        return true;
    case PC_STAIR_SCAN:
        if (!Chassis_IsSettled() || mission.id_count >= 64 ||
            !Rdk_Begin(&rdk, "STAIR_SCAN", c->argument, now, c->timeout_ms)) return false;
        close_rfid_gate();
        disc_action_done_index = disc_rfid_confirmed_index = 0;
        return true;
    case PC_VISION:
        if (!Chassis_IsSettled() || !Rdk_Begin(&rdk, "PILLAR", 0, now, c->timeout_ms)) return false;
        close_rfid_gate();
        disc_action_done_index = disc_rfid_confirmed_index = 0;
        return true;
    case PC_PILLAR_STOPPED:
        return Chassis_IsSettled() && Rdk_PillarStopped(&rdk, (uint8_t)c->argument);
    case PC_PILLAR_END:
        return Rdk_PillarEnd(&rdk); /* Protocol still requires all grab/resume handshakes complete. */
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
            rfid_capture = false;
            disc_action_done_index = 0;
            disc_rfid_confirmed_index = 0;
            disc_waiting_rfid_index = 0;
            rfid_gate_base_count = 0;
            __set_PRIMASK(mask);
        }
        return true;
    case PC_CANCEL:
        cancel_turn();
        close_rfid_gate();
        record_pending_ids();
        Chassis_HoldImmediate();
        motion_pending = false;
        if (!PATH_RDK_ENABLE) return true;
        verified = false;
        return Rdk_Begin(&rdk, "STOP", 0, now, 1);
    default:
        return false;
    }
}
void PathPorts_Init(void)
{
    ir_start=(IrStart){0};
    Turn_Init(&turn, turn_transmit, 0);
    Rdk_Init(&rdk, 0, transmit, 0);
    Path_Init(&mission, send, 0);
    initialized = true;
    if (!PATH_RDK_ENABLE)
    {
        boot = 3;
        ready = verified = true;
        return;
    }
    ready = HAL_UART_Receive_IT(PINCFG_RDK_UART, &rx_byte, 1) == HAL_OK;
    if (PATH_VISION_ENABLE && HAL_UART_Receive_IT(PINCFG_RFID_UART, &rfid_byte, 1) != HAL_OK)
        rfid_fault |= 1;
    if (!ready)
        io_fault |= 1;
}
bool PathPorts_Busy(void)
{
    if (!PATH_RDK_ENABLE) return initialized && mission.result == PATH_RUNNING;
    return initialized && (boot != 3 || side_pending || side_inflight || mission.result == PATH_RUNNING || rdk.active || rdk.locked || turn.pending ||
                           (turn_enabled && turn_issued < mission.inventory.collected));
}
bool PathPorts_Ping(void)
{
    if (!PATH_RDK_ENABLE) return false;
    if (!ready || io_fault || rdk.active || rdk.locked || mission.result == PATH_RUNNING || !Chassis_IsSettled())
        return false;
    verified = false;
    test_ping = true;
    if (boot == 0) boot = 1;
    return Rdk_Begin(&rdk, "HELLO", 0, HAL_GetTick(), 2000);
}
bool PathPorts_SelectSide(bool blue)
{
    /* Chassis-only runs select the same route without an RDK handshake. */
    if (!PATH_RDK_ENABLE) {
        if (!initialized || Chassis_GetState()->armed || !Chassis_IsSettled() ||
            mission.result == PATH_RUNNING) return false;
        requested_blue = blue;
        return true;
    }
    if (!PATH_VISION_ENABLE || !initialized || !ready || io_fault || rdk.locked ||
        Chassis_GetState()->armed || !Chassis_IsSettled() || mission.result == PATH_RUNNING ||
        rdk.warehouse_active || turn.pending) return false;
    requested_blue=blue; side_pending=true; verified=false;
    return true;
}
bool PathPorts_Reset(void)
{
    if (!PATH_RDK_ENABLE) return false;
    if (PINCFG_RDK_UART->gState != HAL_UART_STATE_READY || Chassis_GetState()->armed || !Chassis_IsSettled() || mission.result == PATH_RUNNING ||
        rdk.active || turn.pending)
        return false;
    if (HAL_UART_AbortReceive(PINCFG_RDK_UART) != HAL_OK)
        return false;
    bool rfid_abort_ok = !PATH_VISION_ENABLE || HAL_UART_AbortReceive(PINCFG_RFID_UART) == HAL_OK;
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    head = tail = rfid_head = rfid_tail = 0;
    rfid_length = 0;
    rfid_capture = false;
    disc_action_done_index = disc_rfid_confirmed_index = 0;
    disc_waiting_rfid_index = 0;
    rfid_gate_base_count = 0;
    io_fault = 0;
    rfid_fault = rfid_abort_ok ? 0U : 2U;
    __set_PRIMASK(mask);
    Rdk_Init(&rdk, 0, transmit, 0);
    /* Link recovery does not erase the RFID result; next accepted task does. */
    BallInventory saved_inventory = mission.inventory;
    uint32_t saved_ids[64];
    uint8_t saved_count = mission.id_count;
    bool saved_overflow = mission.id_overflow;
    uint16_t saved_mask = mission.ids;
    memcpy(saved_ids, mission.id_list, sizeof(saved_ids));
    Path_Init(&mission, send, 0);
    memcpy(mission.id_list, saved_ids, sizeof(saved_ids));
    mission.inventory = saved_inventory;
    mission.id_count = saved_count;
    mission.id_overflow = saved_overflow;
    mission.ids = saved_mask;
    Turn_Init(&turn, turn_transmit, 0);
    turn_purpose = TURN_IDLE;
    turn_enabled = false;
    turn_issued = mission.inventory.collected;
    verified = test_ping = stationary = motion_pending = motion_continuous = false;
    boot = 0;
    requested_blue=false; side_pending=PATH_VISION_ENABLE != 0; side_inflight=false;
    boot_retry = HAL_GetTick();
    ready = HAL_UART_Receive_IT(PINCFG_RDK_UART, &rx_byte, 1) == HAL_OK;
    if (PATH_VISION_ENABLE && HAL_UART_Receive_IT(PINCFG_RFID_UART, &rfid_byte, 1) != HAL_OK)
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
    mission.blue = requested_blue;
    stationary = disc_only;
    test_ping = false;
    if (!Path_Start(&mission, HAL_GetTick(), &in)) return false;
    Chassis_BeginPath();
    Turn_Init(&turn, turn_transmit, 0);
    turn_purpose = TURN_IDLE;
    turn_enabled = PATH_VISION_ENABLE && !PATH_SKIP_MATERIAL(&mission);
    turn_issued = 0;
    inventory_fault = 0;
    return true;
}
bool PathPorts_Start(void)
{
    return start(false);
}
bool PathPorts_Disc(void)
{
    if (!PATH_VISION_ENABLE) return false;
    return start(true);
}
void PathPorts_Cancel(void)
{
    if (!initialized)
        return;
    IrStart_Clear(&ir_start);
    if (!PATH_RDK_ENABLE)
    {
        Path_Cancel(&mission);
        return;
    }
    cancel_turn();
    if (boot != 3)
    {
        boot = 4;
        verified = false;
        (void)Rdk_Begin(&rdk, "STOP", 0, HAL_GetTick(), 1);
    }
    if (mission.result == PATH_RUNNING)
        Path_Cancel(&mission);
    else if ((rdk.active && rdk.stage != 1) || rdk.warehouse_active)
    {
        (void)Rdk_Begin(&rdk, "STOP", 0, HAL_GetTick(), 1);
        verified = false;
    }
}
void PathPorts_RxComplete(UART_HandleTypeDef *u)
{
    if (!PATH_RDK_ENABLE) return;
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
    if (!PATH_RDK_ENABLE) return;
    if (u == PINCFG_RDK_UART) {
        rdk_uart_hal_error=HAL_UART_GetError(u);
        ++rdk_uart_error_count;
        io_fault |= 16;
    }
    else if (u == PINCFG_RFID_UART)
        rfid_fault |= 16;
}
void PathPorts_Tick(void)
{
    if (!initialized)
        return;
    uint32_t now = HAL_GetTick();
    if (boot == 0 && ready && !io_fault && !Chassis_GetState()->fault &&
        (int32_t)(now - boot_retry) >= 0 && Chassis_IsSettled())
    {
        if (Rdk_Begin(&rdk, "HELLO", 0, now, 2000)) boot = 1;
    }
    /* Never emit a queued permission after a fault already visible this tick. */
    if (io_fault || Chassis_GetState()->fault ||
        (mission.result == PATH_RUNNING && !Chassis_GetState()->armed))
    {
        if (!rdk.cancel_after_aux) rdk.aux_pending = false;
        if (rdk.active) (void)Rdk_Begin(&rdk, "STOP", 0, now, 1);
        if (boot != 3) boot = 4;
    }
    /* Expire before processing newly received replies; a late reply cannot revive a timeout. */
    rdk.no_timeout=mission.result==PATH_RUNNING && PATH_WAREHOUSE_UNTIMED(&mission);
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
    if (rdk.camera_wait_event) {
        rdk.camera_wait_event=false;
        if (mission.result==PATH_RUNNING && !rdk.locked && mission.step==3 &&
            mission.phase==1 && rdk.stage==4) {
            rdk.started=now; mission.entered=now;
        }
        if (mission.result==PATH_RUNNING && !rdk.locked && !rdk.pillar_ready &&
            ((mission.step==6 && mission.phase==4) || (mission.step==9 && mission.phase==21))) {
            /* Renew only startup deadlines, never motion/grab deadlines. */
            rdk.started=now;
            mission.entered=now;
            if (mission.step==9) mission.stair_started=now;
        }
    }
    uint8_t action_index;
    if (Rdk_TakeDiscActionDone(&rdk, &action_index))
    {
        disc_action_done_index = action_index;
        if (PATH_SKIP_MATERIAL(&mission)) close_rfid_gate();
        else open_rfid_gate(action_index);
    }
    /* Reuse the existing action permission handshake without inventing RFID records.
     * Retry if the auxiliary TX slot is occupied. Never release permission after a fault. */
    if (PATH_SKIP_MATERIAL(&mission) && mission.result==PATH_RUNNING &&
        Chassis_GetState()->armed && !Chassis_GetState()->fault && !io_fault &&
        disc_action_done_index > disc_rfid_confirmed_index &&
        Rdk_SendDiscRfidOk(&rdk,disc_action_done_index))
        disc_rfid_confirmed_index=disc_action_done_index;
    record_pending_ids();
    /* All three capture tasks share this action-complete gate. A missing ID
     * releases permission after a bounded read window, without inventing a UID.
     * Reserve the unknown pocket only after the release was successfully queued. */
    if (disc_waiting_rfid_index && mission.result==PATH_RUNNING &&
        Chassis_GetState()->armed && !Chassis_GetState()->fault && !io_fault && !rdk.locked &&
        (uint32_t)(now-rfid_wait_since)>=PATH_RFID_WAIT_MS &&
        Rdk_SendDiscRfidOk(&rdk,disc_waiting_rfid_index)) {
        if (mission.id_count==rfid_gate_base_count) remember_ball(0,NULL);
        disc_rfid_confirmed_index=disc_waiting_rfid_index;
        close_rfid_gate();
    }
    if (io_fault)
    {
        rdk.locked = true;
        rdk.active = false;
        rdk.stage = 6;
        rdk.error = 4;
        rdk.reply = PATH_FAILED;
    }
    if (boot == 1 && rdk.locked && rdk.error == 1 && !io_fault)
    {
        Rdk_Init(&rdk, 0, transmit, 0);
        boot = 0;
        boot_retry = now + 1000;
    }
    if (boot == 1 && !rdk.active && rdk.stage == 2 && !rdk.locked)
    {
        if (PATH_BLUE_WAREHOUSE_TEST) { boot=3; verified=true; test_ping=false; }
        else if (!Chassis_GetState()->fault && Chassis_IsSettled() &&
            Rdk_Begin(&rdk, "GROUP", 0, now, 30000)) boot = 2;
    }
    else if (boot == 2 && !rdk.active && rdk.reply == PATH_OK && !rdk.locked)
    {
        boot = 3;
        verified = true;
        test_ping = false;
    }
    if (boot == 3 && test_ping && rdk.stage == 2 && !rdk.active)
    {
        verified = true;
        test_ping = false;
    }
    if (side_inflight && !rdk.active && !rdk.locked && rdk.reply==PATH_OK) {
        side_inflight=false;
        side_pending=(requested_blue != (rdk.group != 0));
        verified=!side_pending;
    }
    if (boot==3 && side_pending && !side_inflight && !rdk.active && !rdk.locked) {
        verified=false;
        if (Rdk_Begin(&rdk,"COLOR",requested_blue,now,2000)) side_inflight=true;
    }
    if (side_pending || side_inflight || (PATH_RDK_ENABLE && rdk.locked))
        verified = false;
    bool motion_done = false;
    if (motion_pending)
    {
        if (!PATH_WAREHOUSE_UNTIMED(&mission) && (uint32_t)(now - motion_since) >= motion_timeout)
        {
            io_fault |= 32;
            Chassis_HoldImmediate();
            motion_pending = false;
            motion_continuous = false;
        }
        else if (!Chassis_MotionBusy())
        {
            if (!motion_continuous)
                Chassis_Hold();
            else
                motion_done = true;
            motion_pending = false;
            motion_continuous = false;
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
    /* Boot HELLO/G0/COLOR already prepares vision; no host ARM/PATH is needed. */
    const ChassisState *chassis=Chassis_GetState();
    bool observe_start=!io_fault && !chassis->fault && !rdk.locked && boot!=4 &&
        !chassis->armed && mission.result==PATH_IDLE;
    if (!observe_start) IrStart_Clear(&ir_start);
    else {
        (void)IrStart_Update(&ir_start,now,ir_raw==0,false);
        if (ir_start.pending && requested_blue!=ir_start.blue)
            (void)PathPorts_SelectSide(ir_start.blue);
        bool start_ready=ready && verified && !io_fault &&
            requested_blue==ir_start.blue && !PathPorts_Busy() &&
            !chassis->armed && !chassis->fault && chassis->bias_ready && Chassis_IsSettled();
        if (IrStart_Update(&ir_start,now,ir_raw==0,start_ready) && Chassis_Arm()) {
            if (PathPorts_Start()) ir_start.fired=true;
            else Chassis_Stop();
        }
    }
    PathInput in = {.armed = Chassis_GetState()->armed,
                    .fault = io_fault || Chassis_GetState()->fault ||
                             (PATH_RDK_ENABLE && rdk.locked && !(rdk.error == 1 && disc_deadline)),
                    .settled = Chassis_IsSettled(),
                    .motion_done = motion_done,
                    .gray = gray,
                    .travel_rpm = hypotf(chassis->velocity[0],chassis->velocity[1])*60.0f /
                                  (6.28318530718f*chassis_config.wheel_radius_mm),
                    .yaw_deg = Chassis_ContinuousYaw() * 57.295779513f,
                    .x_mm = Chassis_GetState()->x_mm,
                    .y_mm = Chassis_GetState()->y_mm,
                    .map_yaw_deg = Chassis_MapYaw(),
                    .imu_yaw_deg = Chassis_LineYaw(),
                    .ir = ir_raw == 0,
                    .warehouse_vision = PATH_RDK_ENABLE != 0,
                    .destack_enabled = PATH_DESTACK_ENABLE && PATH_RDK_ENABLE != 0 && PATH_VISION_ENABLE && !PATH_SKIP_MATERIAL(&mission),
                    .warehouse_ready = rdk.warehouse_ready,
                    .warehouse_digit = rdk.warehouse_digit,
                    .warehouse_digit_reply = rdk.warehouse_reply,
                    .vision_ready = rdk.pillar_ready,
                    .disc_completed = disc_action_done_index,
                    .ball_index = rdk.ball_index,
                    .resume_index = rdk.resume_index,
                    .reply = rdk.reply,
                    .turn_reply = turn.reply};
    PathResult previous = mission.result;
    unsigned phase = mission.phase;
    Chassis_SetRoutePolicy(PathPolicy_Chassis(mission.blue,mission.result,mission.step,mission.phase));
    Path_Tick(&mission, now, &in);
    if (stationary && phase == 99 && mission.phase == 0 && mission.result == PATH_RUNNING)
    {
        mission.step = 3;
        mission.phase = 3; /* Start DISC after the departure G100 completes. */
        mission.entered = now;
    }
    if ((!PATH_BLUE_DISC_TEST || PATH_BLUE_PILLAR_TEST) && !stationary && previous == PATH_RUNNING && mission.result == PATH_DONE && mission.step == 3)
    {
        mission.result = PATH_RUNNING;
        mission.step = 4;
        mission.phase = 0;
        mission.entered = now;
        mission.waiting = mission.stable = false;
        PathChassis_Tick(&mission,now,&in); /* DISC_DONE -> G1 + travel without idle ticks. */
    }
    if (mission.result != PATH_RUNNING ||
        (mission.step != 3 && mission.step != 6 && mission.step != 9) || rdk.locked)
        close_rfid_gate();
    if (previous == PATH_RUNNING && mission.result >= PATH_CANCELED)
    {
        Chassis_HoldImmediate();
        motion_pending = false;
        if (PATH_RDK_ENABLE) verified = false;
        if (PATH_RDK_ENABLE && !rdk.locked)
            (void)Rdk_Begin(&rdk, "STOP", 0, now, 1);
    }
    Chassis_SetRoutePolicy(PathPolicy_Chassis(mission.blue,mission.result,mission.step,mission.phase));
    service_turn(now);
    path_diagnostics = (PathDiagnostics){.result = mission.result,
                                         .inventory_fault = inventory_fault,
                                         .inventory_occupied = mission.inventory.occupied,
                                         .inventory_slot = mission.inventory.current,
                                         .inventory_uncertain = mission.inventory.uncertain,
                                         .warehouse_placed = mission.inventory.placed,
                                         .warehouse_code = mission.step == 13 && mission.point < 9 ?
                                              PathWarehouse_Code(&mission) : 0,
                                         .blue = mission.blue,
                                         .rdk_uart_hal_error=rdk_uart_hal_error,
                                         .rdk_uart_error_count=rdk_uart_error_count,
                                         .step = mission.step,
                                         .phase = mission.phase,
                                         .point = mission.step == 9 ? mission.point + 1U : 0U,
                                         .phase_elapsed_ms = now - mission.entered,
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
                                         .gray = gray,
                                         .disc_action_done_index = disc_action_done_index,
                                         .disc_rfid_confirmed_index = disc_rfid_confirmed_index,
                                         .disc_waiting_rfid = disc_waiting_rfid_index,
                                         .disc_action_allowed = mission.result == PATH_RUNNING &&
                                             mission.step == 3 && mission.phase == 1 &&
                                             !rdk.locked && !disc_waiting_rfid_index &&
                                             disc_action_done_index < DISC_REQUIRED_RFID_COUNT &&
                                             disc_action_done_index == disc_rfid_confirmed_index};
}
