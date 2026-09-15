#include "path_ports.h"
#include "path_mission.h"
#include "rdk_link.h"
#include "turntable_link.h"
#include "chassis_control.h"
#include "pin_config.h"
#include "path_session.h"
#include <string.h>

static PathMission mission;
static RdkLink rdk;
static TurntableLink table;
volatile PathDiagnostics path_diagnostics;
static bool initialized, ready, canceled, motion_pending;
static uint32_t motion_since, motion_timeout;
static uint32_t last_rx;
static volatile uint32_t io_fault;
static uint8_t rdk_byte, rfid_byte, rdk_tx[80], turn_tx[13];
static volatile uint8_t ring[128];
static volatile unsigned head, tail;
static volatile uint16_t rfid_ids;

static bool rdk_transmit(void *ctx, const char *data, size_t length)
{
    (void)ctx;
    if (PINCFG_RDK_UART->gState != HAL_UART_STATE_READY)
        return false;
    memcpy(rdk_tx, data, length);
    return HAL_UART_Transmit_IT(PINCFG_RDK_UART, rdk_tx, (uint16_t)length) == HAL_OK;
}
static bool turn_transmit(void *ctx, const uint8_t *data, size_t length)
{
    (void)ctx;
    if (PINCFG_TURNTABLE_UART->gState != HAL_UART_STATE_READY)
        return false;
    memcpy(turn_tx, data, length);
    return HAL_UART_Transmit_IT(PINCFG_TURNTABLE_UART, turn_tx, (uint16_t)length) == HAL_OK;
}
static bool send(void *ctx, const PathCommand *c)
{
    (void)ctx;
    uint32_t now = HAL_GetTick();
    float scale = 2.0f * 3.141592654f * chassis_config.wheel_radius_mm / 60.0f;
    const char *verb = 0;
    uint32_t timeout = c->timeout_ms;
    switch (c->kind)
    {
    case PC_MOVE:
        if (!Chassis_Move(c->x, c->y, c->speed * scale, 550, 550))
            return false;
        motion_pending = true;
        motion_since = now;
        motion_timeout = timeout;
        return true;
    case PC_ROTATE:
        if (!Chassis_Rotate(c->x))
            return false;
        motion_pending = true;
        motion_since = now;
        motion_timeout = timeout;
        return true;
    case PC_BODY:
        if (!Chassis_Body(c->x * scale, c->y * scale,
                          c->speed * scale /
                              (chassis_config.half_track_mm + chassis_config.half_wheelbase_mm)))
            return false;
        if (!motion_pending)
        {
            motion_since = now;
            motion_timeout = 15000;
        }
        motion_pending = true;
        return true;
    case PC_HOLD:
        Chassis_Hold();
        motion_pending = false;
        return true;
    case PC_TURN:
        if (!Chassis_IsSettled())
            return false;
        return Turn_Start(&table, c->argument != 0, now);
    case PC_GROUP:
    case PC_DISC:
        if (!Chassis_IsSettled() || table.pending)
            return false;
        verb = c->kind == PC_GROUP ? "GROUP" : "DISC";
        if (c->kind == PC_DISC)
            timeout += 31000; /* recognition budget + full G102 + wire margin */
        else
            timeout += 1000;
        break;
    case PC_VISION:
        verb = "VISION";
        timeout += 2000;
        break;
    case PC_HELLO:
        verb = "HELLO";
        break;
    case PC_CANCEL:
        Chassis_Hold();
        motion_pending = false;
        if (table.pending)
            Turn_Stop(&table, now);
        canceled = true;
        verb = "STOP";
        timeout += 1000;
        break;
    default:
        return false;
    }
    uint32_t arg = (c->kind == PC_DISC || c->kind == PC_VISION) ? c->timeout_ms : c->argument;
    bool accepted = Rdk_Begin(&rdk, verb, arg, now, timeout);
    if (accepted && c->kind != PC_CANCEL)
        canceled = false;
    return accepted;
}
void PathPorts_Init(void)
{
    uint32_t session = PathSession_Create();
    Rdk_Init(&rdk, session, rdk_transmit, 0);
    Turn_Init(&table, turn_transmit, 0);
    Path_Init(&mission, send, 0);
    initialized = true;
    ready = session != 0 && HAL_UART_Receive_IT(PINCFG_RDK_UART, &rdk_byte, 1) == HAL_OK &&
            HAL_UART_Receive_IT(PINCFG_RFID_UART, &rfid_byte, 1) == HAL_OK;
    if (!ready)
        io_fault |= 1;
}
bool PathPorts_Busy(void)
{
    return initialized &&
           (mission.result == PATH_RUNNING || rdk.active || table.pending || rdk.locked);
}
bool PathPorts_Start(void)
{
    if (!ready || io_fault || PathPorts_Busy() || !Chassis_IsSettled())
        return false;
    canceled = false;
    PathInput in = {.armed = Chassis_GetState()->armed,
                    .fault = Chassis_GetState()->fault != 0,
                    .settled = true};
    return Path_Start(&mission, HAL_GetTick(), &in);
}
void PathPorts_Cancel(void)
{
    if (!initialized)
        return;
    if (mission.result == PATH_RUNNING)
        Path_Cancel(&mission);
    else if (!canceled && (rdk.active || table.pending))
    {
        PathCommand c = {.kind = PC_CANCEL, .timeout_ms = 30000};
        (void)send(0, &c);
    }
}
void PathPorts_RxComplete(UART_HandleTypeDef *uart)
{
    if (uart == PINCFG_RDK_UART)
    {
        unsigned next = (head + 1U) % sizeof(ring);
        if (next == tail)
            io_fault |= 2;
        else
        {
            ring[head] = rdk_byte;
            head = next;
        }
        if (HAL_UART_Receive_IT(uart, &rdk_byte, 1) != HAL_OK)
            io_fault |= 4;
    }
    else if (uart == PINCFG_RFID_UART)
    {
        if (rfid_byte >= 1 && rfid_byte <= 9)
            rfid_ids |= (uint16_t)(1U << rfid_byte);
        if (HAL_UART_Receive_IT(uart, &rfid_byte, 1) != HAL_OK)
            io_fault |= 8;
    }
}
void PathPorts_Error(UART_HandleTypeDef *uart)
{
    if (uart == PINCFG_RDK_UART || uart == PINCFG_RFID_UART || uart == PINCFG_TURNTABLE_UART)
        io_fault |= 16;
}
void PathPorts_Tick(void)
{
    if (!initialized)
        return;
    uint32_t now = HAL_GetTick();
    unsigned budget = sizeof(ring);
    if (rdk.length && (uint32_t)(now - last_rx) >= 500)
        rdk.overflow = true;
    while (tail != head && budget--)
    {
        uint8_t b = ring[tail];
        tail = (tail + 1U) % sizeof(ring);
        Rdk_Feed(&rdk, b);
        last_rx = now;
    }
    if (io_fault || Chassis_GetState()->fault || !Chassis_GetState()->armed)
        PathPorts_Cancel();
    Rdk_Tick(&rdk, now);
    Turn_Tick(&table, now, PINCFG_TURNTABLE_UART->gState == HAL_UART_STATE_READY);
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
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    uint16_t ids = rfid_ids;
    rfid_ids = 0;
    __set_PRIMASK(mask);
    uint8_t gray = 0;
    if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_8) == GPIO_PIN_RESET)
        gray |= 8;
    if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_0) == GPIO_PIN_RESET)
        gray |= 4;
    if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_1) == GPIO_PIN_RESET)
        gray |= 2;
    if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_3) == GPIO_PIN_RESET)
        gray |= 1;
    PathInput in = {.armed = Chassis_GetState()->armed,
                    .fault = io_fault || Chassis_GetState()->fault || rdk.locked ||
                             table.reply == PATH_FAILED,
                    .settled = Chassis_IsSettled(),
                    .yaw_deg = Chassis_ContinuousYaw() * 57.295779513f,
                    .gray = gray,
                    .ir = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_10) == GPIO_PIN_RESET,
                    .rfid = ids,
                    .reply = rdk.reply,
                    .turn_reply = table.reply,
                    .interrupted_reply = rdk.interrupted_reply};
    PathResult previous_result = mission.result;
    Path_Tick(&mission, now, &in);
    /* Dispatch rejection can set ERROR before the interpreter sends its normal
     * cleanup. Run cleanup once at this boundary, never on later manual ticks. */
    if (previous_result == PATH_RUNNING && mission.result >= PATH_CANCELED)
    {
        Chassis_Hold();
        motion_pending = false;
        if (!canceled)
        {
            PathCommand stop = {.kind = PC_CANCEL, .timeout_ms = 30000};
            (void)send(0, &stop);
        }
    }
    path_diagnostics = (PathDiagnostics){mission.result,
                                         mission.step,
                                         mission.phase,
                                         mission.ids,
                                         (uint32_t)__builtin_popcount(mission.ids),
                                         rdk.reply,
                                         table.reply,
                                         io_fault,
                                         rdk.session,
                                         rdk.sequence};
}
