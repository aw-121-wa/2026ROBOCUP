#include "path_ports.h"
#include "path_mission.h"
#include "rdk_link.h"
#include "chassis_control.h"
#include "pin_config.h"
#include <string.h>
static PathMission mission;
static RdkLink rdk;
volatile PathDiagnostics path_diagnostics;
static bool ready, initialized, motion_pending, verified, test_ping, stationary;
static uint32_t motion_since, motion_timeout, last_rx;
static volatile uint32_t io_fault;
static uint8_t rx_byte, tx_buffer[80];
static volatile uint8_t ring[128];
static volatile unsigned head, tail;
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
        if (!Chassis_Body(c->x * scale, c->y * scale, 0))
            return false;
        if (!motion_pending)
        {
            motion_since = now;
            motion_timeout = 5000;
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
        return Rdk_Begin(&rdk, "DISC", 0, now, c->timeout_ms);
    case PC_CANCEL:
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
    Rdk_Init(&rdk, 0, transmit, 0);
    Path_Init(&mission, send, 0);
    initialized = true;
    ready = HAL_UART_Receive_IT(PINCFG_RDK_UART, &rx_byte, 1) == HAL_OK;
    if (!ready)
        io_fault |= 1;
}
bool PathPorts_Busy(void)
{
    return initialized && (mission.result == PATH_RUNNING || rdk.active || rdk.locked);
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
        rdk.active)
        return false;
    if (HAL_UART_AbortReceive(PINCFG_RDK_UART) != HAL_OK)
        return false;
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    head = tail = 0;
    io_fault = 0;
    __set_PRIMASK(mask);
    Rdk_Init(&rdk, 0, transmit, 0);
    Path_Init(&mission, send, 0);
    verified = test_ping = stationary = motion_pending = false;
    ready = HAL_UART_Receive_IT(PINCFG_RDK_UART, &rx_byte, 1) == HAL_OK;
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
    return Path_Start(&mission, HAL_GetTick(), &in);
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
    PathInput in = {.armed = Chassis_GetState()->armed,
                    .fault = io_fault || Chassis_GetState()->fault || rdk.locked,
                    .settled = Chassis_IsSettled(),
                    .gray = gray,
                    .reply = rdk.reply};
    PathResult previous = mission.result;
    unsigned phase = mission.phase;
    Path_Tick(&mission, now, &in);
    if (stationary && phase == 99 && mission.phase == 0 && mission.result == PATH_RUNNING)
    {
        mission.step = 3;
        mission.phase = 1;
        mission.entered = now;
        PathCommand c = {.kind = PC_DISC, .timeout_ms = 180000};
        if (!send(0, &c))
            mission.result = PATH_ERROR;
    }
    if (previous == PATH_RUNNING && mission.result >= PATH_CANCELED)
    {
        Chassis_Hold();
        motion_pending = false;
        verified = false;
        if (!rdk.locked)
            (void)Rdk_Begin(&rdk, "STOP", 0, now, 1);
    }
    path_diagnostics = (PathDiagnostics){.result = mission.result,
                                         .step = mission.step,
                                         .phase = mission.phase,
                                         .accepted_ids = verified,
                                         .link_reply = rdk.reply,
                                         .fault = io_fault,
                                         .sequence = rdk.sequence,
                                         .link_stage = rdk.stage,
                                         .link_error = rdk.error,
                                         .gray = gray};
}
