#include "path_ports.h"
#include "path_mission.h"
#include "chassis_control.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

UART_HandleTypeDef huart4 = {0, 4}, huart6 = {0, 6}, huart8 = {0, 8};
ChassisConfig chassis_config = {
    .wheel_radius_mm = 35, .half_track_mm = 128.5f, .half_wheelbase_mm = 130.5f};
static ChassisState state;
static uint32_t now;
static uint8_t *rx4, *rx8;
static char wire[80];
static unsigned moves, holds, turn_frames, outer_reads;
static float rotation;
static bool moving;
uint32_t HAL_GetTick(void)
{
    return now;
}
uint32_t PathSession_Create(void)
{
    return 123;
}
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *u, uint8_t *b, uint16_t n)
{
    if (n != 1)
        return HAL_ERROR;
    if (u == &huart4)
        rx4 = b;
    else if (u == &huart8)
        rx8 = b;
    else
        return HAL_ERROR;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *u, uint8_t *b, uint16_t n)
{
    if (u == &huart4)
    {
        if (n >= sizeof(wire))
            return HAL_ERROR;
        memcpy(wire, b, n);
        wire[n] = 0;
    }
    else if (u == &huart6)
        turn_frames++;
    else
        return HAL_ERROR;
    return HAL_OK;
}
unsigned HAL_GPIO_ReadPin(void *port, uint16_t pin)
{
    (void)port;
    if (pin != GPIO_PIN_0 && pin != GPIO_PIN_1)
        outer_reads++;
    return (pin == GPIO_PIN_0 || pin == GPIO_PIN_1 || pin == GPIO_PIN_10) ? GPIO_PIN_RESET
                                                                          : GPIO_PIN_SET;
}
const ChassisState *Chassis_GetState(void)
{
    return &state;
}
bool Chassis_MotionBusy(void)
{
    return moving;
}
bool Chassis_IsSettled(void)
{
    return !moving;
}
float Chassis_ContinuousYaw(void)
{
    return 0;
}
void Chassis_Hold(void)
{
    moving = false;
    holds++;
}
bool Chassis_Move(float x, float y, float v, float a, float d)
{
    (void)x;
    (void)y;
    (void)v;
    (void)a;
    (void)d;
    if (!state.armed || moving)
        return false;
    moving = true;
    moves++;
    return true;
}
bool Chassis_Rotate(float deg)
{
    rotation = deg;
    return Chassis_Move(deg, 0, 1, 1, 1);
}
bool Chassis_Body(float x, float y, float w)
{
    (void)x;
    (void)y;
    (void)w;
    moving = true;
    return state.armed;
}

HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *u)
{
    return u == &huart4 ? HAL_OK : HAL_ERROR;
}
static void tick(void)
{
    now += 5;
    PathPorts_Tick();
}
static void feed(const char *s)
{
    while (*s)
    {
        *rx4 = (uint8_t)*s++;
        PathPorts_RxComplete(&huart4);
    }
}
#define CHECK(x)                                                                                   \
    do                                                                                             \
    {                                                                                              \
        if (!(x))                                                                                  \
        {                                                                                          \
            printf("FAIL %d: %s\n", __LINE__, #x);                                                 \
            return 1;                                                                              \
        }                                                                                          \
    } while (0)
int main(void)
{
    PathPorts_Init();
    tick();
    CHECK(rx4 && !rx8);
    state.armed = true;
    CHECK(!PathPorts_Start()); /* Explicit PING proof is mandatory. */
    state.armed = false;
    CHECK(PathPorts_Ping());
    tick();
    CHECK(!strcmp(wire, "PING\r\n"));
    PathPorts_Cancel(); /* A chassis stop must not cancel a standalone UART probe. */
    feed("PONG\r\n");
    tick();
    CHECK(path_diagnostics.accepted_ids == 1 && path_diagnostics.link_stage == 2);
    CHECK(moves == 0 && !state.armed && !PathPorts_Busy());
    state.armed = true;
    CHECK(PathPorts_Start());
    tick();
    CHECK(!strcmp(wire, "PING\r\n"));
    feed("PONG\r\n");
    tick();
    tick();
    CHECK(moves == 1);
    moving = false;
    tick();
    tick();
    CHECK(moves == 2);
    moving = false;
    tick();
    tick();
    CHECK(moves == 3 && fabsf(rotation - 180) < .001f);
    moving = false;
    tick();
    for (int i = 0; i < 20; i++)
        tick();
    CHECK(!strcmp(wire, "DISC_START\r\n"));
    CHECK(turn_frames == 0 && outer_reads == 0);
    feed("DISC_ACK\r\n");
    tick();
    CHECK(path_diagnostics.link_stage == 4);
    feed("DISC_DONE\r\n");
    tick();
    CHECK(path_diagnostics.result == PATH_DONE && !PathPorts_Busy());
    for (int i = 0; i < 100; i++)
    {
        tick();
    }
    CHECK(moves == 3);
    state.armed = false;
    CHECK(PathPorts_Reset());
    CHECK(PathPorts_Ping());
    tick();
    now += 2000;
    tick();
    CHECK(path_diagnostics.link_error == 1 && PathPorts_Busy());
    CHECK(PathPorts_Reset());
    CHECK(PathPorts_Ping());
    tick();
    feed("PONG\r\n");
    tick();
    CHECK(path_diagnostics.accepted_ids == 1);
    CHECK(!state.armed);
    return 0;
}
