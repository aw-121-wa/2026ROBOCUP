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
static unsigned moves, holds, turn_frames;
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
static void tick(void)
{
    now += 5;
    PathPorts_Tick();
}
static void reply(const char *status)
{
    unsigned sid, seq;
    char line[80];
    if (sscanf(wire, "Q %u %u", &sid, &seq) != 2)
        return;
    snprintf(line, sizeof(line), "R %u %u %s\n", sid, seq, status);
    for (const char *p = line; *p; p++)
    {
        *rx4 = (uint8_t)*p;
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
static int start(void)
{
    PathPorts_Init();
    CHECK(!PathPorts_Start());
    state.armed = true;
    CHECK(PathPorts_Start());
    CHECK(!PathPorts_Start());
    tick();
    CHECK(strstr(wire, "HELLO 0") != 0);
    CHECK(moves == 0);
    reply("DONE");
    tick();
    tick();
    tick();
    CHECK(strstr(wire, "GROUP 0") != 0 && moves == 0);
    reply("DONE");
    tick();
    tick();
    CHECK(moves == 1 && moving);
    return 0;
}
static int resume_after_stop(void)
{
    CHECK(start() == 0);
    PathPorts_Cancel();
    tick();
    CHECK(strstr(wire, "STOP 0") != 0);
    CHECK(PathPorts_Busy());
    CHECK(!PathPorts_Start());
    unsigned count = moves;
    for (int i = 0; i < 20; i++)
        tick();
    CHECK(moves == count);
    reply("DONE");
    tick();
    CHECK(!PathPorts_Busy());
    CHECK(Chassis_Move(100, 0, 100, 100, 100));
    tick();
    CHECK(moving); /* terminal PATH must not silently cancel manual driving */
    CHECK(path_diagnostics.result == PATH_CANCELED);
    return 0;
}
static int rfid_waits_for_drop(void)
{
    CHECK(start() == 0);
    for (int i = 0; i < 200 && !strstr(wire, "DISC "); i++)
    {
        moving = false;
        if (strstr(wire, "GROUP 101"))
            reply("DONE");
        tick();
    }
    CHECK(strstr(wire, "DISC ") != 0);
    *rx8 = 1;
    PathPorts_RxComplete(&huart8);
    tick();
    CHECK(path_diagnostics.accepted_ids == 0 && turn_frames == 0);
    for (int i = 0; i < 20; i++)
        tick();
    CHECK(turn_frames == 0);
    reply("DONE");
    tick();
    tick();
    tick();
    CHECK(path_diagnostics.accepted_ids == 1 && turn_frames > 0);
    *rx8 = 1;
    PathPorts_RxComplete(&huart8);
    for (int i = 0; i < 200; i++)
        tick();
    CHECK(path_diagnostics.accepted_ids == 1);
    return 0;
}
int main(int argc, char **argv)
{
    CHECK(argc == 2);
    if (!strcmp(argv[1], "rejected"))
    {
        PathPorts_Init();
        state.armed = true;
        CHECK(PathPorts_Start());
        tick();
        reply("DONE");
        tick();
        tick();
        tick();
        CHECK(strstr(wire, "GROUP 0") != 0);
        reply("DONE");
        tick();
        moving = true;
        tick();
        CHECK(!moving);
        tick();
        CHECK(strstr(wire, "STOP 0") != 0);
        CHECK(path_diagnostics.result == PATH_ERROR);
        return 0;
    }
    int result = !strcmp(argv[1], "resume") ? resume_after_stop() : rfid_waits_for_drop();
    if (!result)
        puts("PATH adapter integration test passed");
    return result;
}
