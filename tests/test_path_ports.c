#include "disc_task_config.h"
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
static bool moving, disc_only, rfid_init_failure;
static float yaw;
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
    {
        rx8 = b;
        if (rfid_init_failure) return HAL_ERROR;
    }
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
    return ((port == GPIOD && (pin == GPIO_PIN_0 || pin == GPIO_PIN_1)) ||
            (port == GPIOD && pin == GPIO_PIN_10)) ? GPIO_PIN_RESET
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
    return yaw / 57.295779513f;
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
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *u)
{
    return (u == &huart4 || u == &huart8) ? HAL_OK : HAL_ERROR;
}
static void tick(void)
{
    now += 5;
    PathPorts_Tick();
}
static void reply(const char *line)
{
    for (const char *p=line; *p; p++) { *rx4=(uint8_t)*p; PathPorts_RxComplete(&huart4); }
}
static void raw(uint8_t value) { *rx8=value; PathPorts_RxComplete(&huart8); }
static void id(uint32_t value) {
    uint8_t f[12]={4,12,2,32,0,4,0,
        (uint8_t)(value>>24),(uint8_t)(value>>16),(uint8_t)(value>>8),(uint8_t)value,0};
    f[11]=255; for(unsigned i=0;i<11;i++) f[11]^=f[i];
    for(unsigned i=0;i<12;i++) raw(f[i]);
}
#define CHECK(x) do { if (!(x)) { printf("FAIL %d: %s\n",__LINE__,#x); return 1; } } while(0)
static int start(void)
{
    PathPorts_Init(); CHECK(!PathPorts_Start());
    state.armed=true; CHECK(!PathPorts_Start()); state.armed=false;
    CHECK(PathPorts_Ping()); tick(); CHECK(!strcmp(wire,"PING\r\n"));
    PathPorts_Cancel(); reply("PONG\r\n"); tick();
    CHECK(path_diagnostics.accepted_ids==1 && !PathPorts_Busy());
    state.armed=true;
    id(8); /* Pre-task bytes must not contaminate the disc list. */
    CHECK(disc_only ? PathPorts_Disc() : PathPorts_Start()); CHECK(!PathPorts_Start()); tick();
    CHECK(!strcmp(wire,"PING\r\n") && moves==0);
    reply("PONG\r\n"); tick(); tick();
    CHECK(disc_only ? moves==0 : (moves==1 && moving));
    for (unsigned i=0;i<200 && strcmp(wire,"DISC_START\r\n");i++) { moving=false; tick(); }
    CHECK(!strcmp(wire,"DISC_START\r\n"));
    CHECK(path_diagnostics.rfid_count==0);
    reply("DISC_ACK\r\n"); tick(); return 0;
}
static void five_ids(void) { id(3); id(1); id(3); id(9); id(2); id(7); }
int main(int argc,char **argv)
{
    CHECK(argc==2); rfid_init_failure = !strcmp(argv[1],"rfid_init"); disc_only = !strcmp(argv[1],"stationary"); CHECK(start()==0);
    if (!strcmp(argv[1],"pending_stop")) {
        five_ids(); PathPorts_Cancel(); tick();
        CHECK(path_diagnostics.result==PATH_CANCELED && path_diagnostics.rfid_count==5);
        uint32_t saved[9]={0}; const uint32_t expected[]={3,1,9,2,7};
        CHECK(PathPorts_CopyIds(saved,9)==5 && !memcmp(saved,expected,sizeof(expected)));
    } else if (!strcmp(argv[1],"frames")) {
        const uint8_t sample[]={4,12,2,32,0,4,0,0x45,0x96,0xb7,0x8a,0x3f};
        for(unsigned i=0;i<6;i++) raw(sample[i]);
        tick();
        CHECK(path_diagnostics.rfid_count==0);
        for(unsigned i=6;i<12;i++) raw(sample[i]);
        tick();
        CHECK(path_diagnostics.rfid_count==1);
        for(unsigned i=0;i<12;i++) raw(sample[i] ^ (i==11 ? 1 : 0));
        tick(); CHECK(path_diagnostics.rfid_count==1);
        uint8_t block[22]={4,22,3,32,0};
        memcpy(block+5,sample,sizeof(sample));
        block[21]=255; for(unsigned i=0;i<21;i++) block[21]^=block[i];
        for(unsigned i=0;i<22;i++) raw(block[i]);
        tick(); CHECK(path_diagnostics.rfid_count==1);
        id(0x4596b78a); id(0xffffffff); id(0); tick();
        uint32_t saved[64]; CHECK(PathPorts_CopyIds(saved,64)==3);
        CHECK(saved[0]==0x4596b78a && saved[1]==0xffffffff && saved[2]==0);
        uint8_t combined[28]={4,28,4,32,0,4,0,1,2,3,4};
        combined[27]=255; for(unsigned i=0;i<27;i++) combined[27]^=combined[i];
        for(unsigned i=0;i<28;i++) raw(combined[i]);
        tick();
        CHECK(path_diagnostics.rfid_count==4);
        for(unsigned i=1;i<=70;i++) { id(i); tick(); }
        CHECK(path_diagnostics.rfid_count==64 && (path_diagnostics.rfid_fault & 128));
        CHECK(path_diagnostics.fault==0 && path_diagnostics.result==PATH_RUNNING);
    } else if (!strcmp(argv[1],"stationary")) {
        five_ids(); reply("DISC_DONE\r\n"); tick();
        CHECK(path_diagnostics.result==PATH_DONE && path_diagnostics.step==3);
        CHECK(path_diagnostics.rfid_count==5 && moves==0 && !PathPorts_Busy());
        for(unsigned i=0;i<20;i++) tick();
        CHECK(moves==0);
        state.armed=false; CHECK(PathPorts_Reset()); tick();
        uint32_t saved[9]={0}; const uint32_t expected[]={3,1,9,2,7};
        CHECK(PathPorts_CopyIds(saved,9)==5 && !memcmp(saved,expected,sizeof(expected)));
        CHECK(path_diagnostics.accepted_ids==0);
        CHECK(!PathPorts_Disc()); CHECK(PathPorts_CopyIds(saved,9)==5);
        CHECK(PathPorts_Ping()); tick(); reply("PONG\r\n"); tick();
        state.armed=true; CHECK(PathPorts_Disc()); CHECK(PathPorts_CopyIds(saved,9)==0);
    } else if (!strcmp(argv[1],"late")) {
        five_ids(); tick(); now+=DISC_TASK_TIMEOUT_MS;
        reply("DISC_DONE\r\n"); tick();
        CHECK(path_diagnostics.result==PATH_TIMEOUT && !moving);
        CHECK(path_diagnostics.rfid_count==5);
    } else if (!strcmp(argv[1],"stop")) {
        five_ids(); tick(); PathPorts_Cancel(); tick();
        CHECK(path_diagnostics.result==PATH_CANCELED);
        CHECK(path_diagnostics.rfid_count==5 && PathPorts_Busy());
        CHECK(!PathPorts_Start() && !moving);
        CHECK(!strcmp(wire,"DISC_START\r\n"));
        reply("DISC_DONE\r\n"); tick();
        CHECK(path_diagnostics.result==PATH_CANCELED && !moving);
    } else if (!strcmp(argv[1],"missing")) {
        id(3); id(3); id(1); id(9); id(2); tick();
        reply("DISC_DONE\r\n"); tick();
        CHECK(path_diagnostics.rfid_count==4 && path_diagnostics.step==4);
        CHECK(path_diagnostics.result==PATH_RUNNING && turn_frames==0);
    } else if (!strcmp(argv[1],"timeout")) {
        five_ids(); tick(); CHECK(path_diagnostics.rfid_count==5);
        CHECK(path_diagnostics.step==3); /* IDs alone cannot finish the RDK job. */
        now+=DISC_TASK_TIMEOUT_MS; tick();
        CHECK(path_diagnostics.result==PATH_TIMEOUT && !moving);
        reply("DISC_DONE\r\n"); id(8); tick();
        CHECK(path_diagnostics.result==PATH_TIMEOUT && path_diagnostics.rfid_count==5);
        CHECK(PathPorts_Busy() && !PathPorts_Start());
    } else if (!strcmp(argv[1],"overflow")) {
        for(unsigned i=0;i<256;i++) id(1);
        tick(); CHECK(path_diagnostics.result==PATH_RUNNING);
        CHECK(path_diagnostics.fault==0 && (path_diagnostics.rfid_fault & 64));
        reply("DISC_DONE\r\n"); tick();
        CHECK(path_diagnostics.step==4 && path_diagnostics.result==PATH_RUNNING);
    } else if (!strcmp(argv[1],"rfid_fault") || !strcmp(argv[1],"rfid_init") || !strcmp(argv[1],"zero")) {
        if (!strcmp(argv[1],"rfid_fault")) PathPorts_Error(&huart8);
        reply("DISC_DONE\r\n"); tick();
        CHECK(path_diagnostics.step==4 && path_diagnostics.result==PATH_RUNNING);
        CHECK(path_diagnostics.fault==0 && path_diagnostics.rfid_count==0);
        if (!strcmp(argv[1],"rfid_fault")) CHECK(path_diagnostics.rfid_fault & 16);
        if (!strcmp(argv[1],"rfid_init")) CHECK(path_diagnostics.rfid_fault & 1);
    } else if (!strcmp(argv[1],"error")) {
        id(2); tick(); reply("DISC_ERROR\r\n"); tick();
        CHECK(path_diagnostics.result==PATH_ERROR && path_diagnostics.rfid_count==1);
    } else {
        five_ids(); id(4); tick();
        CHECK(path_diagnostics.rfid_count==6 && path_diagnostics.step==3);
        uint32_t saved[9]={0};
        const uint32_t expected[]={3,1,9,2,7,4};
        CHECK(PathPorts_CopyIds(saved,9)==6);
        CHECK(!memcmp(saved,expected,sizeof(expected)));
        CHECK(PathPorts_CopyIds(saved,2)==2 && saved[0]==3 && saved[1]==1);
        CHECK(PathPorts_CopyIds(0,0)==0);
        CHECK(turn_frames==0); reply("DISC_DONE\r\n"); tick();
        CHECK(path_diagnostics.step==4 && path_diagnostics.result==PATH_RUNNING);
        for(unsigned i=0;i<1500 && path_diagnostics.result==PATH_RUNNING;i++) {
            moving=false;
            if(path_diagnostics.step==6 && path_diagnostics.phase==2) yaw+=2;
            tick();
        }
        CHECK(path_diagnostics.step==12 && path_diagnostics.result==PATH_DONE);
        CHECK(!moving && turn_frames==0 && path_diagnostics.rfid_count==6);
        CHECK(PathPorts_CopyIds(saved,9)==6 && !memcmp(saved,expected,sizeof(expected)));
        CHECK(!strcmp(wire,"DISC_START\r\n"));
    }
    puts("ZHY adapter test passed"); return 0;
}
