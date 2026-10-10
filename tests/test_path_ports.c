#include "path_config.h"
#include "path_policy.h"
#include "stair_heading.h"
#include "disc_task_config.h"
#include "path_ports.h"
#include "path_mission.h"
#include "chassis_control.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <assert.h>

UART_HandleTypeDef huart4 = {0, 4}, huart6 = {0, 6}, huart7 = {0, 7};
ChassisConfig chassis_config = {
    .wheel_radius_mm = 35, .half_track_mm = 128.5f, .half_wheelbase_mm = 130.5f};
static ChassisState state;
static uint32_t now;
static uint8_t *rx4, *rx7;
static char wire[80];
static unsigned wire_sequence;
static unsigned holds, turn_positions;
static bool moving, rfid_init_failure;
static bool test_blue;
static bool gray_line = true;
static bool outer_line;
static bool ir_blocked=true;
static float yaw, measured_yaw, map_yaw_test;
static unsigned map_headings, line_calibrations, zero_aligns, blend_moves, arc_moves;
static float blend_end, arc_begin, pending_x, pending_y;

uint32_t HAL_GetTick(void) { return now; }
uint32_t HAL_UART_GetError(UART_HandleTypeDef *u) { (void)u; return 8; }
uint32_t PathSession_Create(void) { return 123; }
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *u, uint8_t *b, uint16_t n) {
    if (n != 1) return HAL_ERROR;
    if (u == &huart4) rx4 = b;
    else if (u == &huart7) { rx7 = b; if (rfid_init_failure) return HAL_ERROR; }
    else return HAL_ERROR;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *u, uint8_t *b, uint16_t n) {
    if (u == &huart4) {
        if (n >= sizeof(wire)) return HAL_ERROR;
        memcpy(wire, b, n); wire[n] = 0; ++wire_sequence;
    } else if (u == &huart6) {
        if (n == 13 && b[1] == 0xfd) ++turn_positions;
    } else return HAL_ERROR;
    return HAL_OK;
}
unsigned HAL_GPIO_ReadPin(void *port, uint16_t pin) {
    if(path_diagnostics.blue && path_diagnostics.step==9 && path_diagnostics.phase==30 && pin==GPIO_PIN_1) return GPIO_PIN_SET;
    if(path_diagnostics.step==13 && path_diagnostics.phase==5 && pin!=GPIO_PIN_10) return GPIO_PIN_SET;
    return (((outer_line) && ((port == GPIOD && pin == GPIO_PIN_3) || (port == GPIOB && pin == GPIO_PIN_13))) ||
            (gray_line && port == GPIOD && (pin == GPIO_PIN_0 || pin == GPIO_PIN_1)) ||
            (port == GPIOD && pin == GPIO_PIN_10 && ir_blocked)) ? GPIO_PIN_RESET : GPIO_PIN_SET;
}
const ChassisState *Chassis_GetState(void) { return &state; }
bool Chassis_Arm(void) { if(state.fault || !state.bias_ready) return false; state.armed=true; return true; }
void Chassis_Stop(void) { state.armed=false; moving=false; PathPorts_Cancel(); }
bool Chassis_MotionBusy(void) { return moving; }
bool Chassis_IsSettled(void) { return !moving; }
float Chassis_MapYaw(void) { return map_yaw_test; }
float Chassis_ContinuousYaw(void) { return yaw; }
float Chassis_MeasuredYaw(void) { return measured_yaw; }
float Chassis_LineYaw(void) { return measured_yaw; }
bool Chassis_SetLineReference(void) { return !moving; }
void Chassis_Hold(void) { moving = false; pending_x=pending_y=0; ++holds; }
bool Chassis_Move(float x, float y, float v, float a, float d) {
#if PATH_BLUE_STAIR_TEST
    if(path_diagnostics.step==8 && fabsf(x)>2) assert(x==10 && y==0);
    if(path_diagnostics.step==9 && (path_diagnostics.phase==22 || path_diagnostics.phase==30) && fabsf(x)>2) assert((x>0 || (path_diagnostics.phase==30 && x==-100)) && y==0);
#endif
    (void)x; (void)y; (void)v; (void)a; (void)d;
    if (test_blue && path_diagnostics.step==0 && fabsf(x)>100 && y!=0) assert(x>0 && y<0);
    if (test_blue && path_diagnostics.step==5 && fabsf(x)>100) assert(x==1395 && y==-755);
    if (!state.armed || moving) return false;
    pending_x=x; pending_y=y; moving = true; return true;
}
bool Chassis_MoveBoundary(float x, float y, float v, float a, float d,
                          float start_speed, float end_speed) {
    /* Accept task profiles and transit profiles independently of stale diagnostic step. */
    const float bases[]={650,850,300,250,845,1105,390,325,1098.5f,1436.5f,507,422.5f,715,935,1428.05f,765};
    bool valid_a=false,valid_d=false;
    for(unsigned i=0;i<sizeof(bases)/sizeof(bases[0]);++i) {
        valid_a |= fabsf(a-bases[i])<.05f || fabsf(a-bases[i]*16/9)<.05f;
        valid_d |= fabsf(d-bases[i])<.05f || fabsf(d-bases[i]*16/9)<.05f;
    }
    if(!valid_a || !valid_d) return false;
    (void)start_speed; blend_end=end_speed;
    return Chassis_Move(x, y, v, a, d);
}
bool Chassis_MoveArc(float radius, float start_angle, float turn, float v, float a, float d,
                     float start_speed, float end_speed) {
#if PATH_BLUE_STAIR_TEST
    if(path_diagnostics.step==8) assert(radius==50 && start_angle==0 && turn==90);
#endif
    (void)radius; (void)start_angle; (void)turn; (void)end_speed;
    ++arc_moves; arc_begin=start_speed;
    return Chassis_Move(1, 0, v, a, d);
}
bool Chassis_MoveRotate(float x, float y, float degrees, float v, float a, float d) {
    (void)degrees; return Chassis_Move(x, y, v, a, d);
}
bool Chassis_MoveRotateBoundary(float x, float y, float degrees, float v, float a, float d,
                                 float start_speed, float end_speed) {
    (void)start_speed; ++blend_moves; blend_end=end_speed;
    return Chassis_MoveRotate(x, y, degrees, v, a, d);
}
bool Chassis_Rotate(float deg) { return Chassis_Move(deg, 0, 1, 1, 1); }
bool Chassis_AlignZero(void) { measured_yaw=0; zero_aligns++; return Chassis_Rotate(0); }
bool Chassis_Body(float x, float y, float w) {
    (void)x; (void)y; (void)w; moving = true; return state.armed;
}
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *u) {
    return (u == &huart4 || u == &huart7) ? HAL_OK : HAL_ERROR;
}

#define CHECK(x) do { if (!(x)) { printf("FAIL %d: %s\n",__LINE__,#x); return 1; } } while(0)
static void tick(void) {
    if (!test_blue && !PATH_BLUE_STAIR_TEST && path_diagnostics.step==9) {
        if (path_diagnostics.phase==30) gray_line=false;
        else if (path_diagnostics.phase==31) gray_line=true;
        else if (path_diagnostics.phase==22 && path_diagnostics.point==4) gray_line=false;
    }
    if (!test_blue && !PATH_BLUE_STAIR_TEST && path_diagnostics.step==10) gray_line=true;
    if(!moving) { state.x_mm+=pending_x*cosf(yaw)-pending_y*sinf(yaw);
        state.y_mm+=pending_x*sinf(yaw)+pending_y*cosf(yaw);pending_x=pending_y=0; }
    now += 5; PathPorts_Tick();
}
static void reply(const char *line) {
    for (const char *p = line; *p; ++p) { *rx4 = (uint8_t)*p; PathPorts_RxComplete(&huart4); }
}
static void raw(uint8_t value) { *rx7 = value; PathPorts_RxComplete(&huart7); }
static void block_frame(uint32_t uid, uint8_t code, bool inconsistent) {
    uint8_t f[28]={4,28,4,32,0,4,0,(uint8_t)(uid>>24),(uint8_t)(uid>>16),(uint8_t)(uid>>8),(uint8_t)uid};
    memset(f+11,code,16);
    if(inconsistent) f[26]^=1;
    f[27]=255; for(unsigned i=0;i<27;i++) f[27]^=f[i];
    for(unsigned i=0;i<28;i++) raw(f[i]);
}
static uint8_t code_for(uint32_t uid) {
    unsigned n=uid>=1 && uid<=9 ? uid-1 : uid==99 ? 5 :
               uid>=101 && uid<=105 ? (uid-101+6)%9 : uid==22 ? 1 : 0;
    return (uint8_t)(((n/3+1)<<4)|(n%3+1));
}
static void make_id(uint32_t value, uint8_t out[28]) {
    uint8_t f[28] = {4,28,4,32,0,4,0,(uint8_t)(value>>24),(uint8_t)(value>>16),
                     (uint8_t)(value>>8),(uint8_t)value};
    memset(f+11,code_for(value),16);
    f[27]=255; for(unsigned i=0;i<27;i++) f[27]^=f[i];
    memcpy(out,f,28);
}
static void id(uint32_t value) { uint8_t f[28]; make_id(value,f); for(unsigned i=0;i<28;i++) raw(f[i]); }
static void uid_only(void) {
    uint8_t f[]={4,12,2,32,0,4,0,0,0,0,1,0};
    f[11]=255; for(unsigned i=0;i<11;i++) f[11]^=f[i];
    for(unsigned i=0;i<12;i++) raw(f[i]);
}
static void finish_store(void) { for(unsigned i=0;i<195;i++) tick(); }

static int start_disc(bool full) {
    PathPorts_Init();
    if(test_blue) CHECK(PathPorts_SelectSide(true));
    CHECK(PathPorts_Busy()); tick(); CHECK(!strcmp(wire, "PING\r\n"));
    reply("PONG\r\n"); tick(); CHECK(path_diagnostics.accepted_ids == 0);
    CHECK(PathPorts_Busy()); tick(); CHECK(!strcmp(wire,"GROUP 0\r\n"));
    reply("GROUP_ACK 0\r\n"); tick(); CHECK(PathPorts_Busy());
    reply("GROUP_DONE 0\r\n"); tick(); tick(); CHECK(!strcmp(wire,test_blue?"COLOR BLUE\r\n":"COLOR RED\r\n")); CHECK(PathPorts_Busy() && !PathPorts_Start()); reply(test_blue?"COLOR_OK BLUE\r\n":"COLOR_OK RED\r\n"); tick(); CHECK(path_diagnostics.accepted_ids == 1);
    state.armed = true;
    CHECK(!PathPorts_SelectSide(!test_blue));
    id(0x01020304); tick(); /* Capture is closed before a disc action gate. */
    CHECK(full ? PathPorts_Start() : PathPorts_Disc()); tick(); reply("PONG\r\n"); tick(); tick();
    if (full) {
        for(unsigned i=0;i<30 && strcmp(wire,"GROUP 100\r\n");i++) { moving=false; tick(); }
        CHECK(!strcmp(wire,"GROUP 100\r\n"));
        reply("GROUP_ACK 100\r\nGROUP_DONE 100\r\n"); tick();
        for(unsigned i=0;i<30 && strcmp(wire,"DISC_START\r\n");i++) { moving=false; tick(); }
    } else {
        for(unsigned i=0;i<10 && strcmp(wire,"GROUP 100\r\n");i++) tick();
        CHECK(!strcmp(wire,"GROUP 100\r\n"));
        reply("GROUP_ACK 100\r\nGROUP_DONE 100\r\n");tick();tick();
    }
    CHECK(!strcmp(wire, "DISC_START\r\n"));
    reply("DISC_ACK\r\n"); tick();
    CHECK(path_diagnostics.rfid_count == 0);
    CHECK(path_diagnostics.disc_waiting_rfid == 0);
    CHECK(path_diagnostics.disc_action_allowed == 1);
    return 0;
}
static int action_done(uint8_t index) {
    char line[32]; snprintf(line, sizeof(line), "DISC_ACTION_DONE %u\r\n", index);
    reply(line); tick();
    CHECK(path_diagnostics.disc_action_done_index == index);
    CHECK(path_diagnostics.disc_waiting_rfid == index);
    CHECK(path_diagnostics.disc_action_allowed == 0);
    return 0;
}
static int scan(uint8_t index, uint32_t uid) {
    id(uid); tick(); finish_store();
    char expected[32]; snprintf(expected, sizeof(expected), "DISC_RFID_OK %u\r\n", index);
    CHECK(!strcmp(wire, expected));
    CHECK(path_diagnostics.disc_rfid_confirmed_index == index);
    CHECK(path_diagnostics.disc_waiting_rfid == 0);
    CHECK(path_diagnostics.disc_action_allowed == (index < DISC_REQUIRED_RFID_COUNT));
    return 0;
}
static int complete_gate(uint8_t index, uint32_t uid) {
    CHECK(action_done(index) == 0); CHECK(scan(index, uid) == 0); return 0;
}

int main(int argc, char **argv) {
    CHECK(argc == 2);
    if (!strcmp(argv[1], "ir_start")) {
        PathPorts_Init(); state.bias_ready=false;
        tick(); CHECK(!state.armed && !strcmp(wire,"PING\r\n"));
        now+=200; tick(); ir_blocked=false; now+=200; tick();
        CHECK(!state.armed); /* Request waits for G0/COLOR and IMU readiness. */
        reply("PONG\r\n"); tick(); tick();
        reply("GROUP_ACK 0\r\nGROUP_DONE 0\r\n"); tick(); tick();
        CHECK(!strcmp(wire,"COLOR RED\r\n"));
        reply("COLOR_OK RED\r\n"); tick(); now+=200; tick();
        CHECK(!state.armed); /* IMU is the final startup dependency. */
        state.bias_ready=true; tick();
        CHECK(state.armed && path_diagnostics.result==PATH_RUNNING && !path_diagnostics.blue);
        Chassis_Stop(); tick(); ir_blocked=true; now+=200; tick();
        ir_blocked=false; now+=200; tick(); now+=200; tick();
        CHECK(!state.armed && path_diagnostics.result!=PATH_RUNNING);
        puts("infrared boot handshake and one-shot start passed"); return 0;
    }
    if (!strcmp(argv[1], "ir_start_blue")) {
        PathPorts_Init();state.bias_ready=true;ir_blocked=true;
        tick();reply("PONG\r\n");tick();tick();
        reply("GROUP_ACK 0\r\nGROUP_DONE 0\r\n");tick();tick();
        reply("COLOR_OK RED\r\n");tick();
        now+=4900;tick();CHECK(!state.armed);
        now+=110;tick();tick();tick();CHECK(!state.armed && !strcmp(wire,"COLOR BLUE\r\n"));
        reply("COLOR_OK BLUE\r\n");tick();
        CHECK(state.armed && path_diagnostics.result==PATH_RUNNING && path_diagnostics.blue);
        CHECK(ir_blocked); /* Blue starts while the infrared remains covered. */
        Chassis_Stop();tick();ir_blocked=false;now+=200;tick();now+=200;tick();
        CHECK(!state.armed && path_diagnostics.result!=PATH_RUNNING);
        puts("blue long-hold handshake and one-shot start passed");return 0;
    }
    if (!strcmp(argv[1], "no_vision")) {
        PathPorts_Init(); outer_line=true; tick(); CHECK(path_diagnostics.gray==6);
        outer_line=false; tick(); CHECK(path_diagnostics.gray==6);
        CHECK(!PathPorts_Busy() && wire[0]==0 && rx4==NULL && rx7==NULL);
        CHECK(!PathPorts_Start()); /* ARM is still mandatory. */
        state.armed=true; CHECK(PathPorts_Start());
        tick();
        CHECK(blend_moves==(PATH_BLUE_DISC_TEST?1:0) && moving && blend_end>0);
        unsigned before=holds;
        moving=false; tick(); /* Translation done while wheels carry nonzero speed. */
        CHECK(arc_moves==1 && moving && holds==before && arc_begin==blend_end);
        moving=false; tick();
        CHECK(path_diagnostics.step==1 && moving && holds==before);
        unsigned previous_phase=99;
        for(unsigned i=0;i<3000 && PathPorts_Busy();i++) {
            moving=false;
            if(path_diagnostics.step==6 && path_diagnostics.phase==2) yaw-=0.03f;
            if(path_diagnostics.step==9 && path_diagnostics.phase==4 && previous_phase!=4)
                measured_yaw=5;
            previous_phase=path_diagnostics.phase;
            tick();
            CHECK(wire[0]==0 && !path_diagnostics.fault);
        }
#if PATH_BLUE_STAIR_TEST
        CHECK(path_diagnostics.result==PATH_DONE && path_diagnostics.step==9);
        CHECK(!moving && blend_moves==2 && wire[0]==0);
        for(unsigned i=0;i<100;i++) tick();
        CHECK(path_diagnostics.result==PATH_DONE && path_diagnostics.step==9 && !moving);
#elif PATH_BLUE_PILLAR_TEST
        CHECK(path_diagnostics.result==PATH_DONE && path_diagnostics.step==6);
        CHECK(!moving && blend_moves==2 && wire[0]==0);
        for(unsigned i=0;i<100;i++) tick();
        CHECK(path_diagnostics.result==PATH_DONE && path_diagnostics.step==6 && !moving && blend_moves==2);
#elif PATH_BLUE_DISC_TEST
        CHECK(path_diagnostics.result==PATH_DONE && path_diagnostics.step==3);
        CHECK(!moving && blend_moves==1 && wire[0]==0);
        for(unsigned i=0;i<100;i++) tick();
        CHECK(path_diagnostics.result==PATH_DONE && path_diagnostics.step==3 && !moving && blend_moves==1);
#else
        CHECK(path_diagnostics.result==PATH_DONE && path_diagnostics.step==13);
        CHECK(line_calibrations==0 && map_headings==9 && zero_aligns==0 && path_diagnostics.rfid_count==0);
#endif
        CHECK(!PathPorts_Disc() && !PathPorts_Ping());
        CHECK(PathPorts_Start()); tick(); PathPorts_Cancel(); tick();
        CHECK(path_diagnostics.result==PATH_CANCELED && !PathPorts_Busy());
        CHECK(PathPorts_Start()); tick(); state.fault=2; tick();
        CHECK(path_diagnostics.result==PATH_ERROR && !moving && wire[0]==0);
        puts("no-vision continuous stair route, restart, STOP and fault passed");
        return 0;
    }
    if (!strncmp(argv[1],"side_",5)) {
        PathPorts_Init(); CHECK(PathPorts_SelectSide(true));
        tick(); reply("PONG\r\n"); tick(); tick();
        reply("GROUP_ACK 0\r\nGROUP_DONE 0\r\n"); tick(); tick();
        CHECK(!strcmp(wire,"COLOR BLUE\r\n") && PathPorts_Busy());
        CHECK(!PathPorts_Start());
        if(!strcmp(argv[1],"side_timeout")) {
            now+=2100;tick();CHECK(PathPorts_Busy() && !path_diagnostics.accepted_ids);
            CHECK(!PathPorts_SelectSide(false));
        } else {
            CHECK(PathPorts_SelectSide(false)); /* Selection changed while the earlier ACK is in flight. */
            reply("COLOR_OK BLUE\r\n"); tick();tick();
            CHECK(!strcmp(wire,"COLOR RED\r\n") && PathPorts_Busy());
            reply("COLOR_OK RED\r\n");tick(); CHECK(!PathPorts_Busy());
            CHECK(PathPorts_SelectSide(true));tick();tick();reply("COLOR_OK BLUE\r\n");tick();
            CHECK(!PathPorts_Busy());
        }
        CHECK(PathPorts_Reset());tick();reply("PONG\r\n");tick();tick();
        reply("GROUP_ACK 0\r\nGROUP_DONE 0\r\n");tick();tick();
        CHECK(!strcmp(wire,"COLOR RED\r\n"));reply("COLOR_OK RED\r\n");tick();
        CHECK(!PathPorts_Busy() && path_diagnostics.accepted_ids);
        state.armed=true;CHECK(!PathPorts_SelectSide(true));
        puts("side synchronization passed");return 0;
    }
    if (!strncmp(argv[1],"boot_",5)) {
        PathPorts_Init(); tick(); CHECK(!strcmp(wire,"PING\r\n"));
        CHECK(PathPorts_Busy() && !PathPorts_Start());
        if (!strcmp(argv[1],"boot_retry")) {
            now+=2000; tick(); CHECK(PathPorts_Busy());
            now+=1000; tick(); CHECK(!strcmp(wire,"PING\r\n"));
        }
        reply("PONG\r\n"); tick(); tick(); CHECK(!strcmp(wire,"GROUP 0\r\n"));
        reply("GROUP_ACK 0\r\n"); tick(); CHECK(PathPorts_Busy());
        if (!strcmp(argv[1],"boot_retry")) {
            reply("GROUP_DONE 0\r\n"); tick(); tick(); reply("COLOR_OK RED\r\n"); tick(); CHECK(!PathPorts_Busy());
            CHECK(path_diagnostics.accepted_ids==1);
            CHECK(PathPorts_Ping()); tick(); reply("PONG\r\n"); tick(); tick();
            CHECK(!strcmp(wire,"PING\r\n")); /* No second G0. */
        } else if (!strcmp(argv[1],"boot_error")) {
            reply("GROUP_ERROR 0\r\n"); tick(); tick();
            CHECK(PathPorts_Busy() && !PathPorts_Start());
            CHECK(path_diagnostics.accepted_ids==0 && !strcmp(wire,"DISC_CANCEL\r\n"));
        } else if (!strcmp(argv[1],"boot_stop")) {
            PathPorts_Cancel(); tick(); CHECK(!strcmp(wire,"DISC_CANCEL\r\n"));
            reply("GROUP_DONE 0\r\n"); tick(); CHECK(PathPorts_Busy());
            CHECK(path_diagnostics.accepted_ids==0);
        } else CHECK(0);
        puts("boot test passed"); return 0;
    }
    test_blue = !strcmp(argv[1], "full_path_blue") || !strcmp(argv[1],"blue_bypass") || !strcmp(argv[1],"no_id_continue_blue");
    rfid_init_failure = !strcmp(argv[1], "rfid_init");
    CHECK(start_disc((test_blue && strcmp(argv[1],"no_id_continue_blue")) || !strncmp(argv[1], "full_path",9)) == 0);
    if(!strcmp(argv[1],"blue_bypass")) {
        for(unsigned i=1;i<=5;i++) {
            char response[60];snprintf(response,sizeof(response),"DISC_ACTION_DONE %u\r\n",i);
            reply(response);tick();tick();
            snprintf(response,sizeof(response),"DISC_RFID_OK %u\r\n",i);
            CHECK(!strcmp(wire,response));
            id(i);tick();CHECK(path_diagnostics.rfid_count==0 && turn_positions==0);
            CHECK(path_diagnostics.step==3);
        }
        reply("DISC_DONE\r\n");tick();
        unsigned seen=0,columns=0,actions=0;
        for(unsigned i=0;i<60000 && path_diagnostics.result==PATH_RUNNING;i++) {
            moving=false;
            if(path_diagnostics.step==6 && path_diagnostics.phase==2) yaw-=.03f;
            if(wire_sequence!=seen) {
                seen=wire_sequence;unsigned group,level,token,excluded,index;char response[80];
                if(sscanf(wire,"GROUP %u",&group)==1) {
                    CHECK(group<109);snprintf(response,sizeof(response),"GROUP_ACK %u\r\nGROUP_DONE %u\r\n",group,group);reply(response);
                } else if(!strcmp(wire,"PILLAR_START\r\n") || sscanf(wire,"STAIR_SCAN %u",&level)==1) {
                    reply("PILLAR_ACK\r\nPILLAR_READY\r\nPILLAR_BALL 1\r\n");
                } else if(sscanf(wire,"PILLAR_STOPPED %u",&index)==1) {
                    snprintf(response,sizeof(response),"PILLAR_ACTION_DONE %u\r\n",index);reply(response);actions++;
                } else if(sscanf(wire,"PILLAR_RFID_OK %u",&index)==1) {
                    snprintf(response,sizeof(response),"PILLAR_RESUME %u\r\n",index);reply(response);
                } else if(!strcmp(wire,"PILLAR_END\r\n")) reply("PILLAR_DONE\r\n");
                else if(sscanf(wire,"WAREHOUSE_CHECK %u %u",&token,&excluded)==2) {
                    CHECK(columns<3);columns++;
                    snprintf(response,sizeof(response),"WAREHOUSE_READY %u\r\nWAREHOUSE_DIGIT %u %u\r\n",token,token,columns);reply(response);
                }
            }
            tick();
        }
        CHECK(path_diagnostics.result==PATH_DONE && columns==2 && actions>=2);
        CHECK(turn_positions==0 && path_diagnostics.rfid_count==0 && path_diagnostics.inventory_occupied==0);
        puts("blue bypass: five disc actions, pillar/stair actions, two digits plus inferred third, home passed");return 0;
    }

    if (!strcmp(argv[1], "uid_only")) {
        CHECK(action_done(1)==0); uid_only(); tick(); tick();
        CHECK(path_diagnostics.rfid_count==1 && path_diagnostics.disc_waiting_rfid==0);
        CHECK(!strcmp(wire,"DISC_RFID_OK 1\r\n"));
        finish_store(); CHECK(turn_positions==1);
        BallInventory stock; PathPorts_CopyInventory(&stock);
        CHECK(stock.occupied==1 && stock.uid[0]==1 && stock.code[0]==0 && stock.current==1);
    } else if (!strcmp(argv[1], "store_wait")) {
        CHECK(action_done(1)==0); block_frame(1,0x23,false); tick();
        CHECK(path_diagnostics.rfid_count==1 && path_diagnostics.disc_waiting_rfid==0);
        for(unsigned i=0;i<80;i++) tick();
        CHECK(!strcmp(wire,"DISC_RFID_OK 1\r\n") && path_diagnostics.inventory_slot==0);
        for(unsigned i=0;i<110;i++) tick();
        CHECK(!strcmp(wire,"DISC_RFID_OK 1\r\n") && turn_positions==1);
    } else if (!strcmp(argv[1], "invalid_block")) {
        CHECK(action_done(1)==0); block_frame(1,0x23,true); tick(); tick();
        CHECK(path_diagnostics.result==PATH_RUNNING && path_diagnostics.rfid_count==1);
        CHECK(!strcmp(wire,"DISC_RFID_OK 1\r\n") && path_diagnostics.inventory_fault);
    } else if (!strcmp(argv[1], "duplicate_cell")) {
        CHECK(action_done(1)==0); block_frame(1,0x23,false);
        for(unsigned i=0;i<200;i++) tick();
        CHECK(action_done(2)==0); block_frame(2,0x23,false); tick(); tick();
        CHECK(path_diagnostics.result==PATH_RUNNING && path_diagnostics.rfid_count==2);
        CHECK(!strcmp(wire,"DISC_RFID_OK 2\r\n") && path_diagnostics.inventory_fault);
        BallInventory stock; PathPorts_CopyInventory(&stock);
        CHECK(stock.occupied==3 && stock.code[0]==0x23 && stock.code[1]==0);
    } else if (!strcmp(argv[1], "turn_error")) {
        CHECK(action_done(1)==0); huart6.gState=1; block_frame(1,0x23,false); tick();
        for(unsigned i=0;i<410;i++) tick();
        CHECK(path_diagnostics.result==PATH_RUNNING && !strcmp(wire,"DISC_RFID_OK 1\r\n"));
        CHECK(path_diagnostics.inventory_uncertain);
    } else if (!strcmp(argv[1], "store_cancel")) {
        CHECK(action_done(1)==0); id(1); tick();
        PathPorts_Cancel(); tick(); finish_store();
        CHECK(path_diagnostics.result==PATH_CANCELED);
        CHECK(path_diagnostics.inventory_uncertain && path_diagnostics.inventory_occupied==1);
        CHECK(strcmp(wire,"DISC_RFID_OK 1\r\n"));
        state.armed=false;
        CHECK(PathPorts_Reset());
        BallInventory stock; PathPorts_CopyInventory(&stock);
        CHECK(stock.uncertain && stock.occupied==1 && stock.code[0]==0x11);
        CHECK(!PathPorts_Start() && !PathPorts_Disc());
    } else if (!strcmp(argv[1], "queued_fault")) {
        CHECK(action_done(1)==0); id(1); tick();
        for(unsigned i=0;i<200 && !path_diagnostics.disc_rfid_confirmed_index;i++) tick();
        CHECK(path_diagnostics.disc_rfid_confirmed_index==1);
        CHECK(strcmp(wire,"DISC_RFID_OK 1\r\n"));
        PathPorts_Error(&huart4); tick();
        CHECK(path_diagnostics.rdk_uart_hal_error==8 && path_diagnostics.rdk_uart_error_count==1);
        CHECK(path_diagnostics.result==PATH_ERROR && path_diagnostics.inventory_uncertain);
        CHECK(strcmp(wire,"DISC_RFID_OK 1\r\n"));
    } else if (!strncmp(argv[1], "full_path",9)) {
        const bool missing_ids=!strcmp(argv[1],"full_path_missing_ids");
        const unsigned extra=!strcmp(argv[1],"full_path9") || test_blue;
        for (uint8_t i=1;i<=5;i++) CHECK(complete_gate(i,i)==0);
        CHECK(moving && strcmp(wire,"GROUP 1\r\n"));
        reply("DISC_DONE\r\n"); tick(); tick(); tick();
        CHECK(!strcmp(wire,"GROUP 1\r\n")); CHECK(moving);
        reply("GROUP_ACK 1\r\nGROUP_DONE 1\r\n"); tick(); tick(); CHECK(moving);
        moving=false;
        for(unsigned i=0;i<30 && strcmp(wire,"PILLAR_START\r\n");i++) { moving=false; tick(); }
        CHECK(!strcmp(wire,"PILLAR_START\r\n")); CHECK(!moving);
        reply("PILLAR_ACK\r\n"); tick(); CHECK(!moving);
        for(unsigned retry=0;retry<20;retry++) {
            now+=20000;
            reply("PILLAR_CAMERA_WAIT\r\n"); tick();
            CHECK(!moving && path_diagnostics.result==PATH_RUNNING && path_diagnostics.phase==4);
        }
        reply("PILLAR_READY\r\n"); tick(); CHECK(moving);
        yaw=-2.0f; reply("PILLAR_BALL 1\r\n"); tick(); CHECK(!moving);
        tick(); tick(); CHECK(!strcmp(wire,"PILLAR_STOPPED 1\r\n"));
        id(99); tick(); CHECK(path_diagnostics.rfid_count==5);
        reply("PILLAR_ACTION_DONE 1\r\n"); tick();
        CHECK(path_diagnostics.disc_waiting_rfid==1);
        id(1); tick(); CHECK(path_diagnostics.rfid_count==5);
        if(missing_ids) { now+=PATH_RFID_WAIT_MS; tick(); tick(); }
        else { id(99); tick(); }
        finish_store(); CHECK(!strcmp(wire,"PILLAR_RFID_OK 1\r\n"));
        CHECK(!moving && path_diagnostics.rfid_count==(missing_ids?5:6));
        reply("PILLAR_RESUME 1\r\n"); tick(); CHECK(moving);
        if(extra) {
            reply("PILLAR_BALL 2\r\n"); tick(); tick(); tick();
            CHECK(!strcmp(wire,"PILLAR_STOPPED 2\r\n") && !moving);
            reply("PILLAR_ACTION_DONE 2\r\n"); tick();
            id(103); tick(); finish_store();
            CHECK(!strcmp(wire,"PILLAR_RFID_OK 2\r\n"));
            reply("PILLAR_RESUME 2\r\n"); tick(); CHECK(moving);
        }
        gray_line=false; outer_line=false; yaw=test_blue?-9.26f:-6.26f; tick(); CHECK(moving && path_diagnostics.step==8); tick(); tick();
        CHECK(!strcmp(wire,"PILLAR_END\r\n"));
        reply("PILLAR_DONE\r\n"); tick(); CHECK(path_diagnostics.step==8);
        gray_line=false;
        if (!extra) {
            for(unsigned i=0;i<30 && strcmp(wire,"GROUP 2\r\n");i++) {moving=false; tick();}
            CHECK(!strcmp(wire,"GROUP 2\r\n"));
            gray_line=false;
            for(unsigned i=0;i<4 && !moving;i++) tick();
            CHECK(moving); /* G2 completion does not block approach. */
            reply("GROUP_ACK 2\r\nGROUP_DONE 2\r\n"); tick();
        }
        for(unsigned i=0;i<30 && strcmp(wire,"GROUP 105\r\n");i++) {moving=false; tick();}
        CHECK(!strcmp(wire,"GROUP 105\r\n"));
        reply("GROUP_ACK 105\r\nGROUP_DONE 105\r\n"); tick();
        for(unsigned i=0;i<20 && path_diagnostics.step!=9;i++) {moving=false; tick();}
        CHECK(path_diagnostics.step==9);
        gray_line=false; tick(); CHECK(moving);
        now+=10000; tick(); CHECK(moving && !path_diagnostics.fault);
        CHECK(path_diagnostics.result==PATH_RUNNING); /* Body watchdog also exceeds 5s. */
        gray_line=true;
        measured_yaw=5;
        for(unsigned i=0;i<60 && strcmp(wire,"STAIR_SCAN 1\r\n");i++) {moving=false; tick();}
        CHECK(!strcmp(wire,"STAIR_SCAN 1\r\n") && !moving);
        reply("PILLAR_ACK\r\nPILLAR_READY\r\n"); tick(); tick();CHECK(moving);
        reply("PILLAR_BALL 1\r\n");tick();tick();tick();
        CHECK(!moving && !strcmp(wire,"PILLAR_STOPPED 1\r\n"));
        reply("PILLAR_ACTION_DONE 1\r\n");tick();
        if(missing_ids) { now+=PATH_RFID_WAIT_MS; tick(); tick(); }
        else { id(99);tick();CHECK(path_diagnostics.rfid_count==6+extra); id(101);tick(); }
        finish_store();CHECK(!strcmp(wire,"PILLAR_RFID_OK 1\r\n"));
        reply("PILLAR_RESUME 1\r\n");tick();
        for(unsigned i=0;i<60 && strcmp(wire,"PILLAR_END\r\n");i++) {moving=false;tick();}
        CHECK(!strcmp(wire,"PILLAR_END\r\n"));reply("PILLAR_DONE\r\n");tick();
        for(unsigned i=0;i<30 && strcmp(wire,"GROUP 4\r\n");i++) tick();
        CHECK(!strcmp(wire,"GROUP 4\r\n") && !moving);
        reply("GROUP_ACK 4\r\n");for(unsigned i=0;i<10;i++)tick();CHECK(!moving);
        reply("GROUP_DONE 4\r\n");tick();
        for(unsigned i=0;i<30 && strcmp(wire,"STAIR_SCAN 2\r\n");i++)tick();
        CHECK(!strcmp(wire,"STAIR_SCAN 2\r\n"));
        reply("PILLAR_ACK\r\nPILLAR_READY\r\n");tick();tick();CHECK(moving);
        reply("PILLAR_BALL 1\r\n");tick();tick();tick();
        CHECK(!strcmp(wire,"PILLAR_STOPPED 1\r\n") && !moving);
        reply("PILLAR_ACTION_DONE 1\r\n");tick();
        if(missing_ids) { now+=PATH_RFID_WAIT_MS; tick(); tick(); }
        else { id(101);tick();CHECK(path_diagnostics.rfid_count==7+extra); id(102);tick(); }
        finish_store();CHECK(!strcmp(wire,"PILLAR_RFID_OK 1\r\n"));
        reply("PILLAR_RESUME 1\r\n");
        for(unsigned i=0;i<10 && strcmp(wire,"PILLAR_END\r\n");i++) tick();
        CHECK(!strcmp(wire,"PILLAR_END\r\n"));reply("PILLAR_DONE\r\n");tick();
        bool group3_replied=false;
        unsigned warehouse_done=0, group_sequence=0;
        for(unsigned i=0;i<30000 && path_diagnostics.result==PATH_RUNNING;i++) {
            moving=false;
            if (!group3_replied && !strcmp(wire,"GROUP 3\r\n")) {
                reply("GROUP_ACK 3\r\nGROUP_DONE 3\r\n"); group3_replied=true;
            }
            if(path_diagnostics.step==13 && !strncmp(wire,"WAREHOUSE_CHECK ",16)) {
                unsigned token=0;sscanf(wire,"WAREHOUSE_CHECK %u",&token);
                char response[60];snprintf(response,sizeof(response),"WAREHOUSE_DIGIT %u 0\r\n",token);
                reply(response);
            }
            if(path_diagnostics.step==13 && path_diagnostics.phase==3 &&
               !strncmp(wire,"GROUP 1",7) && wire_sequence!=group_sequence) {
                unsigned group=0;
                if(sscanf(wire,"GROUP %u",&group)==1 && group>=109 && group<=111) {
                    BallInventory stock; PathPorts_CopyInventory(&stock);
                    CHECK(!stock.uncertain && (stock.occupied&(1U<<stock.current)));
                    CHECK(stock.code[stock.current]==path_diagnostics.warehouse_code);
                    CHECK(group==112U-(stock.code[stock.current]>>4));
                    char response[60]; snprintf(response,sizeof(response),"GROUP_ACK %u\r\nGROUP_DONE %u\r\n",group,group);
                    reply(response); warehouse_done++; group_sequence=wire_sequence;
                }
            }
            tick();
        }
        CHECK(warehouse_done==8+extra && path_diagnostics.warehouse_placed==8+extra);
        CHECK(path_diagnostics.inventory_occupied==0);
        CHECK(path_diagnostics.result==PATH_DONE && path_diagnostics.step==13 && !moving);
        CHECK(path_diagnostics.rfid_count==(missing_ids?5:8+extra));
        CHECK(!strncmp(wire,"GROUP 1",7)); /* Each reordered group/code pair was verified above. */
        CHECK(PathPorts_Start()); /* All assigned destinations were unloaded. */
    } else if (!strcmp(argv[1], "frames")) {
        CHECK(action_done(1) == 0);
        /* Actual reader capture: UID 90 BB E1 76, block 1 filled with 12. */
        const uint8_t f[28]={4,0x1c,4,0x20,0,4,0,0x90,0xbb,0xe1,0x76,
                            0x12,0x12,0x12,0x12,0x12,0x12,0x12,0x12,
                            0x12,0x12,0x12,0x12,0x12,0x12,0x12,0x12,0x7b};
        for (unsigned i=0;i<6;i++) raw(f[i]);
        tick(); CHECK(path_diagnostics.rfid_count == 0);
        for (unsigned i=6;i<28;i++) raw(f[i]);
        tick(); finish_store();
        CHECK(path_diagnostics.rfid_count == 1 && !strcmp(wire, "DISC_RFID_OK 1\r\n"));
        BallInventory stock; PathPorts_CopyInventory(&stock);
        CHECK(stock.uid[0]==0x90bbe176 && stock.code[0]==0x12 && stock.occupied==1 && stock.current==1);
    } else if (!strcmp(argv[1], "rfid")) {
        CHECK(complete_gate(1, 3) == 0);
        CHECK(action_done(2) == 0); id(3); tick();
        CHECK(path_diagnostics.rfid_count == 1 && path_diagnostics.disc_waiting_rfid == 2);
        CHECK(scan(2, 1) == 0);
        id(9); tick(); CHECK(path_diagnostics.rfid_count == 2); /* closed: no preauthorization */
        CHECK(complete_gate(3, 9) == 0);
        CHECK(complete_gate(4, 2) == 0);
        CHECK(complete_gate(5, 7) == 0);
        uint32_t saved[8] = {0}; const uint32_t expected[] = {3,1,9,2,7};
        CHECK(PathPorts_CopyIds(saved, 8) == 5);
        CHECK(!memcmp(saved, expected, sizeof(expected)));
        reply("DISC_DONE\r\n"); tick(); CHECK(path_diagnostics.result == PATH_DONE);
    } else if (!strcmp(argv[1], "overflow")) {
        CHECK(action_done(1) == 0); id(11); id(22); tick(); finish_store();
        CHECK(path_diagnostics.rfid_count == 1);
        CHECK(action_done(2) == 0); tick();
        CHECK(path_diagnostics.rfid_count == 1 && path_diagnostics.disc_waiting_rfid == 2);
        CHECK(scan(2, 22) == 0);
    } else if (!strcmp(argv[1], "stop") || !strcmp(argv[1], "pending_stop")) {
        CHECK(action_done(1) == 0); PathPorts_Cancel(); tick();
        CHECK(path_diagnostics.result == PATH_CANCELED);
        CHECK(path_diagnostics.disc_waiting_rfid == 0 && path_diagnostics.disc_action_allowed == 0);
        CHECK(!strcmp(wire, "DISC_CANCEL\r\n"));
        id(5); tick(); CHECK(path_diagnostics.rfid_count == 0);
    } else if (!strcmp(argv[1], "timeout")) {
        CHECK(action_done(1) == 0); now += DISC_TASK_TIMEOUT_MS; tick();
        CHECK(path_diagnostics.result == PATH_TIMEOUT);
        CHECK(path_diagnostics.disc_waiting_rfid == 0 && path_diagnostics.disc_action_allowed == 0);
    } else if (!strcmp(argv[1], "late")) {
        for (uint8_t i=1;i<=4;i++) CHECK(complete_gate(i, i) == 0);
        now += DISC_TASK_TIMEOUT_MS; tick(); reply("DISC_DONE\r\n"); tick();
        CHECK(path_diagnostics.result == PATH_TIMEOUT);
    } else if (!strcmp(argv[1], "error")) {
        CHECK(action_done(1) == 0); reply("DISC_ERROR\r\n"); tick();
        CHECK(path_diagnostics.result == PATH_ERROR);
        CHECK(path_diagnostics.disc_waiting_rfid == 0 && path_diagnostics.disc_action_allowed == 0);
    } else if (!strcmp(argv[1], "link_fault")) {
        CHECK(action_done(1) == 0); id(44); tick(); /* Storage turn has begun; permission is withheld. */
        CHECK(path_diagnostics.rfid_count == 1);
        huart4.gState=1; /* Cancellation must survive temporarily busy TX. */
        PathPorts_Error(&huart4); tick();
        CHECK(strcmp(wire, "DISC_RFID_OK 1\r\n") != 0);
        CHECK(path_diagnostics.result == PATH_ERROR);
        CHECK(path_diagnostics.disc_action_allowed == 0);
        tick(); CHECK(strcmp(wire,"DISC_CANCEL\r\n")!=0);
        huart4.gState=HAL_UART_STATE_READY;
        tick(); CHECK(!strcmp(wire,"DISC_CANCEL\r\n"));
    } else if (!strcmp(argv[1], "stationary")) {
        for (uint8_t i=1;i<=5;i++) CHECK(complete_gate(i, 100+i) == 0);
        reply("DISC_DONE\r\n"); tick(); CHECK(path_diagnostics.result == PATH_DONE);
        for (unsigned i=0;i<1000;i++) tick();
        CHECK(turn_positions == 5);
    } else if ((!strcmp(argv[1], "no_id_continue") || !strcmp(argv[1], "no_id_continue_blue"))) {
        for(uint8_t i=1;i<=5;i++) {
            CHECK(action_done(i)==0);
            uint32_t start=now;
            now=start+PATH_RFID_WAIT_MS-10; tick();
            CHECK(path_diagnostics.disc_waiting_rfid==i);
            now=start+PATH_RFID_WAIT_MS; tick(); tick(); finish_store();
            CHECK(path_diagnostics.disc_waiting_rfid==0);
            CHECK(path_diagnostics.disc_rfid_confirmed_index==i);
            CHECK(path_diagnostics.rfid_count==0 && turn_positions==i);
        }
        BallInventory stock; PathPorts_CopyInventory(&stock);
        CHECK(stock.collected==5 && stock.occupied==31 && !stock.uncertain);
        for(unsigned i=0;i<5;i++) CHECK(stock.uid[i]==0 && stock.code[i]==0);
        id(99);tick();CHECK(path_diagnostics.rfid_count==0); /* Late read ignored. */
        reply("DISC_DONE\r\n");tick();CHECK(path_diagnostics.result==PATH_DONE);
    } else if (!strcmp(argv[1], "missing")) {
        for (uint8_t i=1;i<=4;i++) CHECK(complete_gate(i, i) == 0);
        reply("DISC_DONE\r\n"); tick(); CHECK(path_diagnostics.result == PATH_ERROR);
    } else if (!strcmp(argv[1], "rfid_fault") || !strcmp(argv[1], "rfid_init") || !strcmp(argv[1], "zero")) {
        if (!strcmp(argv[1], "rfid_fault")) PathPorts_Error(&huart7);
        reply("DISC_DONE\r\n"); tick(); CHECK(path_diagnostics.result == PATH_ERROR);
        CHECK(path_diagnostics.rfid_count == 0 && path_diagnostics.disc_waiting_rfid == 0);
        if (!strcmp(argv[1], "rfid_fault")) CHECK(path_diagnostics.rfid_fault & 16);
        if (!strcmp(argv[1], "rfid_init")) CHECK(path_diagnostics.rfid_fault & 1);
    } else {
        CHECK(0);
    }
    puts("ZHY adapter test passed"); return 0;
}

void Chassis_BeginPath(void) { }
bool Chassis_ExitOrbitArc(float radius,float angle,float v,float a,float d) { (void)angle;map_headings++;moving=false;return Chassis_MoveArc(radius,180,-90,v,a,d,v,v); }
bool Chassis_ExitOrbit(float x,float y,float angle,float v,float end,float a,float d) { (void)angle;(void)v;(void)end;(void)a;(void)d;pending_x=x;pending_y=y;map_headings++;moving=true;return true; }
bool Chassis_ReturnHome(unsigned leg) { (void)leg; moving=true; return true; }
bool Chassis_FinishForward(float x,float v,float a,float d) { (void)x;(void)v;(void)a;(void)d;moving=true;return true; }
void Chassis_HoldImmediate(void) { Chassis_Hold(); }
void Chassis_HoldCapture(void) { Chassis_Hold(); }

bool Chassis_LineSearch(float y,float w) { return Chassis_Body(0,y,w); }
bool Chassis_CalibrateLine(void) { if(moving)return false; line_calibrations++; measured_yaw=0; return true; }

bool Chassis_AlignMapAxis(void) { map_yaw_test=STAIR_TARGET_DEG(path_diagnostics.blue); moving=true; return true; }
bool Chassis_MapSearch(float mm_s) { (void)mm_s; moving=true; return true; }
bool Chassis_MapLateral(float mm) { (void)mm; moving=true; return true; }

bool Chassis_SetMapHeading(float degrees) { if(moving || (degrees!=0 && degrees!=180))return false; map_headings++; map_yaw_test=degrees; return true; }

bool Chassis_AlignHome(float target_deg) { map_yaw_test=target_deg; moving=true; return true; }

void Chassis_SetRoutePolicy(ChassisRoutePolicy policy) { (void)policy; }

bool Chassis_AnchorLine(float degrees) { (void)degrees; return !moving; }
