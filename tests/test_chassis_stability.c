#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../App/chassis_control.c"
uint32_t fake_cycle;
UART_HandleTypeDef huart3={0,3}, huart4={0,4}, huart6={0,6}, huart7={0,7};
static JY60_State_t fake_imu;
volatile PathDiagnostics path_diagnostics;
static bool path_busy;
uint32_t HAL_GetTick(void) { return fake_cycle/1000; }
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *u,uint8_t *p,uint16_t n) {
    (void)p;(void)n; HAL_UART_TxCpltCallback(u); return HAL_OK;
}
bool JY60_Init(void) { return true; }
void JY60_Process(void) { }
const JY60_State_t *JY60_GetState(void) { return &fake_imu; }
bool HostUart_Init(void) { return true; }
int HostUart_Read(uint8_t *p,size_t n) { (void)p;(void)n;return 0; }
void HostUart_Flush(void) { }
void HostUart_Error(UART_HandleTypeDef *u) { (void)u; }
void HostUart_RxComplete(UART_HandleTypeDef *u) { (void)u; }
void PathPorts_Init(void) { }
void PathPorts_Tick(void) { }
void PathPorts_Cancel(void) { path_busy=false; }
bool PathPorts_Busy(void) { return path_busy; }
bool PathPorts_Start(void) { return false; }
bool PathPorts_Disc(void) { return false; }
bool PathPorts_Reset(void) { return false; }
bool PathPorts_Ping(void) { return false; }
bool PathPorts_SelectSide(bool blue) { (void)blue; return false; }
void PathPorts_Error(UART_HandleTypeDef *u) { (void)u; }
void PathPorts_RxComplete(UART_HandleTypeDef *u) { (void)u; }
static void tick(void) {
    fake_cycle+=5000;
    fake_imu.angle_frame_count++;fake_imu.gyro_frame_count++;
    Chassis_Update();
}
static void wait_stop(void) {
    for(int n=0;n<500 && !Chassis_IsSettled();n++) tick();
    assert(Chassis_IsSettled());
}
static void setup(void) {
    fake_imu.trust=JY60_TRUST_GOOD;fake_imu.has_angle=true;
    assert(Chassis_Init());for(int i=0;i<500;i++)tick();
    assert(Chassis_Arm());path_busy=true;Chassis_BeginPath();
}
int main(int argc,char **argv) {
    assert(argc==2);setup();
    if(!strcmp(argv[1],"heading")) {
        assert(Chassis_Rotate(180));
        fake_imu.yaw_deg=178;tick();Chassis_Hold();wait_stop();
        assert(fabsf(Angle_Wrap(heading-180*RAD))<1e-5f);
        assert(Chassis_SetLineReference());
        assert(fabsf(Chassis_LineYaw()+2)<0.01f);
        assert(Chassis_Move(100,0,100,550,550));tick();
        assert(fabsf(state.yaw_error-2*RAD)<0.0001f);
    } else if(!strcmp(argv[1],"blend")) {
        assert(Chassis_MoveRotate(100,0,180,100,550,550));
        fake_imu.yaw_deg=179.7f;planner.active=false;
        for(int n=0;n<12;n++)tick();
        wait_stop();assert(fabsf(Angle_Wrap(heading-180*RAD))<1e-5f);
    } else if(!strcmp(argv[1],"entry_arc")) {
        assert(Chassis_MoveRotateBoundary(-1640,0,180,550,550,550,0,165));
        fake_imu.yaw_deg=180; segment_progress=1640; tick();
        assert(!Chassis_MotionBusy() && path_heading_chain && !normal_stopping);
        assert(fabsf(dx-1)<0.001f && fabsf(dy)<0.001f);
        assert(Chassis_MoveArc(100,0,90,165,550,550,165,110));tick();
        assert(hypotf(body_output[0],body_output[1])>100);
        segment_progress=planner.distance; tick();
        assert(!Chassis_MotionBusy() && !normal_stopping);
        assert(fabsf(dx)<0.01f && dy>0.99f);
        assert(Chassis_Body(0,110,0));tick();
        assert(body_output[1]>100 && !normal_stopping);
        Chassis_Hold();wait_stop();assert(Chassis_IsSettled());
    } else if(!strcmp(argv[1],"orbit")) {
        assert(Chassis_Body(-200,0,-0.7f));
        for(int n=0;n<150;n++)tick();
        fake_imu.yaw_deg=35;tick();Chassis_Hold();wait_stop();
        assert(fabsf(heading)<1e-6f);
        assert(Chassis_Body(-200,0,-0.7f));tick();
        assert(fabsf(heading)<1e-6f);
    } else if(!strcmp(argv[1],"capture")) {
        assert(Chassis_Body(-295,0,-0.86f));
        for(int n=0;n<200;n++) tick();
        float before=body_output[0];
        Chassis_HoldCapture(); tick();
        assert(normal_stopping && fabsf(body_output[0]-before-6.0f)<0.01f);
        assert(fabsf(body_output[2]/body_output[0]-0.86f/295)<1e-5f);
        unsigned n=1; while(!Chassis_IsSettled() && n<100) {tick();++n;}
        assert(n>=65 && n<75 && Chassis_IsSettled());
        assert(Chassis_Body(-295,0,-0.86f));for(int i=0;i<200;i++)tick();
        before=body_output[0]; Chassis_Hold();tick();
        assert(fabsf(body_output[0]-before-4.0f)<0.01f);
        unsigned normal_n=1; while(!Chassis_IsSettled() && normal_n<150) {tick();++normal_n;}
        assert(Chassis_IsSettled() && normal_n>n);
    } else if(!strcmp(argv[1],"slew")) {
        assert(Chassis_Body(-200,0,-0.7f));tick();
        float v[3];Mecanum_Forward(geometry(),state.rpm_requested,v);
        assert(v[0]<0 && fabsf(v[0])<=650*0.005f+0.001f);
        assert(fabsf(v[2]/v[0]-0.7f/200)<1e-5f);
        for(int n=0;n<150;n++)tick();
        Chassis_Hold();tick();
        Mecanum_Forward(geometry(),state.rpm_requested,v);
        assert(v[0]<-190);assert(Chassis_MotionBusy());
        assert(!Chassis_Move(100,0,100,550,550));wait_stop();
    } else if(!strcmp(argv[1],"stop")) {
        assert(Chassis_Body(200,0,0));for(int n=0;n<150;n++)tick();
        Chassis_Stop();tick();
        assert(!state.armed);
        for(int i=0;i<4;i++)assert(state.rpm_requested[i]==0 && state.rpm_pending[i]==0);
    } else if(!strcmp(argv[1],"map_search")) {
        assert(Chassis_SetMapHeading(0));
        fake_imu.yaw_deg=5;tick();
        assert(Chassis_MapSearch(-40));tick();
        assert(!line_search && fabsf(body_w)<1e-6f);
        assert(state.yaw_error<0 && body_output[2]<0);
        assert(fabsf(Chassis_MapYaw()-5)<0.1f);
    } else if(!strcmp(argv[1],"map")) {
        Chassis_Hold();wait_stop();
        state.yaw_rad=0;Chassis_BeginPath();
        state.yaw_rad=170*RAD;path_yaw.continuous=170*RAD;
        assert(Chassis_AlignMapAxis());
        assert(fabsf(path_target-STAIR_MAP_TARGET_DEG*RAD)<1e-5f);
        Chassis_Hold();wait_stop();state.yaw_rad=10*RAD;path_yaw.continuous=10*RAD;
        assert(Chassis_AlignMapAxis());
        assert(fabsf(path_target-STAIR_MAP_TARGET_DEG*RAD)<1e-5f); /* Near zero still targets map 180. */
        Chassis_Hold();wait_stop();state.yaw_rad=180*RAD;
        assert(Chassis_MapLateral(30));
        assert(fabsf(dx)<1e-5f && fabsf(dy+1)<1e-5f);
        Chassis_Hold();wait_stop();state.yaw_rad=0;
        assert(Chassis_MapLateral(30));
        assert(fabsf(dx)<1e-5f && fabsf(dy-1)<1e-5f);
        /* Stair grab hold corrects both signs, limits rate, never translates. */
        Chassis_Hold();wait_stop();
        path_diagnostics.result=PATH_RUNNING;path_diagnostics.step=9;path_diagnostics.phase=24;
        fake_imu.yaw_deg=179.5f;
        for(int n=0;n<50;n++) tick();
        assert(body_output[2]>0 && body_output[2]<=1.01f*RAD);
        assert(fabsf(body_output[0])+fabsf(body_output[1])<1e-6f);
        fake_imu.yaw_deg=180.5f;
        for(int n=0;n<100;n++) tick();
        assert(body_output[2]<0 && body_output[2]>=-1.01f*RAD);
        path_diagnostics.step=8;path_diagnostics.phase=0;fake_imu.yaw_deg=176;
        tick();assert(body_output[2]==0); /* Let IsSettled admit the normal large-angle turn. */
        wait_stop();assert(Chassis_AlignMapAxis());assert(path_rotation);
        Chassis_Hold();wait_stop();
        path_diagnostics.step=13;tick();assert(body_output[2]==0);
        path_diagnostics.step=9;Chassis_Stop();tick();
        assert(!state.armed && body_output[2]==0);
    } else if(!strcmp(argv[1],"home")) {
        Chassis_Hold();wait_stop();
        state.x_mm=100;state.y_mm=200;Chassis_BeginPath();
        state.x_mm=1100;state.y_mm=700;state.yaw_rad=0;
        assert(Chassis_ReturnHome());
        float k=(hypotf(1000,500)+100)/hypotf(1000,500);
        float ex=1000*k-40, ey=500*k+40, length=hypotf(ex,ey);
        assert(fabsf(planner.distance-length)<0.01f);
        assert(fabsf(dx+ex/length)<1e-5f);
        assert(fabsf(dy+ey/length)<1e-5f);
        Chassis_Hold();wait_stop();
        state.x_mm=1100;state.y_mm=700;state.yaw_rad=90*RAD;
        assert(!Chassis_ReturnHome());
        assert(Chassis_AlignHome(0));assert(fabsf(rotate_tolerance_deg-0.1f)<1e-6f);
        assert(fabsf(Angle_Wrap(route_heading-map_yaw))<1e-5f);
        Chassis_Hold();wait_stop();
        state.x_mm=100;state.y_mm=200;state.yaw_rad=map_yaw;
        assert(Chassis_ReturnHome() && !Chassis_MotionBusy());
        Chassis_Stop();assert(!Chassis_ReturnHome());
    } else if(!strcmp(argv[1],"line")) {
        fake_imu.yaw_deg=5;tick();
        assert(Chassis_LineSearch(20,0));
        for(int n=0;n<100;n++)tick();
        assert(fabsf(body_output[2])<1e-6f); /* Old route yaw must not fight gray search. */
        assert(!Chassis_CalibrateLine());
        Chassis_Hold();wait_stop();assert(Chassis_CalibrateLine());
        assert(fabsf(Chassis_LineYaw())<1e-6f);
        assert(fabsf(route_heading-5*RAD)<1e-5f);
        assert(Chassis_Move(100,0,100,550,550));tick();
        assert(fabsf(state.yaw_error)<1e-6f);
    } else if(!strcmp(argv[1],"manual")) {
        path_busy=false;fake_imu.yaw_deg=35;tick();
        assert(Chassis_Move(100,0,100,550,550));tick();
        assert(fabsf(state.yaw_error)<1e-6f);
    } else if(!strcmp(argv[1],"align")) {
        assert(Chassis_Rotate(180));fake_imu.yaw_deg=178;tick();
        Chassis_Hold();wait_stop();assert(Chassis_SetLineReference());
        assert(Chassis_AlignZero());
        assert(fabsf(path_target-180*RAD)<1e-5f); /* Residual applied only once. */
        Chassis_Hold();wait_stop();assert(Chassis_Rotate(180));
        assert(fabsf(path_target-360*RAD)<1e-5f); /* Second turn retains route intent. */
    } else if(!strcmp(argv[1],"immediate")) {
        assert(Chassis_Body(200,0,0));for(int n=0;n<150;n++)tick();
        Chassis_HoldImmediate();tick();
        assert(!normal_stopping && !Chassis_MotionBusy());
        for(int i=0;i<4;i++)assert(state.rpm_requested[i]==0);
    } else if(!strcmp(argv[1],"fault")) {
        assert(Chassis_Body(200,0,0));for(int n=0;n<150;n++)tick();
        fake_imu.trust=JY60_TRUST_LOST;tick();
        assert(!state.armed);
        for(int i=0;i<4;i++)assert(state.rpm_requested[i]==0);
    } else { assert(0); }
    puts("ok");return 0;
}
