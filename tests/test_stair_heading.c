#include "path_mission.h"
#include "stair_heading.h"
#include <stdio.h>
#define CHECK(x) do { if (!(x)) { printf("FAIL %d: %s\n", __LINE__, #x); return 1; } } while (0)
static unsigned checks, corrections, holds, moves;
static bool reject;
static PathCommand last;
static bool send(void *ctx,const PathCommand *c) {
    (void)ctx; last=*c;
    if(c->kind==PC_STAIR) checks++;
    if(c->kind==PC_ALIGN_ZERO) { corrections++; if(reject) return false; }
    if(c->kind==PC_HOLD) holds++;
    if(c->kind==PC_MOVE) moves++;
    return true;
}
static void init(PathMission *m,PathInput *in) {
    Path_Init(m,send,0); m->result=PATH_RUNNING; m->step=9; m->phase=1;
    *in=(PathInput){.armed=true,.settled=true,.gray=6,.reply=PATH_OK,.yaw_deg=100,.imu_yaw_deg=5};
    checks=corrections=holds=moves=0; reject=false;
    Path_Tick(m,0,in); Path_Tick(m,5,in);
}
static int gate(void) {
    PathMission m; PathInput in; init(&m,&in);
    Path_Tick(&m,10,&in); CHECK(checks==0 && corrections==1);
    in.settled=false; Path_Tick(&m,100,&in); CHECK(corrections==1 && checks==0);
    in.settled=true; in.imu_yaw_deg=0.19f; Path_Tick(&m,110,&in);
    Path_Tick(&m,159,&in); CHECK(checks==0);
    in.imu_yaw_deg=0.21f; Path_Tick(&m,160,&in); CHECK(corrections==2 && checks==0);
    in.imu_yaw_deg=-0.19f; Path_Tick(&m,170,&in); Path_Tick(&m,269,&in); CHECK(checks==0);
    Path_Tick(&m,270,&in); Path_Tick(&m,275,&in); CHECK(checks==1 && last.kind==PC_STAIR);
    return 0;
}
static int all_points(void) {
    PathMission m; PathInput in; init(&m,&in);
    unsigned corrected=0;
    for(unsigned t=10;t<10000 && m.result==PATH_RUNNING;t+=5) {
        in.reply=PATH_NONE;
        if(last.kind==PC_MOVE) in.imu_yaw_deg=3;
        if(corrections>corrected) { corrected=corrections; in.imu_yaw_deg=0; }
        Path_Tick(&m,t,&in);
    }
    CHECK(m.result==PATH_DONE && checks==8 && corrections==8 && moves==7);
    return 0;
}
static int failures(void) {
    PathMission m; PathInput in; init(&m,&in); Path_Tick(&m,10,&in);
    in.settled=false; Path_Tick(&m,304,&in); CHECK(holds==0 && checks==0);
    Path_Tick(&m,305,&in); CHECK(m.result==PATH_RUNNING && checks==0 && holds==1);
    unsigned before=corrections;
    in.settled=true; Path_Tick(&m,310,&in);
    CHECK(checks==1 && corrections==before && last.kind==PC_STAIR);
    in.reply=PATH_NONE; Path_Tick(&m,315,&in); Path_Tick(&m,320,&in);
    CHECK(last.kind==PC_MOVE && m.result==PATH_RUNNING);
    init(&m,&in); Path_Tick(&m,10,&in); Path_Cancel(&m); unsigned n=corrections;
    Path_Tick(&m,20,&in); CHECK(m.result==PATH_CANCELED && corrections==n && checks==0);
    init(&m,&in); in.fault=true; Path_Tick(&m,10,&in); CHECK(m.result==PATH_ERROR && checks==0);
    init(&m,&in); reject=true; Path_Tick(&m,10,&in); CHECK(m.result==PATH_ERROR && checks==0);
    init(&m,&in); in.imu_yaw_deg=NAN; Path_Tick(&m,10,&in); CHECK(m.result==PATH_ERROR);
    return 0;
}
int main(void) {
    CHECK(fabsf(StairHeading_Error(725)+5)<0.001f);
    CHECK(fabsf(StairHeading_Error(-725)-5)<0.001f);
    CHECK(fabsf(StairHeading_Error(359)-1)<0.001f);
    CHECK(fabsf(StairHeading_Error(-359)+1)<0.001f);
    CHECK(gate()==0); CHECK(all_points()==0); CHECK(failures()==0);
    puts("stair heading: gate, drift, 8 points, wrap, timeout, cancel, fault passed"); return 0;
}
