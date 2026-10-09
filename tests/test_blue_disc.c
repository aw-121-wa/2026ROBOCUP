#include "path_mission.h"
#include "path_chassis.h"
#include "path_config.h"
#include <assert.h>
#include <math.h>
static PathCommand last;
static bool send(void *ctx, const PathCommand *c) {
    (void)ctx;
    assert(c->kind==PC_MOVE || c->kind==PC_MOVE_ROTATE || c->kind==PC_ARC || c->kind==PC_BODY ||
           c->kind==PC_HOLD || c->kind==PC_LINE_REFERENCE || c->kind==PC_HOME_ALIGN);
    last=*c; return true;
}
int main(void) {
    PathMission m; Path_Init(&m,send,0);
    PathInput in={.armed=true,.settled=true};
    assert(Path_Start(&m,0,&in));
    Path_Tick(&m,0,&in);
    assert(last.kind==PC_MOVE_ROTATE && fabsf(last.x-1558.8922f)<0.02f && last.y<0 && last.angle==180);
    assert(fabsf(last.speed-139.5f)<.001f && fabsf(last.end_speed-139.5f)<.001f);
    in.motion_done=true;in.settled=false;Path_Tick(&m,5,&in);
    assert(last.kind==PC_ARC && last.x==800 && last.y==160 && last.angle==20);
    assert(fabsf(last.start_speed-139.5f)<.001f && last.end_speed==155);
    Path_Tick(&m,10,&in);
    assert(last.kind==PC_MOVE && fabsf(last.x+2223.9384f)<0.02f && last.y==0);
    Path_Tick(&m,15,&in);
    assert(last.kind==PC_ARC && last.x==50 && last.y==180 && last.angle==-90);
    Path_Tick(&m,20,&in);Path_Tick(&m,25,&in);Path_Tick(&m,30,&in);
    assert(last.kind==PC_BODY && last.y==25);
    in.gray=6;Path_Tick(&m,35,&in);assert(last.kind==PC_HOLD);
    Path_Tick(&m,40,&in);assert(m.result==PATH_RUNNING);
    in.settled=true;Path_Tick(&m,45,&in);assert(m.result==PATH_DONE);
    Path_Tick(&m,50,&in);assert(m.result==PATH_DONE && m.step==3);
#if PATH_BLUE_PILLAR_TEST
    m.result=PATH_RUNNING;m.step=4;m.phase=0;m.waiting=false;
    Path_Tick(&m,55,&in);Path_Tick(&m,60,&in);
    assert(last.kind==PC_MOVE_ROTATE && last.x==1395 && last.y==-715 && last.angle==-90);
    Path_Tick(&m,65,&in);
    assert(last.kind==PC_BODY && last.x==0 && last.y==30);
    Path_Tick(&m,70,&in);assert(last.kind==PC_BODY && last.x==0 && last.y==30);
    in.ir=true;Path_Tick(&m,75,&in);assert(last.kind==PC_HOLD);
    Path_Tick(&m,110,&in);in.map_yaw_deg=90;in.yaw_deg=-180;Path_Tick(&m,115,&in);
    assert(last.kind==PC_BODY && last.x==-76.89408f && last.speed==-58.653f && m.phase==2);
    in.yaw_deg=-180+354;Path_Tick(&m,120,&in);assert(m.phase==2);
    in.yaw_deg=-180-529;Path_Tick(&m,125,&in);assert(m.phase==2);
    in.yaw_deg=-180-530;in.settled=false;Path_Tick(&m,130,&in);
    assert(last.kind==PC_HOLD && m.result==PATH_DONE);
    Path_Tick(&m,450,&in);assert(m.result==PATH_DONE);
    in.settled=true;Path_Tick(&m,455,&in);assert(m.result==PATH_DONE && m.step==6);
    Path_Tick(&m,500,&in);assert(m.result==PATH_DONE && m.step==6);
#endif
    return 0;
}
