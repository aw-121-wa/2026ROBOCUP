#include "path_mission.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
static PathCommand last;
static unsigned checks, references, searches;
static bool reject;
static bool send(void *ctx,const PathCommand *c) {
    (void)ctx;last=*c;
    if(c->kind==PC_STAIR)checks++;
    if(c->kind==PC_LINE_CALIBRATE) { references++;if(reject)return false; }
    if(c->kind==PC_LINE_SEARCH)searches++;
    return true;
}
static void init(PathMission *m,PathInput *in) {
    Path_Init(m,send,0);m->result=PATH_RUNNING;m->step=9;m->phase=4;
    *in=(PathInput){.armed=true,.settled=true,.gray=6,.imu_yaw_deg=5};
    checks=references=searches=0;reject=false;
}
int main(void) {
    PathMission m;PathInput in;
    const unsigned valid[]={6,15,9};
    for(unsigned i=0;i<3;i++) {
        init(&m,&in);in.gray=valid[i];Path_Tick(&m,0,&in);
        assert(last.kind==PC_HOLD && searches==0);
        Path_Tick(&m,5,&in);Path_Tick(&m,105,&in);
        assert(references==1 && m.phase==2);
    }
    init(&m,&in);in.gray=7;Path_Tick(&m,0,&in);
    assert(last.kind==PC_LINE_SEARCH && last.speed>0);
    in.gray=14;Path_Tick(&m,10,&in);
    assert(last.kind==PC_LINE_SEARCH && last.speed<0);
    Path_Tick(&m,1999,&in);assert(m.line_recovery!=6);
    Path_Tick(&m,2000,&in);assert(last.kind==PC_HOLD);
    Path_Tick(&m,2005,&in);assert(last.kind==PC_MAP_HEADING && last.x==180);
    assert(m.stair_heading_locked && references==0);
    unsigned old=searches;m.phase=4;m.line_active=false;in.gray=0;
    Path_Tick(&m,2010,&in);assert(m.phase==2 && searches==old);
    m.step=13;m.point=0;m.phase=4;Path_Tick(&m,2020,&in);
    Path_Tick(&m,2025,&in);assert(m.warehouse_heading_locked && last.x==0);
    for(unsigned split=2;split<=4;split+=2) {
        init(&m,&in);m.step=13;in.gray=split|9U;
        Path_Tick(&m,0,&in);assert(last.kind==PC_HOLD);
        in.settled=false;Path_Tick(&m,5,&in);assert(last.kind==PC_HOLD);
        in.settled=true;Path_Tick(&m,10,&in);
        assert(last.kind==PC_MAP_SEARCH && last.y==-40 && searches==0);
        in.gray=0;Path_Tick(&m,20,&in);assert(last.kind==PC_MAP_SEARCH);
        in.gray=6;in.settled=false;Path_Tick(&m,30,&in);assert(last.kind==PC_HOLD);
        in.settled=true;Path_Tick(&m,40,&in);Path_Tick(&m,45,&in);Path_Tick(&m,145,&in);
        assert(references==1 && !m.warehouse_heading_locked);
    }
    init(&m,&in);m.step=13;in.gray=13;
    Path_Tick(&m,0,&in);Path_Tick(&m,10,&in);
    Path_Tick(&m,2000,&in);Path_Tick(&m,2005,&in);
    assert(m.warehouse_heading_locked && last.kind==PC_MAP_HEADING && last.x==0);
    init(&m,&in);in.yaw_deg=NAN;Path_Tick(&m,0,&in);assert(m.result==PATH_ERROR);
    init(&m,&in);reject=true;Path_Tick(&m,0,&in);Path_Tick(&m,5,&in);Path_Tick(&m,105,&in);
    assert(m.result==PATH_ERROR);
    puts("sensor directed alignment passed");return 0;
}
