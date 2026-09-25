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
    *in=(PathInput){.armed=true,.settled=true,.gray=15,.imu_yaw_deg=5};
    checks=references=searches=0;reject=false;
}
int main(void) {
    PathMission m;PathInput in;
    /* Any missing probe blocks recognition, including outer-only loss. */
    for(unsigned bit=0;bit<4;bit++) {
        init(&m,&in);in.gray=15^(1U<<bit);Path_Tick(&m,0,&in);
        assert(checks==0 && references==0 && searches==1);
    }
    init(&m,&in);Path_Tick(&m,0,&in);assert(last.kind==PC_HOLD);
    in.settled=false;Path_Tick(&m,100,&in);assert(references==0);
    in.settled=true;Path_Tick(&m,110,&in);Path_Tick(&m,209,&in);assert(references==0);
    /* Sensor chatter restarts the stability timer and alignment. */
    in.gray=14;Path_Tick(&m,210,&in);assert(checks==0);
    in.gray=15;Path_Tick(&m,220,&in);Path_Tick(&m,225,&in);
    Path_Tick(&m,325,&in);assert(references==1 && m.phase==2);
    /* Loss before request must return to alignment. */
    in.gray=7;Path_Tick(&m,330,&in);assert(checks==0 && m.phase==4);
    in.gray=15;Path_Tick(&m,335,&in);Path_Tick(&m,340,&in);Path_Tick(&m,440,&in);
    Path_Tick(&m,445,&in);assert(checks==1);
    init(&m,&in);in.gray=0;Path_Tick(&m,0,&in);Path_Tick(&m,30000,&in);
    assert(m.result==PATH_TIMEOUT && checks==0 && last.kind==PC_CANCEL);
    init(&m,&in);Path_Tick(&m,0,&in);Path_Cancel(&m);Path_Tick(&m,5,&in);
    assert(m.result==PATH_CANCELED && references==0);
    init(&m,&in);in.yaw_deg=NAN;Path_Tick(&m,0,&in);assert(m.result==PATH_ERROR);
    init(&m,&in);reject=true;Path_Tick(&m,0,&in);Path_Tick(&m,5,&in);Path_Tick(&m,105,&in);
    assert(m.result==PATH_ERROR && checks==0);
    init(&m,&in);in.gray=0;Path_Tick(&m,0,&in);in.yaw_deg=11;
    Path_Tick(&m,10,&in);assert(m.result==PATH_ERROR && last.kind==PC_CANCEL);
    /* Reproduce the last stair point: 0111 -> 0101 must abort a bad sweep. */
    init(&m,&in);in.gray=7;Path_Tick(&m,0,&in);
    in.gray=5;Path_Tick(&m,10,&in);Path_Tick(&m,109,&in);
    assert(m.result==PATH_RUNNING && references==0);
    Path_Tick(&m,110,&in);
    assert(m.result==PATH_RUNNING && last.kind==PC_HOLD && references==0);
    in.settled=false;Path_Tick(&m,115,&in);assert(last.kind==PC_HOLD);
    in.settled=true;Path_Tick(&m,120,&in);
    assert(last.kind==PC_MOVE && last.x==0 && last.y==-10 && m.line_retries==1);
    in.settled=false;in.gray=15;Path_Tick(&m,125,&in);assert(references==0);
    in.settled=true;Path_Tick(&m,200,&in);Path_Tick(&m,205,&in);
    Path_Tick(&m,305,&in);assert(references==1 && m.phase==2);
    /* Retry exhaustion still cancels instead of moving indefinitely. */
    init(&m,&in);in.gray=7;Path_Tick(&m,0,&in);m.line_retries=2;
    in.gray=5;Path_Tick(&m,10,&in);Path_Tick(&m,110,&in);
    assert(m.result==PATH_ERROR && last.kind==PC_CANCEL);
    /* A transient loss which recovers must not trip the protection. */
    init(&m,&in);in.gray=7;Path_Tick(&m,0,&in);
    in.gray=5;Path_Tick(&m,10,&in);
    in.gray=7;Path_Tick(&m,80,&in);Path_Tick(&m,120,&in);
    assert(m.result==PATH_RUNNING);
    puts("four-probe gate passed");return 0;
}
