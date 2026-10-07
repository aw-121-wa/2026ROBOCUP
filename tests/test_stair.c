#include "path_mission.h"
#include "rdk_link.h"
#include <stdio.h>
#include <string.h>
#define CHECK(x) do { if (!(x)) { printf("FAIL %d: %s\n", __LINE__, #x); return 1; } } while (0)
static PathCommand last;
static unsigned groups[4], ng, checks, moves, rotations;
static float distances[20];
static bool send(void *ctx, const PathCommand *c) {
    (void)ctx; last = *c;
    if (c->kind == PC_GROUP) groups[ng++] = c->argument;
    if (c->kind == PC_STAIR) checks++;
    if (c->kind == PC_MOVE) distances[moves++] = c->x;
    if (c->kind == PC_ROTATE) { rotations++; }
    return true;
}
static char wire[80];
static bool tx(void *ctx,const char *s,size_t n) {
    (void)ctx; memcpy(wire,s,n); wire[n]=0; return true;
}
static void feed(RdkLink *r,const char *s) {while(*s) Rdk_Feed(r,(uint8_t)*s++);}
static int exit_route(void) {
    PathMission m; Path_Init(&m,send,0); m.result=PATH_RUNNING; m.step=10;
    PathInput in={.armed=true,.settled=true,.reply=PATH_WAIT};
    ng=0;
    Path_Tick(&m,0,&in); CHECK(last.kind==PC_GROUP && last.argument==3);
    Path_Tick(&m,100,&in); CHECK(m.step==11 && last.kind==PC_MOVE_ROTATE);
    in.settled=false; in.reply=PATH_OK; Path_Tick(&m,105,&in); CHECK(m.step==11);
    Path_Tick(&m,110,&in);
    CHECK(last.kind==PC_MOVE_ROTATE && last.x==0 && last.y==-1425 && last.angle==180);
    CHECK(last.speed==155 && last.timeout_ms==30000);
    in.settled=false; Path_Tick(&m,1000,&in); CHECK(m.step==11);
    in.motion_done=true; Path_Tick(&m,1995,&in); CHECK(m.phase==2 && last.kind==PC_ARC && last.y==90 && last.angle==-90);
    Path_Tick(&m,2000,&in); CHECK(m.phase==1);
    Path_Tick(&m,2001,&in); CHECK(last.kind==PC_MOVE && last.x==110 && last.start_speed==45 && last.end_speed==40);
    in.motion_done=false; Path_Tick(&m,2002,&in); CHECK(m.step==11);
    in.motion_done=true; Path_Tick(&m,2003,&in); CHECK(last.kind==PC_ARC && last.x==25 && last.angle==90);
    Path_Tick(&m,2004,&in); CHECK(m.step==12);
    Path_Tick(&m,2005,&in); CHECK(last.kind==PC_BODY && last.y==40 && last.timeout_ms==50000);
    in.gray=6; in.settled=false; Path_Tick(&m,2010,&in); CHECK(last.kind==PC_HOLD);
    Path_Tick(&m,2070,&in); CHECK(m.result==PATH_RUNNING);
    in.settled=true; in.gray=0; Path_Tick(&m,2075,&in);
    CHECK(m.result==PATH_RUNNING && m.step==12 && m.phase==0);
    in.gray=14; Path_Tick(&m,2080,&in);CHECK(last.kind==PC_MAP_SEARCH);
    in.gray=6;Path_Tick(&m,2085,&in);Path_Tick(&m,2090,&in);Path_Tick(&m,2190,&in);
    for(unsigned t=2195;t<2600 && m.step==12;t+=5) Path_Tick(&m,t,&in);
    CHECK(m.step==13 && m.result==PATH_RUNNING);
    Path_Init(&m,send,0); m.result=PATH_RUNNING; m.step=12; in.gray=0;
    Path_Tick(&m,49999,&in); CHECK(m.result==PATH_RUNNING);
    Path_Tick(&m,50000,&in); CHECK(m.result==PATH_TIMEOUT && last.kind==PC_CANCEL);
    Path_Init(&m,send,0); m.result=PATH_RUNNING; m.step=11;
    Path_Tick(&m,0,&in); Path_Cancel(&m); CHECK(m.result==PATH_CANCELED && last.kind==PC_CANCEL);
    Path_Init(&m,send,0); m.result=PATH_RUNNING; m.step=11;
    Path_Tick(&m,0,&in); in.settled=false;
    Path_Tick(&m,30000,&in); CHECK(m.result==PATH_TIMEOUT);
    return 0;
}
int main(void) {
    CHECK(exit_route()==0);
    PathMission m; Path_Init(&m,send,0); m.result=PATH_RUNNING; m.step=9;
    PathInput in={.armed=true,.settled=true,.reply=PATH_OK};
    Path_Tick(&m,0,&in); CHECK(last.kind==PC_BODY && last.timeout_ms==50000);
    CHECK(last.x==0 && last.y==90 && last.speed==0);
    in.y_mm=1079; Path_Tick(&m,95,&in); CHECK(last.y==90);
    in.y_mm=1080; Path_Tick(&m,100,&in); CHECK(last.y==30);
    in.y_mm=0; Path_Tick(&m,105,&in); CHECK(last.y==30); /* Slow mode latches. */
    Path_Tick(&m,49999,&in); CHECK(m.result==PATH_RUNNING);
    Path_Tick(&m,50000,&in); CHECK(m.result==PATH_TIMEOUT);
    Path_Init(&m,send,0); m.result=PATH_RUNNING; m.step=9;
    in=(PathInput){.armed=true,.settled=true,.gray=1,.reply=PATH_OK};
    Path_Tick(&m,0,&in); CHECK(last.kind==PC_HOLD);
    in.gray=0; Path_Tick(&m,5,&in); CHECK(last.kind==PC_MAP_SEARCH);
    in.gray=6; Path_Tick(&m,10,&in); CHECK(last.kind==PC_HOLD);
    in.settled=false; Path_Tick(&m,15,&in); CHECK(m.phase==0);
    RdkLink r; Rdk_Init(&r,0,tx,0); r.stage=2;
    CHECK(Rdk_Begin(&r,"STAIR",3,0,70000)); Rdk_Tick(&r,0);
    CHECK(!strcmp(wire,"STAIR_CHECK 3\r\n"));
    feed(&r,"STAIR_ACK 3\r\nSTAIR_ACTION_DONE 3\r\n");
    uint8_t index; CHECK(Rdk_TakeDiscActionDone(&r,&index) && index==1);
    CHECK(Rdk_SendDiscRfidOk(&r,1)); Rdk_Tick(&r,1);
    CHECK(!strcmp(wire,"STAIR_RFID_OK 3\r\n"));
    feed(&r,"STAIR_DONE 3\r\n"); CHECK(!r.active && r.reply==PATH_OK);
    CHECK(Rdk_Begin(&r,"STAIR",4,2,70000)); Rdk_Tick(&r,2);
    feed(&r,"STAIR_ACK 4\r\nSTAIR_NONE 4\r\n"); CHECK(!r.active && r.reply==PATH_NONE);
    CHECK(Rdk_Begin(&r,"STAIR",5,3,70000)); Rdk_Tick(&r,3);
    feed(&r,"STAIR_ACK 5\r\nSTAIR_DONE 5\r\n"); CHECK(r.locked);
    return 0;
}

