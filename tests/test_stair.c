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
static int route(unsigned mask, unsigned expected_checks, unsigned balls) {
    PathMission m; Path_Init(&m, send, 0);
    m.result = PATH_RUNNING; m.step = 8;
    PathInput in = {.armed=true, .settled=true, .gray=6};
    ng=checks=moves=rotations=0;
    last=(PathCommand){0};
    for (unsigned t=0; t<20000 && m.result==PATH_RUNNING; t+=100) {
        in.reply = last.kind == PC_STAIR && !(mask & (1U << (last.argument-1))) ? PATH_NONE : PATH_OK;
        Path_Tick(&m,t,&in);
    }
    CHECK(m.result == PATH_DONE);
    CHECK(ng==4 && groups[0]==2 && groups[1]==105 && groups[2]==4 && groups[3]==3);
    CHECK(m.step==13);
    CHECK(checks == expected_checks);
    CHECK(m.grabs == balls);
    CHECK(rotations==0);
    CHECK(moves==14);
    const float expected[]={-350,55,90,117,90,90,90,117,90,100,200,100,200,200};
    for(unsigned i=0;i<14;i++) CHECK(distances[i]==expected[i]);
    return 0;
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
    Path_Tick(&m,100,&in); CHECK(m.step==10);
    in.reply=PATH_OK; Path_Tick(&m,105,&in); CHECK(m.step==10 && m.phase==1);
    Path_Tick(&m,106,&in); CHECK(last.kind==PC_MOVE && last.x==100 && last.y==0);
    in.settled=false; Path_Tick(&m,107,&in); CHECK(m.step==10);
    in.settled=true; Path_Tick(&m,108,&in); CHECK(m.step==11);
    Path_Tick(&m,110,&in);
    CHECK(last.kind==PC_MOVE_ROTATE && last.x==0 && last.y==-1500 && last.angle==180);
    CHECK(last.speed==60 && last.timeout_ms==30000);
    in.settled=false; Path_Tick(&m,1000,&in); CHECK(m.step==11);
    in.settled=true; Path_Tick(&m,2000,&in); CHECK(m.step==11 && m.phase==1);
    Path_Tick(&m,2001,&in); CHECK(last.kind==PC_MOVE && last.x==200 && last.y==0);
    in.settled=false; Path_Tick(&m,2002,&in); CHECK(m.step==11);
    in.settled=true; Path_Tick(&m,2003,&in); CHECK(m.step==12);
    Path_Tick(&m,2005,&in); CHECK(last.kind==PC_BODY && last.y==30 && last.timeout_ms==50000);
    in.gray=6; in.settled=false; Path_Tick(&m,2010,&in); CHECK(last.kind==PC_HOLD);
    Path_Tick(&m,2070,&in); CHECK(m.result==PATH_RUNNING);
    in.settled=true; in.gray=0; Path_Tick(&m,2075,&in);
    CHECK(m.result==PATH_RUNNING && m.step==12 && m.phase==1);
    in.gray=14; Path_Tick(&m,2080,&in);
    CHECK(last.kind==PC_LINE_SEARCH && m.step==12);
    in.gray=6; Path_Tick(&m,2085,&in); CHECK(last.kind==PC_HOLD);
    Path_Tick(&m,2090,&in); Path_Tick(&m,2189,&in); CHECK(m.step==12);
    Path_Tick(&m,2190,&in);
    CHECK(last.kind==PC_MAP_HEADING && last.x==0 && m.step==13);
    Path_Init(&m,send,0); m.result=PATH_RUNNING; m.step=12; m.phase=1; in.gray=14;
    Path_Tick(&m,0,&in); Path_Tick(&m,30000,&in);
    Path_Tick(&m,30005,&in);
    CHECK(m.result==PATH_RUNNING && last.kind==PC_MAP_HEADING && last.x==0 && m.step==13);
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
    CHECK(route(0,8,0)==0); CHECK(route(1,8,1)==0); CHECK(route(3,2,2)==0);
    CHECK(route(0x84,8,2)==0); /* A gap and second ball at the last point. */
    PathMission m; Path_Init(&m,send,0); m.result=PATH_RUNNING; m.step=9;
    PathInput in={.armed=true,.settled=true};
    Path_Tick(&m,0,&in); CHECK(last.kind==PC_BODY && last.timeout_ms==50000);
    CHECK(last.x==0 && last.y==30 && last.speed==0);
    Path_Tick(&m,49999,&in); CHECK(m.result==PATH_RUNNING);
    Path_Tick(&m,50000,&in); CHECK(m.result==PATH_TIMEOUT);
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

