#include "path_mission.h"
#include "rdk_link.h"
#include <stdio.h>
#include <string.h>
#define CHECK(x) do { if (!(x)) { printf("FAIL %d: %s\n", __LINE__, #x); return 1; } } while(0)
static PathCommand last;
static char wire[80];
static bool send(void *ctx, const PathCommand *c) { (void)ctx; last=*c; return true; }
static bool tx(void *ctx, const char *s, size_t n) { (void)ctx; memcpy(wire,s,n); wire[n]=0; return true; }
static void feed(RdkLink *r,const char *s) { while(*s) Rdk_Feed(r,(uint8_t)*s++); }
int main(void) {
    RdkLink r; Rdk_Init(&r,0,tx,0);
    CHECK(Rdk_Begin(&r,"HELLO",0,0,2000)); Rdk_Tick(&r,0); feed(&r,"PONG\r\n");
    CHECK(Rdk_Begin(&r,"GROUP",0,1,30000)); Rdk_Tick(&r,1);
    CHECK(!strcmp(wire,"GROUP 0\r\n"));
    feed(&r,"GROUP_ACK 0\r\n"); CHECK(r.active);
    feed(&r,"GROUP_DONE 0\r\n"); CHECK(!r.active && r.reply==PATH_OK);
    PathMission m; Path_Init(&m,send,0);
    PathInput in={.armed=true,.settled=true,.reply=PATH_OK,.gray=6,.ir=true};
    m.result=PATH_RUNNING; m.step=2; m.waiting=true;
    Path_Tick(&m,10,&in); Path_Tick(&m,15,&in);
    CHECK(last.kind==PC_GROUP && last.argument==100);
    Path_Init(&m,send,0); m.result=PATH_RUNNING; m.step=4;
    Path_Tick(&m,20,&in); CHECK(last.kind==PC_GROUP && last.argument==1);
    in.reply=PATH_WAIT; Path_Tick(&m,25,&in); CHECK(m.step==4);
    in.reply=PATH_OK; Path_Tick(&m,30,&in); CHECK(m.step==5);
    Path_Init(&m,send,0); m.result=PATH_RUNNING; m.step=6;
    Path_Tick(&m,40,&in); Path_Tick(&m,70,&in); Path_Tick(&m,75,&in);
    CHECK(last.kind==PC_VISION); /* RDK prepares G103 before reporting READY. */
    in.reply=PATH_WAIT; in.vision_ready=true;
    Path_Tick(&m,80,&in); CHECK(last.kind==PC_BODY && m.phase==2 && last.x==-58.9f && last.speed==-49);
    in.settled=false; in.ball_index=1; in.yaw_deg=-120;
    Path_Tick(&m,85,&in); CHECK(last.kind==PC_HOLD && m.phase==5);
    Path_Tick(&m,90,&in); CHECK(last.kind==PC_HOLD);
    in.settled=true; Path_Tick(&m,95,&in);
    CHECK(last.kind==PC_PILLAR_STOPPED && last.argument==1 && m.phase==6);
    Path_RecordId(&m,0x12345678); CHECK(m.id_count==1);
    Path_Tick(&m,20000,&in); CHECK(m.phase==6); /* RFID wait is not orbit time. */
    in.resume_index=1; Path_Tick(&m,20005,&in); CHECK(m.phase==2 && last.kind==PC_BODY && last.x==-58.9f && last.speed==-49);
    in.yaw_deg=-351; Path_Tick(&m,20010,&in); CHECK(m.phase==2);
    in.yaw_deg=-354; Path_Tick(&m,20015,&in); CHECK(m.phase==3 && last.kind==PC_HOLD);
    Path_Tick(&m,20020,&in); CHECK(last.kind==PC_PILLAR_END);
    in.reply=PATH_OK; Path_Tick(&m,20025,&in); CHECK(m.step==7);

    CHECK(Rdk_Begin(&r,"PILLAR",0,2,300000)); Rdk_Tick(&r,2);
    feed(&r,"PILLAR_ACK\r\nPILLAR_READY\r\nPILLAR_BALL 1\r\n");
    CHECK(r.ball_index==1 && r.pillar_ready && !r.locked);
    CHECK(!Rdk_PillarEnd(&r)); CHECK(!Rdk_PillarStopped(&r,2));
    CHECK(Rdk_PillarStopped(&r,1)); Rdk_Tick(&r,3);
    CHECK(!strcmp(wire,"PILLAR_STOPPED 1\r\n"));
    feed(&r,"PILLAR_ACTION_DONE 1\r\n");
    uint8_t index=0; CHECK(Rdk_TakeDiscActionDone(&r,&index) && index==1);
    CHECK(Rdk_SendDiscRfidOk(&r,1)); Rdk_Tick(&r,4);
    CHECK(!strcmp(wire,"PILLAR_RFID_OK 1\r\n"));
    feed(&r,"PILLAR_RESUME 1\r\n"); CHECK(r.resume_index==1 && !r.locked);
    CHECK(Rdk_PillarEnd(&r)); Rdk_Tick(&r,5); feed(&r,"PILLAR_DONE\r\n");
    CHECK(!r.active && r.reply==PATH_OK);
    CHECK(Rdk_Begin(&r,"PILLAR",0,10,300000)); Rdk_Tick(&r,10);
    feed(&r,"PILLAR_ACK\r\nPILLAR_READY\r\n");
    CHECK(Rdk_PillarEnd(&r)); Rdk_Tick(&r,11);
    feed(&r,"PILLAR_BALL 1\r\nPILLAR_DONE\r\n");
    CHECK(!r.locked && r.reply==PATH_OK); /* Detection already on wire when END sent. */
    CHECK(Rdk_Begin(&r,"GROUP",0,20,30)); Rdk_Tick(&r,20);
    feed(&r,"GROUP_ACK 0\r\n"); Rdk_Tick(&r,50); Rdk_Tick(&r,51);
    CHECK(r.locked && !strcmp(wire,"DISC_CANCEL\r\n"));
    Path_Init(&m,send,0); m.result=PATH_RUNNING; m.step=6; m.phase=4;
    in.reply=PATH_WAIT; in.vision_ready=true; in.ball_index=1; in.resume_index=0;
    Path_Tick(&m,100,&in); CHECK(last.kind==PC_HOLD && m.phase==5);
    m.phase=6; in.resume_index=1; in.ball_index=2;
    Path_Tick(&m,105,&in); CHECK(last.kind==PC_HOLD && m.phase==5);
    puts("action sequence passed"); return 0;
}
