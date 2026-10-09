#include "stair_heading.h"
#include "disc_task_config.h"
#include "path_mission.h"
#include "path_chassis.h"
#include <stdio.h>
#include <math.h>
#define CHECK(x) do { if (!(x)) { printf("FAIL %d: %s\n", __LINE__, #x); return 1; } } while (0)
typedef struct { PathCommand commands[512]; unsigned n; } Port;
static bool send(void *ctx, const PathCommand *c) {
    Port *p = ctx; if (p->n == 512) return false; p->commands[p->n++] = *c; return true;
}
static PathInput ready(void) {
    return (PathInput){.armed=true, .settled=true, .gray=6, .ir=true,
        .reply=PATH_OK, .turn_reply=PATH_OK};
}
static unsigned count(const Port *p, PathCommandKind k) {
    unsigned n=0; for(unsigned i=0;i<p->n;i++) if(p->commands[i].kind==k) n++; return n;
}
static int parallel_prep(void) {
    Port p={0}; PathMission m; PathInput in=ready(); Path_Init(&m,send,&p);
    CHECK(Path_Start(&m,0,&in)); Path_Tick(&m,0,&in);
    m.step=3; m.phase=2; in.gray=0; in.reply=PATH_WAIT;
    Path_Tick(&m,0,&in); CHECK(m.prep_pending && m.phase==0);
    Path_Tick(&m,5,&in); CHECK(p.commands[p.n-1].kind==PC_BODY);
    in.gray=6; Path_Tick(&m,10,&in); Path_Tick(&m,15,&in);
    Path_Tick(&m,20,&in); CHECK(m.phase==3 && count(&p,PC_DISC)==0);
    in.reply=PATH_OK; Path_Tick(&m,25,&in); CHECK(count(&p,PC_DISC)==1);
    Path_Init(&m,send,&p); m.result=PATH_RUNNING;m.step=10;in.reply=PATH_WAIT;
    Path_Tick(&m,0,&in); Path_Tick(&m,5,&in);
    CHECK(m.step==11 && m.prep_pending && p.commands[p.n-1].kind==PC_MOVE_ROTATE);
    in.reply=PATH_FAILED; Path_Tick(&m,10,&in);CHECK(m.result==PATH_ERROR);
    return 0;
}
static int startup(void) {
    Port p={0}; PathMission m; PathInput in=ready(); Path_Init(&m,send,&p);
    CHECK(Path_Start(&m,0,&in)); Path_Tick(&m,0,&in); Path_Tick(&m,5,&in);
    CHECK(p.n==3 && p.commands[0].kind==PC_HELLO && p.commands[1].kind==PC_GROUP && p.commands[1].argument==100 && p.commands[2].kind==PC_MOVE);
    CHECK(fabsf(p.commands[2].x-1558.8922f)<0.02f);
    CHECK(fabsf(p.commands[2].y-617.3904f)<0.02f);
    CHECK(p.commands[2].angle==0 && p.commands[2].continuous);
    CHECK(p.commands[2].start_speed==0 && p.commands[2].end_speed==155);
    in.settled=true; in.motion_done=false; Path_Tick(&m,8,&in);
    CHECK(m.part==0 && p.n==3);
    in.settled=false; in.motion_done=true; Path_Tick(&m,10,&in);
    CHECK(m.step==0 && m.part==1 && m.waiting && p.commands[p.n-1].kind==PC_ARC);
    CHECK(p.commands[p.n-1].x==800 && fabsf(p.commands[p.n-1].y-21.6057608f)<.001f);
    CHECK(p.commands[p.n-1].angle==-p.commands[p.n-1].y && p.commands[p.n-1].start_speed==155);
    CHECK(p.commands[p.n-1].speed==155 && p.commands[p.n-1].end_speed==155);
    in.settled=false; in.motion_done=true; Path_Tick(&m,15,&in);
    CHECK(m.step==1 && m.part==0 && m.waiting && p.commands[p.n-1].kind==PC_MOVE);
    CHECK(fabsf(p.commands[p.n-1].x-2028.9384f)<0.02f);
    CHECK(p.commands[p.n-1].start_speed==155 && p.commands[p.n-1].continuous && p.commands[p.n-1].end_speed==25);
    in.motion_done=true; in.settled=false; in.gray=0; Path_Tick(&m,20,&in);
    CHECK(m.step==1 && m.part==1 && p.commands[p.n-1].kind==PC_ARC);
    CHECK(p.commands[p.n-1].x==50 && p.commands[p.n-1].angle==90);
    CHECK(p.commands[p.n-1].start_speed==25 && p.commands[p.n-1].end_speed==25);
    Path_Tick(&m,25,&in);
    CHECK(m.step==3 && m.phase==2 && count(&p,PC_ROTATE)==0);
    CHECK(count(&p,PC_GROUP)==1 && count(&p,PC_HOLD)==0);
    m.step=1; m.part=1; m.waiting=true; in.motion_done=false; in.gray=6;
    Path_Tick(&m,30,&in);
    CHECK(m.step==3 && m.stable && p.commands[p.n-1].kind==PC_HOLD);
    return 0;
}
static int stair_prep_parallel(void) {
    Port p={0}; PathMission m; PathInput in=ready(); Path_Init(&m,send,&p);
    m.result=PATH_RUNNING; m.step=8; m.phase=1; in.map_yaw_deg=STAIR_MAP_TARGET_DEG; in.reply=PATH_WAIT;
    Path_Tick(&m,0,&in); CHECK(m.step==9 && m.prep_pending && p.commands[p.n-1].argument==2);
    in.gray=0; Path_Tick(&m,5,&in); CHECK(p.commands[p.n-1].kind==PC_BODY);
    in.gray=6; Path_Tick(&m,10,&in); Path_Tick(&m,15,&in);
    CHECK(m.phase==1); unsigned n=p.n; Path_Tick(&m,20,&in); CHECK(p.n==n);
    in.reply=PATH_OK; Path_Tick(&m,25,&in);
    CHECK(!m.prep_pending && p.commands[p.n-1].kind==PC_GROUP && p.commands[p.n-1].argument==105);
    return 0;
}
static int post_disc_parallel(void) {
    Port p={0}; PathMission m; PathInput in=ready(); Path_Init(&m,send,&p);
    m.result=PATH_RUNNING; m.step=4; in.reply=PATH_WAIT;
    Path_Tick(&m,0,&in);
    CHECK(m.step==5 && m.prep_pending && p.commands[0].kind==PC_GROUP && p.commands[0].argument==1);
    Path_Tick(&m,5,&in);
    CHECK(p.commands[p.n-1].kind==PC_MOVE_ROTATE && m.prep_pending);
    m.step=6; m.phase=1; m.waiting=false;
    unsigned n=p.n; Path_Tick(&m,10,&in);
    CHECK(p.n==n && m.phase==1);
    in.reply=PATH_OK; Path_Tick(&m,15,&in);
    CHECK(!m.prep_pending && m.phase==4 && p.commands[p.n-1].kind==PC_VISION);
    Path_Init(&m,send,&p); m.result=PATH_RUNNING; m.step=4; in.reply=PATH_WAIT;
    Path_Tick(&m,0,&in); in.reply=PATH_FAILED; Path_Tick(&m,5,&in);
    CHECK(m.result==PATH_ERROR);
    Path_Init(&m,send,&p); m.result=PATH_RUNNING; m.step=4; in.reply=PATH_WAIT;
    Path_Tick(&m,0,&in); Path_Tick(&m,30000,&in);
    CHECK(m.result==PATH_TIMEOUT);
    return 0;
}
static int post_disc_turn(void) {
    Port p={0}; PathMission m; PathInput in=ready(); Path_Init(&m,send,&p);
    m.result=PATH_RUNNING; m.step=5; in.ir=false;
    Path_Tick(&m,0,&in);
    CHECK(p.n==1 && p.commands[0].kind==PC_MOVE_ROTATE);
    CHECK(p.commands[0].x==-1425 && p.commands[0].y==-725);
    CHECK(p.commands[0].angle==90 && p.commands[0].speed==195);
    CHECK(p.commands[0].continuous && p.commands[0].end_speed==30);
    in.settled=false; in.motion_done=true; Path_Tick(&m,5,&in);
    CHECK(m.step==6 && p.commands[p.n-1].kind==PC_BODY);
    CHECK(p.commands[p.n-1].x==0 && p.commands[p.n-1].y==30 && count(&p,PC_HOLD)==0);
    in.motion_done=false; in.ir=true; Path_Tick(&m,15,&in);
    CHECK(p.commands[p.n-1].kind==PC_HOLD);
    Path_Init(&m,send,&p); m.result=PATH_RUNNING; m.step=5;
    Path_Tick(&m,30000,&in); CHECK(m.result==PATH_TIMEOUT);
    return 0;
}
static int line_alignment(void) {
    Port p={0}; PathMission m; PathInput in=ready(); Path_Init(&m,send,&p);
    m.result=PATH_RUNNING; m.step=3; in.gray=2;
    Path_Tick(&m,0,&in);
    CHECK(p.commands[p.n-1].kind==PC_HOLD);
    in.gray=14; in.settled=false; Path_Tick(&m,5,&in);CHECK(m.phase==0);
    in.settled=true;Path_Tick(&m,10,&in);CHECK(p.commands[p.n-1].kind==PC_MAP_SEARCH);
    in.gray=6;Path_Tick(&m,15,&in);Path_Tick(&m,20,&in);Path_Tick(&m,120,&in);Path_Tick(&m,125,&in);
    CHECK(m.phase==1 && p.commands[p.n-1].kind==PC_DISC);
    CHECK(count(&p,PC_ROTATE)==0 && count(&p,PC_HOME_ALIGN)==0);
    Path_Init(&m,send,&p); m.result=PATH_RUNNING; m.step=3; in.gray=0;
    Path_Tick(&m,30000,&in); CHECK(m.result==PATH_TIMEOUT);
    return 0;
}
static int disc_contract(void) {
    Port p={0}; PathMission m; PathInput in=ready(); Path_Init(&m,send,&p);
    m.result=PATH_RUNNING; m.step=3;
    Path_Tick(&m,0,&in); Path_Tick(&m,50,&in); Path_Tick(&m,55,&in);
    CHECK(count(&p,PC_DISC)==1 && count(&p,PC_GROUP)==0);
    for (uint8_t ids=0; ids<DISC_REQUIRED_RFID_COUNT; ++ids) {
        Path_Init(&m,send,&p); m.result=PATH_RUNNING; m.step=3; m.phase=1;
        m.id_count=ids; in.reply=PATH_OK; in.disc_completed=DISC_REQUIRED_RFID_COUNT;
        Path_Tick(&m,100,&in); CHECK(m.result==PATH_DONE);
    }
    CHECK(count(&p,PC_TURN)==0);
    Path_Init(&m,send,&p); m.result=PATH_RUNNING; m.step=3; m.phase=1;
    m.id_count=DISC_REQUIRED_RFID_COUNT; in.reply=PATH_WAIT;
    Path_Tick(&m,100,&in); CHECK(m.result==PATH_DONE && m.disc_depart_pending);
    m.result=PATH_RUNNING; m.step=4; m.waiting=false;
    unsigned groups=count(&p,PC_GROUP);
    Path_Tick(&m,105,&in); CHECK(m.step==5 && m.prep_pending);
    CHECK(count(&p,PC_GROUP)==groups && p.commands[p.n-1].kind==PC_MOVE_ROTATE);
    in.settled=false; in.reply=PATH_OK; Path_Tick(&m,110,&in);
    CHECK(!m.disc_depart_pending && m.prep_pending && count(&p,PC_GROUP)==groups+1);
    in.reply=PATH_WAIT; Path_Tick(&m,115,&in); CHECK(m.prep_pending);
    in.reply=PATH_OK; Path_Tick(&m,120,&in); CHECK(!m.prep_pending);
    in.settled=true;
    Path_Init(&m,send,&p); m.result=PATH_RUNNING; m.step=3; m.phase=1;
    m.entered=0xfffffff0U; in.reply=PATH_WAIT;
    Path_Tick(&m,DISC_TASK_TIMEOUT_MS-17U,&in); CHECK(m.result==PATH_RUNNING);
    Path_Tick(&m,DISC_TASK_TIMEOUT_MS-16U,&in); CHECK(m.result==PATH_TIMEOUT);
    return 0;
}
static int chassis_only(void) {
    Port p={0}; PathMission m; PathInput in=ready(); Path_Init(&m,send,&p);
    m.result=PATH_RUNNING; m.step=6; in.vision_ready=true;
    for(unsigned t=0;t<10000 && m.step==6;t+=10) {
        if(m.phase==2) in.yaw_deg-=1.0f;
        if(m.step==13 && m.point==9) in.gray=m.phase==5?0:6;
        Path_Tick(&m,t,&in);
    }
    CHECK(m.step==8 && m.result==PATH_RUNNING);
    CHECK(count(&p,PC_GROUP)==0 && count(&p,PC_VISION)==1 && count(&p,PC_TURN)==0);
    bool orbit=false;
    for(unsigned i=0;i<p.n;i++) if(p.commands[i].kind==PC_BODY && fabsf(p.commands[i].x+80.94114f)<0.001f && p.commands[i].speed==-58.653f) orbit=true;
    CHECK(orbit);
    p.n=0; Path_Init(&m,send,&p); m.result=PATH_RUNNING; m.step=6;
    in.ir=true; in.settled=false;
    Path_Tick(&m,0,&in);
    CHECK(count(&p,PC_HOLD)==1 && count(&p,PC_BODY)==0);
    Path_Tick(&m,20,&in); CHECK(m.phase==0 && count(&p,PC_BODY)==0);
    Path_Tick(&m,30,&in); CHECK(m.phase==1);
    Path_Tick(&m,40,&in); CHECK(count(&p,PC_BODY)==0);
    in.settled=true; Path_Tick(&m,100,&in); CHECK(m.phase==4);
    Path_Tick(&m,105,&in); CHECK(m.phase==2);

    p.n=0; Path_Init(&m,send,&p); m.result=PATH_RUNNING; m.step=6; m.phase=2;
    m.orbit_yaw=100; in.yaw_deg=-251;
    Path_Tick(&m,100,&in); CHECK(m.phase==2 && p.n==0);
    in.yaw_deg=-258; Path_Tick(&m,105,&in); CHECK(m.step==8 && count(&p,PC_HOLD)==0 && count(&p,PC_ORBIT_EXIT)==1);
    CHECK(p.commands[p.n-1].kind==PC_ORBIT_EXIT && p.commands[p.n-1].x==-10 && p.commands[p.n-1].end_speed==30);
    p.n=0; Path_Init(&m,send,&p); m.result=PATH_RUNNING; m.step=6; in.ir=false;
    Path_Tick(&m,0,&in); CHECK(p.n==1 && p.commands[0].kind==PC_BODY && p.commands[0].y==30);
    in.ir=true; in.yaw_deg=0;
    p.n=0; Path_Init(&m,send,&p); m.result=PATH_RUNNING; m.step=9;
    for(unsigned t=0;t<10000 && m.result==PATH_RUNNING;t+=10) {
        in.map_yaw_deg=m.step<=9?STAIR_MAP_TARGET_DEG:0; in.motion_done=true; unsigned before=p.n;
        if(m.step==13 && m.point==9) in.gray=m.phase==5?0:6;
        Path_Tick(&m,t,&in);
        for(unsigned j=before;j<p.n;j++) if(p.commands[j].kind==PC_MOVE) in.x_mm+=p.commands[j].x;
    }
    CHECK(m.step==13 && m.result==PATH_DONE && m.grabs==0);
    CHECK(count(&p,PC_GROUP)==3 && count(&p,PC_STAIR_SCAN)==3 && count(&p,PC_TURN)==0);
    const float expected[]={5,160,470,230,110,-30,200,200};
    unsigned n=0;
    for(unsigned i=0;i<p.n;i++) if(p.commands[i].kind==PC_MOVE) {
        CHECK(n<8 && fabsf(p.commands[i].x-expected[n])<.01f && p.commands[i].y==0); n++;
    }
    CHECK(n==8);
    CHECK(count(&p,PC_MAP_LATERAL)==2);
    Path_Tick(&m,5000,&in); CHECK(m.step==13 && m.result==PATH_DONE);
    p.n=0; Path_Init(&m,send,&p); m.result=PATH_RUNNING; m.step=12;
    for(unsigned t=0;t<5000 && m.result==PATH_RUNNING;t+=10) { in.map_yaw_deg=m.step<=9?STAIR_MAP_TARGET_DEG:0; if(m.step==13 && m.point==9) in.gray=m.phase==5?0:6; Path_Tick(&m,t,&in); }
    CHECK(m.result==PATH_DONE);
    CHECK(count(&p,PC_MAP_LATERAL)==2 && count(&p,PC_MOVE)==3 && count(&p,PC_ROTATE)==0);
    return 0;
}
static int chassis_errors(void) {
    Port p={0}; PathMission m; PathInput in=ready(); Path_Init(&m,send,&p);
    m.result=PATH_RUNNING; m.step=6; in.ir=false;
    Path_Tick(&m,5000,&in); CHECK(m.result==PATH_RUNNING);
    CHECK(p.commands[p.n-1].timeout_ms==10000);
    Path_Tick(&m,10000,&in); CHECK(m.result==PATH_TIMEOUT);
    Path_Init(&m,send,&p); m.result=PATH_RUNNING; m.step=6; m.phase=2;
    Path_Tick(&m,15000,&in); CHECK(m.result==PATH_TIMEOUT);
    Path_Init(&m,send,&p); m.result=PATH_RUNNING; m.step=9; in.gray=0;
    Path_Tick(&m,5000,&in); CHECK(m.result==PATH_RUNNING);
    Path_Tick(&m,50000,&in); CHECK(m.result==PATH_TIMEOUT);
    Path_Init(&m,send,&p); m.result=PATH_RUNNING; m.step=9; m.ids=0x3e;
    Path_Cancel(&m); CHECK(m.result==PATH_CANCELED && m.ids==0x3e);
    unsigned n=p.n; Path_Tick(&m,5010,&in); CHECK(p.n==n);
    return 0;
}
int main(void) {
    CHECK(line_alignment()==0); CHECK(parallel_prep()==0); CHECK(startup()==0); CHECK(post_disc_turn()==0); CHECK(post_disc_parallel()==0); CHECK(stair_prep_parallel()==0); CHECK(disc_contract()==0);
    CHECK(chassis_only()==0); CHECK(chassis_errors()==0);
    puts("ZHY mission and chassis-only tests passed"); return 0;
}
