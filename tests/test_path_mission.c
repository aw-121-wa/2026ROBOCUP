#include "disc_task_config.h"
#include "path_mission.h"
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
static int startup(void) {
    Port p={0}; PathMission m; PathInput in=ready(); Path_Init(&m,send,&p);
    CHECK(Path_Start(&m,0,&in)); Path_Tick(&m,0,&in); Path_Tick(&m,5,&in);
    CHECK(p.n==2 && p.commands[0].kind==PC_HELLO && p.commands[1].kind==PC_MOVE);
    CHECK(fabsf(p.commands[1].x-1691.4467f)<0.02f);
    CHECK(count(&p,PC_GROUP)==0); return 0;
}
static int disc_contract(void) {
    Port p={0}; PathMission m; PathInput in=ready(); Path_Init(&m,send,&p);
    m.result=PATH_RUNNING; m.step=3;
    Path_Tick(&m,0,&in); Path_Tick(&m,50,&in);
    CHECK(count(&p,PC_DISC)==1 && count(&p,PC_GROUP)==0);
    in.reply=PATH_OK; /* Completion is independent of RFID, including zero IDs. */
    Path_Tick(&m,100,&in); CHECK(m.result==PATH_DONE && m.id_count==0);
    CHECK(count(&p,PC_TURN)==0 && count(&p,PC_DISC)==1);
    Path_Init(&m,send,&p); m.result=PATH_RUNNING; m.step=3; m.phase=1;
    m.ids=0x3e; m.id_count=5; in.reply=PATH_WAIT;
    Path_Tick(&m,100,&in); CHECK(m.step==3);
    in.reply=PATH_OK; Path_Tick(&m,105,&in); CHECK(m.result==PATH_DONE);
    Path_Init(&m,send,&p); m.result=PATH_RUNNING; m.step=3; m.phase=1;
    m.entered=0xfffffff0U; in.reply=PATH_WAIT;
    Path_Tick(&m,DISC_TASK_TIMEOUT_MS-17U,&in); CHECK(m.result==PATH_RUNNING);
    Path_Tick(&m,DISC_TASK_TIMEOUT_MS-16U,&in); CHECK(m.result==PATH_TIMEOUT);
    return 0;
}
static int chassis_only(void) {
    Port p={0}; PathMission m; PathInput in=ready(); Path_Init(&m,send,&p);
    m.result=PATH_RUNNING; m.step=6; in.reply=PATH_FAILED;
    for(unsigned t=0;t<10000 && m.step==6;t+=10) {
        if(m.phase==2) in.yaw_deg+=1.0f;
        Path_Tick(&m,t,&in);
    }
    CHECK(m.step==7 && m.result==PATH_RUNNING);
    CHECK(count(&p,PC_GROUP)==0 && count(&p,PC_VISION)==0 && count(&p,PC_TURN)==0);
    bool orbit=false;
    for(unsigned i=0;i<p.n;i++) if(p.commands[i].kind==PC_BODY && p.commands[i].x==62 && p.commands[i].speed==49) orbit=true;
    CHECK(orbit);
    p.n=0; Path_Init(&m,send,&p); m.result=PATH_RUNNING; m.step=6; m.phase=2;
    m.orbit_yaw=100; in.yaw_deg=459;
    Path_Tick(&m,100,&in); CHECK(m.phase==2 && p.n==0);
    in.yaw_deg=460; Path_Tick(&m,105,&in); CHECK(m.phase==3 && count(&p,PC_HOLD)==1);
    p.n=0; Path_Init(&m,send,&p); m.result=PATH_RUNNING; m.step=6; in.ir=false;
    Path_Tick(&m,0,&in); CHECK(p.n==1 && p.commands[0].kind==PC_BODY && p.commands[0].y==25);
    in.ir=true;
    p.n=0; Path_Init(&m,send,&p); m.result=PATH_RUNNING; m.step=9;
    for(unsigned t=0;t<5000 && m.step==9;t+=10) Path_Tick(&m,t,&in);
    CHECK(m.step==10 && m.result==PATH_RUNNING);
    CHECK(count(&p,PC_GROUP)==0 && count(&p,PC_VISION)==0 && count(&p,PC_TURN)==0);
    const float expected[]={18,90,117,90,90,90,117,90};
    unsigned n=0;
    for(unsigned i=0;i<p.n;i++) if(p.commands[i].kind==PC_MOVE) {
        CHECK(n<8 && p.commands[i].x==expected[n] && p.commands[i].y==0); n++;
    }
    CHECK(n==8);
    Path_Tick(&m,5000,&in); CHECK(m.step==11); /* No G0 between tasks. */
    p.n=0; Path_Init(&m,send,&p); m.result=PATH_RUNNING; m.step=12;
    for(unsigned t=0;t<5000 && m.result==PATH_RUNNING;t+=10) Path_Tick(&m,t,&in);
    CHECK(m.result==PATH_DONE);
    const float wx[]={0,200,200,200,-200,-200};
    n=0;
    for(unsigned i=0;i<p.n;i++) {
        CHECK(p.commands[i].kind!=PC_GROUP && p.commands[i].kind!=PC_VISION && p.commands[i].kind!=PC_TURN);
        if(p.commands[i].kind==PC_MOVE) {
            CHECK(n<6 && p.commands[i].x==wx[n]);
            CHECK(p.commands[i].y==(n==0 ? -50 : 0)); n++;
        }
    }
    CHECK(n==6 && count(&p,PC_ROTATE)==1);
    return 0;
}
static int chassis_errors(void) {
    Port p={0}; PathMission m; PathInput in=ready(); Path_Init(&m,send,&p);
    m.result=PATH_RUNNING; m.step=6; in.ir=false;
    Path_Tick(&m,5000,&in); CHECK(m.result==PATH_TIMEOUT);
    Path_Init(&m,send,&p); m.result=PATH_RUNNING; m.step=6; m.phase=2;
    Path_Tick(&m,15000,&in); CHECK(m.result==PATH_TIMEOUT);
    Path_Init(&m,send,&p); m.result=PATH_RUNNING; m.step=9; in.gray=0;
    Path_Tick(&m,5000,&in); CHECK(m.result==PATH_TIMEOUT);
    Path_Init(&m,send,&p); m.result=PATH_RUNNING; m.step=9; m.ids=0x3e;
    Path_Cancel(&m); CHECK(m.result==PATH_CANCELED && m.ids==0x3e);
    unsigned n=p.n; Path_Tick(&m,5010,&in); CHECK(p.n==n);
    return 0;
}
int main(void) {
    CHECK(startup()==0); CHECK(disc_contract()==0);
    CHECK(chassis_only()==0); CHECK(chassis_errors()==0);
    puts("ZHY mission and chassis-only tests passed"); return 0;
}
