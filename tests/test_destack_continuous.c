#include "path_destack.h"
#include "path_warehouse.h"
#include "path_config.h"
#include "rdk_link.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
static PathInput in;
static PathMission m;
static float vx,vy;
static unsigned row,frame,groups[20],ngroups,turns,stream_starts;
static bool emit(void *ctx,const PathCommand *c)
{
    (void)ctx;
    switch(c->kind) {
    case PC_BODY: vx=c->x;vy=c->y;in.settled=false;break;
    case PC_HOLD:vx=vy=0;in.settled=true;break;
    case PC_MAP_HEADING:break;
    case PC_MAP_SEARCH:vx=0;vy=c->y;in.settled=false;break;
    case PC_WAREHOUSE_STREAM:
        if(c->argument==4) {in.scan_active=in.scan_fresh=false;}
        else {row=c->argument;frame=0;in.scan_active=true;in.scan_fresh=false;++stream_starts;}
        break;
    case PC_GROUP:
        assert(in.settled && !in.scan_active);assert(ngroups<20);
        groups[ngroups++]=c->argument;in.reply=PATH_OK;break;
    case PC_TURN:++turns;break;
    default:assert(c->kind!=PC_CANCEL);break;
    }
    return true;
}
static void init(bool blue)
{
    vx=vy=0;row=frame=ngroups=turns=stream_starts=0;
    m=(PathMission){.blue=blue,.step=13,.result=PATH_RUNNING,.send=emit};
    in=(PathInput){.gray=4,.settled=true,.destack_enabled=true,.warehouse_vision=true,
                   .yaw_deg=blue?180:0,.map_yaw_deg=blue?180:0};
}
static void route(bool blue)
{
    init(blue);unsigned ball_columns=0;unsigned last_point=99;
    for(unsigned tick=0;tick<120000;++tick) {
        in.gray=m.destack.enabled && m.destack.state==DS_ENTRY ? 4 : 6;
        float p=in.x_mm*(blue?-1:1);
        if(in.scan_active) {
            assert(row || m.destack.cleared); /* Never map digits before destacking. */
            in.scan_fresh=true;in.scan_frame=++frame;in.scan_digit=0;in.scan_offset=0;
            float goal=row ? (4-row)*187.0f : roundf(p/187.0f)*187.0f;
            unsigned digit=row ? 4-row : (goal<280?2:goal<467?1:3);
            if(fabsf(p-goal)<45 && !(row && (m.destack.occupied&(1U<<digit)))) {
                in.scan_digit=digit;in.scan_offset=(int16_t)lroundf((goal-p)*2);
            }
        }
        PathWarehouse_Tick(&m,tick*20,&in);
        assert(m.result==PATH_RUNNING);
        if(m.destack.state==DS_BALL && m.point<9 && m.point!=last_point) {
            if(m.point%3==0) ball_columns=ball_columns*10+(m.point/3+1);
            last_point=m.point;
        }
        if(m.point==9) {
            assert(m.destack.cleared && m.destack.occupied==14);
            const unsigned expected[]={112,113,120,115,116,117,118,119,114};
            assert(ngroups==sizeof(expected)/sizeof(expected[0]));
            for(unsigned g=0;g<ngroups;++g) assert(groups[g]==expected[g]);
            assert(ball_columns==321 && turns==0);
            assert(m.warehouse_columns[0]==2 && m.warehouse_columns[1]==1 && m.warehouse_columns[2]==3);
            return;
        }
        in.x_mm+=vx*.07f*(blue?-1:1);in.y_mm+=vy*.07f*(blue?-1:1);
    }
    printf("stuck state=%u row=%u x=%f groups=%u\n",m.destack.state,m.destack.row,in.x_mm,ngroups);assert(0);
}
static void line_and_stale(void)
{
    init(false);PathWarehouse_Tick(&m,0,&in);assert(m.destack.enabled);
    PathWarehouse_Tick(&m,5,&in);
    in.gray=15;PathWarehouse_Tick(&m,20,&in);assert(vx==0);
    PathWarehouse_Tick(&m,40,&in);assert(vx==0 && vy!=0);
    /* Loss recovery cannot accept the image that arrived during lateral motion. */
    in.gray=6;in.scan_frame=10;
    for(unsigned n=60;n<500;n+=20) PathWarehouse_Tick(&m,n,&in);
    in.scan_active=true;in.scan_fresh=false;
    PathWarehouse_Tick(&m,500,&in);assert(vx==0 && vy==0 && ngroups==0);
}
static void braking_and_arm(void)
{
    init(false);in.gray=6;
    m.destack=(PathDestack){.enabled=true,.state=DS_SCAN,.row=3,.direction=-1,.scan_end=-200,.stream=true};
    in.scan_active=true;in.scan_fresh=false;in.settled=false;
    PathWarehouse_Tick(&m,1000,&in);assert(vx==0 && m.destack.braking);
    in.scan_fresh=true;in.scan_frame=1;in.settled=false;
    PathWarehouse_Tick(&m,1500,&in);assert(vx==0 && m.result==PATH_RUNNING);
    in.settled=true;
    PathWarehouse_Tick(&m,1520,&in);assert(vx<0);
    m.destack.state=DS_PICK;m.destack.row=3;m.destack.arm_wait=true;
    in.gray=0;in.reply=PATH_WAIT;in.scan_active=false;in.settled=true;vx=vy=0;
    PathWarehouse_Tick(&m,1540,&in);assert(vx==0 && vy==0 && ngroups==0);
}
static void entry_and_nearest(void)
{
    init(false);in.gray=2; /* Front off, rear on must continue backing. */
    PathWarehouse_Tick(&m,0,&in);
    assert(m.destack.state==DS_ENTRY && vx<0);
    in.gray=4;in.settled=true;PathWarehouse_Tick(&m,20,&in);
    assert(m.destack.state==DS_THIRD && m.destack.columns[0]==187 && m.destack.columns[2]==561);
    m.destack.state=DS_STOP_ROW;m.destack.row=1;
    in.gray=6;in.x_mm=374;in.settled=true;
    PathWarehouse_Tick(&m,40,&in);
    assert(m.destack.cleared && m.destack.state==DS_BALL_MOVE && m.destack.column==1);
    m.destack.state=DS_BALL;m.point=5;
    assert(PathDestack_Advance(&m,60));
    assert(m.destack.column==0 && m.destack.mapped==2); /* Equal distance: first column. */
}
static char wire[100];
static bool tx(void *c,const char *s,size_t n) {(void)c;memcpy(wire,s,n);wire[n]=0;return true;}
static void feed(RdkLink *r,const char *s) {while(*s)Rdk_Feed(r,*s++);}
static void protocol(void)
{
    RdkLink r;Rdk_Init(&r,1,tx,0);r.stage=2;
    assert(Rdk_Scan(&r,3));Rdk_Tick(&r,100);
    assert(!strcmp(wire,"WH_SCAN 1 3\r\n"));
    feed(&r,"WH_FRAME 2 1 1 0\r\n");assert(!r.scan_seen);
    feed(&r,"WH_FRAME 1 1 2 3\r\n");assert(r.scan_seen && r.scan_digit==2);
    feed(&r,"WH_FRAME 1 1 3 0\r\n");assert(r.scan_digit==2);
    assert(!Rdk_Begin(&r,"GROUP",113,100,30000));
    assert(Rdk_Scan(&r,4));Rdk_Tick(&r,120);
    feed(&r,"WH_FRAME 1 2 3 0\r\nWH_DONE 2\r\n");assert(r.scan_active);
    feed(&r,"WH_DONE 1\r\n");assert(!r.scan_active);
    assert(Rdk_Begin(&r,"GROUP",113,140,30000));Rdk_Tick(&r,140);
    feed(&r,"WH_FRAME 1 3 1 0\r\n");assert(!r.locked && r.active);
    feed(&r,"GROUP_ACK 113\r\nGROUP_DONE 113\r\n");assert(r.reply==PATH_OK);
}
int main(void) {protocol();entry_and_nearest();line_and_stale();braking_and_arm();route(false);route(true);return 0;}



