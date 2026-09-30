#include "path_mission.h"
#include <stdio.h>
#include <string.h>
#define CHECK(x) do { if (!(x)) { printf("FAIL %d: %s\n",__LINE__,#x); return 1; } } while(0)
static PathMission m;
static PathCommand last;
static unsigned moves, groups, turns, calibrations, homes, group_ids[9], ball_codes[9];
static bool bad;
static bool send(void *ctx,const PathCommand *c) {
    (void)ctx; last=*c;
    if(c->kind==PC_RETURN_HOME) homes++;
    if(c->kind==PC_LINE_CALIBRATE) bad=true;
    if(c->kind==PC_MAP_HEADING) { calibrations++; if(c->x!=0) bad=true; }
    if(c->kind==PC_MOVE) { moves++; if(c->x!=200 || c->y!=0) bad=true; }
    if(c->kind==PC_TURN) { turns++; if(c->argument>1) bad=true; }
    if(c->kind==PC_GROUP) {
        if(groups>=9) { bad=true; return false; }
        group_ids[groups]=c->argument;
        ball_codes[groups++]=m.inventory.code[m.inventory.current];
        if(!(m.inventory.occupied&(1U<<m.inventory.current))) bad=true;
    }
    return true;
}
static void init(void) {
    Path_Init(&m,send,0); m.result=PATH_RUNNING; m.step=13;
    moves=groups=turns=calibrations=homes=0; bad=false; last=(PathCommand){0};
}
static int run(unsigned mask) {
    init();
    const uint8_t order[]={0x23,0x11,0x32,0x13,0x21,0x33,0x12,0x31,0x22};
    unsigned expected=0;
    for(unsigned i=0;i<9;i++) if(mask&(1U<<i)) {
        CHECK(BallInventory_Record(&m.inventory,i+1,order[i])==BALL_ADDED);
        BallInventory_Step(&m.inventory,false); expected++;
    }
    PathInput in={.armed=true,.settled=true,.gray=6,.reply=PATH_OK,.turn_reply=PATH_WAIT};
    unsigned finished_turns=0, turn_wait=0;
    for(unsigned t=0;t<30000 && m.result==PATH_RUNNING;t+=5) {
        if(turns>finished_turns) {
            if(++turn_wait==3) {
                BallInventory_Step(&m.inventory,last.argument!=0);
                in.turn_reply=PATH_OK; finished_turns=turns; turn_wait=0;
            } else in.turn_reply=PATH_WAIT;
        }
        Path_Tick(&m,t,&in);
    }
    CHECK(m.result==PATH_DONE && moves==2 && calibrations==3 && homes==1 && !bad);
    CHECK(groups==expected && m.inventory.placed==expected && !m.inventory.occupied);
    unsigned n=0;
    for(unsigned col=1;col<=3;col++) for(unsigned row=1;row<=3;row++) {
        uint8_t code=(uint8_t)((row<<4)|col);
        for(unsigned i=0;i<9;i++) if(order[i]==code && (mask&(1U<<i))) {
            CHECK(ball_codes[n]==code && group_ids[n]==108+row); n++;
        }
    }
    return 0;
}
static int alignment(void) {
    PathInput in={.armed=true,.settled=true,.gray=7};
    init(); Path_Tick(&m,0,&in); Path_Tick(&m,5,&in);
    CHECK(m.phase==4 && groups==0);
    Path_Tick(&m,10,&in); CHECK(last.kind==PC_MAP_SEARCH && calibrations==1);
    in.gray=6; Path_Tick(&m,20,&in); Path_Tick(&m,25,&in);
    Path_Tick(&m,124,&in); CHECK(m.phase==4);
    Path_Tick(&m,125,&in); CHECK(m.phase==1 && calibrations==1);
    init(); in.gray=0; in.settled=true;
    Path_Tick(&m,0,&in); Path_Tick(&m,5,&in);
    CHECK(last.kind==PC_MAP_SEARCH && last.y==-10);
    Path_Tick(&m,1005,&in); CHECK(last.kind==PC_HOLD);
    in.settled=false; Path_Tick(&m,1010,&in); CHECK(last.kind==PC_HOLD);
    in.settled=true; Path_Tick(&m,1020,&in);
    CHECK(last.kind==PC_MAP_SEARCH && last.y==10);
    Path_Tick(&m,2020,&in); CHECK(last.kind==PC_HOLD);
    Path_Tick(&m,2025,&in); CHECK(last.kind==PC_MAP_SEARCH && last.y==-10);
    Path_Tick(&m,3025,&in); CHECK(last.kind==PC_HOLD);
    Path_Tick(&m,3030,&in); CHECK(m.phase==1 && m.line_skipped);
    return 0;
}
static int errors(void) {
    PathInput in={.armed=true,.settled=true,.gray=6,.reply=PATH_WAIT,.turn_reply=PATH_WAIT};
    init(); CHECK(BallInventory_Record(&m.inventory,1,0x11)==BALL_ADDED);
    BallInventory_Step(&m.inventory,false); m.phase=1;
    Path_Tick(&m,0,&in); CHECK(last.kind==PC_TURN && groups==0);
    Path_Tick(&m,1999,&in); CHECK(m.result==PATH_RUNNING && groups==0);
    Path_Tick(&m,2000,&in); CHECK(m.result==PATH_TIMEOUT && groups==0 && m.inventory.occupied==1);
    init(); CHECK(BallInventory_Record(&m.inventory,1,0x11)==BALL_ADDED); m.phase=1;
    Path_Tick(&m,0,&in); Path_Tick(&m,5,&in); CHECK(groups==1 && m.inventory.occupied==1);
    Path_Tick(&m,100,&in); CHECK(m.inventory.occupied==1);
    in.reply=PATH_FAILED; Path_Tick(&m,105,&in); CHECK(m.result==PATH_ERROR && m.inventory.occupied==1);
    init(); m.inventory.uncertain=true; Path_Tick(&m,0,&in); CHECK(m.result==PATH_ERROR && moves==0);
    init(); m.point=3; Path_Tick(&m,0,&in); in.settled=false;
    Path_Tick(&m,30000,&in); CHECK(m.result==PATH_TIMEOUT && groups==0);
    init(); Path_Cancel(&m); CHECK(m.result==PATH_CANCELED && groups==0);
    init(); m.inventory.occupied=1; m.inventory.uid[0]=123; /* Unknown destination: never guess a row. */
    in.settled=true;
    for(unsigned t=0;t<1000 && m.result==PATH_RUNNING;t+=5) Path_Tick(&m,t,&in);
    CHECK(m.result==PATH_ERROR && moves==2 && groups==0 && m.inventory.occupied==1);
    return 0;
}
int main(void) {
    CHECK(run(0x1ff)==0); CHECK(run(0)==0); CHECK(run(1)==0); CHECK(run(0x112)==0);
    CHECK(errors()==0); CHECK(alignment()==0);
    puts("warehouse sorted rows, missing cells, three columns and failure gates passed"); return 0;
}
