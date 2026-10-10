#include "path_mission.h"
#include <stdio.h>
#include <string.h>
#define CHECK(x) do { if (!(x)) { printf("FAIL %d: %s\n",__LINE__,#x); return 1; } } while(0)
static PathMission m;
static PathCommand last;
static unsigned moves, groups, turns, calibrations, homes, group_ids[9], ball_codes[9];
static bool bad;
static unsigned turn_steps;
static bool send(void *ctx,const PathCommand *c) {
    (void)ctx; last=*c;
    if(c->kind==PC_RETURN_HOME) homes++;
    if(c->kind==PC_LINE_CALIBRATE) bad=true;
    if(c->kind==PC_MAP_HEADING) { calibrations++; if(c->x!=0) bad=true; }
    if(c->kind==PC_MOVE && c->x==-30 && c->y==0) return c->acceleration==300 && c->deceleration==300;
    if(c->kind==PC_MOVE) { moves++; if(c->x!=190 || c->y!=0 || c->acceleration!=850 || c->deceleration!=850) bad=true; }
    if(c->kind==PC_TURN) { turns++; turn_steps+=(unsigned)c->x; if(c->argument>1 || c->x<1 || c->x>BALL_SLOT_COUNT/2) bad=true; }
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
    moves=groups=turns=calibrations=homes=turn_steps=0; bad=false; last=(PathCommand){0};
}
/* Independent exhaustive visit search: enforce increasing columns, try each
 * remaining destination in the current column and count physical pocket steps. */
static unsigned optimum(const BallInventory *stock,unsigned current,unsigned col,unsigned used) {
    if(col>3) return 0;
    unsigned best=~0U; bool found=false;
    for(unsigned slot=0;slot<BALL_SLOT_COUNT;slot++) {
        if(!(stock->occupied&(1U<<slot)) || (used&(1U<<slot)) || (stock->code[slot]&15)!=col) continue;
        found=true;
        unsigned d=current>slot?current-slot:slot-current;
        if(d>BALL_SLOT_COUNT/2) d=BALL_SLOT_COUNT-d;
        unsigned cost=d+optimum(stock,slot,col,used|(1U<<slot));
        if(cost<best) best=cost;
    }
    return found?best:optimum(stock,current,col+1,used);
}
static int run(unsigned mask) {
    init();
    const uint8_t order[]={0x23,0x11,0x32,0x13,0x21,0x33,0x12,0x31,0x22};
    unsigned expected=0;
    for(unsigned i=0;i<9;i++) if(mask&(1U<<i)) {
        CHECK(BallInventory_Record(&m.inventory,i+1,order[i])==BALL_ADDED);
        BallInventory_Step(&m.inventory,false); expected++;
    }
    unsigned optimal_steps=optimum(&m.inventory,m.inventory.current,1,0);
    PathInput in={.armed=true,.settled=true,.gray=6,.reply=PATH_OK,.turn_reply=PATH_WAIT};
    unsigned finished_turns=0, turn_wait=0;
    for(unsigned t=0;t<30000 && m.result==PATH_RUNNING;t+=5) {
        if(turns>finished_turns) {
            if(++turn_wait==3) {
                for(unsigned k=0;k<(unsigned)last.x;k++) BallInventory_Step(&m.inventory,last.argument!=0);
                in.turn_reply=PATH_OK; finished_turns=turns; turn_wait=0;
            } else in.turn_reply=PATH_WAIT;
        }
        if(m.point==9) in.gray=m.phase==5?0:6;
        Path_Tick(&m,t,&in);
    }
    CHECK(m.result==PATH_DONE && moves==2 && calibrations==3 && homes==1 && !bad);
    CHECK(groups==expected && m.inventory.placed==expected && !m.inventory.occupied);
    unsigned seen=0, previous_col=0;
    for(unsigned n=0;n<groups;n++) {
        unsigned code=ball_codes[n], col=code&15, row=code>>4;
        CHECK(col>=previous_col && group_ids[n]==112-row);
        previous_col=col;
        for(unsigned i=0;i<9;i++) if(order[i]==code) {
            CHECK((mask&(1U<<i)) && !(seen&(1U<<i))); seen|=1U<<i;
        }
    }
    CHECK(seen==mask);
    CHECK(turn_steps==optimal_steps);
    return 0;
}
static int optimized_order(void) {
    init();
    /* Current pocket already holds row 3: row 3 -> 2 -> 1 avoids a reversal. */
    m.inventory.current=0; CHECK(BallInventory_Record(&m.inventory,1,0x31)==BALL_ADDED);
    m.inventory.current=1; CHECK(BallInventory_Record(&m.inventory,2,0x21)==BALL_ADDED);
    m.inventory.current=5; CHECK(BallInventory_Record(&m.inventory,3,0x11)==BALL_ADDED);
    m.inventory.current=0; m.phase=1;
    PathInput in={.armed=true,.settled=true,.gray=6};
    Path_Tick(&m,0,&in); Path_Tick(&m,5,&in);
    CHECK(last.kind==PC_GROUP && last.argument==109 && turns==0);
    return 0;
}
static int alignment(void) {
 PathInput in={.armed=true,.settled=true,.gray=2};
 init(); m.phase=4;
 Path_Tick(&m,0,&in); CHECK(last.kind==PC_MAP_SEARCH && last.y==-15 && groups==0);
 Path_Tick(&m,2000,&in); CHECK(last.kind==PC_HOLD);
 Path_Tick(&m,2005,&in); CHECK(last.kind==PC_MAP_SEARCH && last.y==15);
 in.gray=6; Path_Tick(&m,2010,&in); CHECK(last.kind==PC_HOLD);
 Path_Tick(&m,2015,&in); Path_Tick(&m,2115,&in); CHECK(m.phase==1);
 return 0;
}
static int errors(void) {
    PathInput in={.armed=true,.settled=true,.gray=6,.reply=PATH_WAIT,.turn_reply=PATH_WAIT};
    init(); CHECK(BallInventory_Record(&m.inventory,1,0x11)==BALL_ADDED);
    BallInventory_Step(&m.inventory,false); m.phase=1;
    Path_Tick(&m,0,&in); CHECK(last.kind==PC_TURN && groups==0);
    Path_Tick(&m,2999,&in); CHECK(m.result==PATH_RUNNING && groups==0);
    Path_Tick(&m,3000,&in); CHECK(m.result==PATH_RUNNING && groups==0 && m.inventory.occupied==1);
    init(); CHECK(BallInventory_Record(&m.inventory,1,0x11)==BALL_ADDED); m.phase=1;
    Path_Tick(&m,0,&in); Path_Tick(&m,5,&in); CHECK(groups==1 && m.inventory.occupied==1);
    Path_Tick(&m,100,&in); CHECK(m.inventory.occupied==1);
    in.reply=PATH_FAILED; Path_Tick(&m,105,&in); CHECK(m.result==PATH_ERROR && m.inventory.occupied==1);
    init(); m.inventory.uncertain=true; Path_Tick(&m,0,&in); CHECK(m.result==PATH_ERROR && moves==0);
    init(); m.point=3; Path_Tick(&m,0,&in); in.settled=false;
    Path_Tick(&m,30000,&in); CHECK(m.result==PATH_RUNNING && groups==0);
    init(); Path_Cancel(&m); CHECK(m.result==PATH_CANCELED && groups==0);
    init(); m.inventory.occupied=1; m.inventory.uid[0]=123; /* Unknown destination receives the first unused code at warehouse entry. */
    in.settled=true; in.reply=PATH_OK;
    for(unsigned t=0;t<1000 && m.result==PATH_RUNNING;t+=5) {
        if(m.point==9) in.gray=m.phase==5?0:6;
        Path_Tick(&m,t,&in);
    }
    CHECK(m.result==PATH_DONE && moves==2 && groups==1 && m.inventory.occupied==0 && m.inventory.inferred==1);
    return 0;
}
static int home_line_stop(void) {
 init(); m.point=9; m.phase=6;
 PathInput tilted={.armed=true,.settled=false,.map_yaw_deg=12};
 Path_Tick(&m,0,&tilted); CHECK(m.phase==6);
 tilted.settled=true;
 Path_Tick(&m,20000,&tilted); CHECK(m.phase==5 && m.result==PATH_RUNNING);
 Path_Tick(&m,20005,&tilted); CHECK(last.kind==PC_RETURN_HOME && last.argument==0);
 for(unsigned mask=0;mask<16;mask++) {
  init(); m.point=9; m.phase=5;
  PathInput in={.armed=true,.settled=true,.gray=15};
  Path_Tick(&m,0,&in); CHECK(homes==1 && m.result==PATH_RUNNING);
  in.settled=false; Path_Tick(&m,5,&in); CHECK(m.phase==5);
  in.gray=0; Path_Tick(&m,10,&in);
  in.settled=true; Path_Tick(&m,11,&in); CHECK(m.phase==16);
  Path_Tick(&m,12,&in); CHECK(last.kind==PC_RETURN_HOME && last.argument==1);
  in.settled=false; in.gray=mask; Path_Tick(&m,15,&in);
  unsigned bits=0;for(unsigned i=0;i<4;i++)bits+=(mask>>i)&1U;
  if(bits>=2) {
   CHECK(m.phase==14 && last.kind==PC_HOLD && m.result==PATH_RUNNING);
   in.gray=0;Path_Tick(&m,20,&in);CHECK(m.phase==14);
   in.settled=true;Path_Tick(&m,25,&in);
   CHECK(m.result==PATH_DONE && last.kind==PC_HOLD);
  } else CHECK(m.phase==16 && m.result==PATH_RUNNING);
 }
 return 0;
}
static int home_advance_braking(void) {
 for(unsigned blue=0;blue<1;++blue) for(unsigned slide=0;slide<=25;slide+=5) {
  init();m.blue=blue;m.point=9;m.phase=16;m.stable=true;m.waiting=true;
  PathInput in={.armed=true,.gray=6,.y_mm=100};
  Path_Tick(&m,0,&in);CHECK(m.phase==14 && last.kind==PC_HOLD);
  in.y_mm=blue ? 100+slide : 100-slide;in.settled=true;
  Path_Tick(&m,100,&in);
  CHECK(m.result==PATH_DONE && last.kind==PC_HOLD);
 }
 return 0;
}
static int home_missing_line(void) {
 init(); m.point=9; m.phase=5;
 PathInput in={.armed=true,.settled=true,.gray=15};
 Path_Tick(&m,0,&in); CHECK(last.kind==PC_RETURN_HOME && last.argument==0);
 in.settled=false;Path_Tick(&m,5,&in);CHECK(m.phase==5);
 in.gray=0;Path_Tick(&m,10,&in);
 in.settled=true;Path_Tick(&m,15,&in);CHECK(m.phase==16);
 Path_Tick(&m,20,&in);CHECK(last.kind==PC_RETURN_HOME && last.argument==1);
 Path_Tick(&m,25,&in);CHECK(m.phase==17);
 Path_Tick(&m,30,&in);CHECK(last.kind==PC_RETURN_HOME && last.argument==2);
 Path_Tick(&m,35,&in);CHECK(m.result==PATH_TIMEOUT); /* No gray: not PATH_DONE. */
 return 0;
}
int main(void) {
    CHECK(home_advance_braking()==0);
    CHECK(home_missing_line()==0);
    CHECK(home_line_stop()==0);
    for(unsigned mask=0;mask<512;mask++) CHECK(run(mask)==0);
    CHECK(optimized_order()==0); CHECK(errors()==0); CHECK(alignment()==0);
    puts("warehouse sorted rows, missing cells, three columns and failure gates passed"); return 0;
}
