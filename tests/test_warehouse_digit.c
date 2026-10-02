#include "path_warehouse.h"
#include <stdio.h>
#define CHECK(x) do { if(!(x)) {printf("FAIL %d: %s\n",__LINE__,#x); return 1;} } while(0)
static PathMission m;
static PathCommand last;
static unsigned queries, moves, holds, groups, turns, offsets;
static unsigned codes[9];
static bool send(void *ctx,const PathCommand *c) {
 (void)ctx; last=*c;
 if(c->kind==PC_WAREHOUSE_DIGIT) queries++;
 if(c->kind==PC_BODY || c->kind==PC_LINE_SEARCH) return false;
 if(c->kind==PC_MOVE) {if(c->x<0) return false; moves++;}
 if(c->kind==PC_MAP_LATERAL) {offsets++; if(c->y!=-15) return false;}
 if(c->kind==PC_HOLD) holds++;
 if(c->kind==PC_TURN) {turns++; for(unsigned k=0;k<(unsigned)c->x;k++) BallInventory_Step(&m.inventory,c->argument!=0);}
 if(c->kind==PC_GROUP) {if(groups>=9) return false; codes[groups++]=m.inventory.code[m.inventory.current];}
 return true;
}
static void init(void) {
 Path_Init(&m,send,0); m.result=PATH_RUNNING; m.step=13;
 queries=moves=holds=groups=turns=offsets=0;
 for(unsigned col=1;col<=3;col++) for(unsigned row=1;row<=3;row++) {
  BallInventory_Record(&m.inventory,row*3+col,(uint8_t)((row<<4)|col)); BallInventory_Step(&m.inventory,false);
 }
}
static int run(bool fallback,const unsigned digits[3]) {
 init(); unsigned answered=0;
 PathInput in={.armed=true,.settled=true,.gray=6,.reply=PATH_OK,.turn_reply=PATH_OK,.warehouse_vision=true};
 for(unsigned t=0;t<20000 && m.result==PATH_RUNNING;t+=5) {
  if(queries>answered) {in.warehouse_digit_reply=fallback?PATH_NONE:PATH_OK; in.warehouse_digit=(uint8_t)digits[answered++];}
  if(fallback && answered && queries==1 && m.phase!=8 && m.phase!=10) {in.warehouse_digit_reply=PATH_OK; in.warehouse_digit=3;}
  Path_Tick(&m,t,&in);
 }
 CHECK(m.result==PATH_DONE && groups==9 && !m.inventory.occupied);
 CHECK(queries==(fallback?1:3) && moves==(fallback?2U:0U) && offsets==1);
 for(unsigned i=0;i<9;i++) CHECK((codes[i]&15)==(fallback?i/3+1:digits[i/3]));
 return 0;
}
static int prep_gate(void) {
 init(); m.prep_pending=true;
 PathInput in={.armed=true,.settled=true,.gray=6,.reply=PATH_WAIT,.warehouse_vision=true};
 Path_Tick(&m,0,&in); CHECK(queries==0 && m.phase==0);
 in.reply=PATH_OK; Path_Tick(&m,5,&in); CHECK(queries==0 && offsets==1 && m.phase==11);
 Path_Tick(&m,10,&in); CHECK(queries==1 && m.phase==10);
 return 0;
}
static int moving_stop(void) {
 init(); m.point=3; m.warehouse_ignore_line=false; m.warehouse_mode=2; m.warehouse_columns[0]=3; m.warehouse_used=1U<<3;
 PathInput in={.armed=true,.settled=true,.gray=0,.warehouse_vision=true,.warehouse_digit_reply=PATH_WAIT};
 Path_Tick(&m,0,&in); CHECK(offsets==0 && moves==0 && queries==0 && m.phase==11);
 Path_Tick(&m,5,&in); CHECK(queries==1 && m.phase==10);
 in.warehouse_ready=true; Path_Tick(&m,10,&in); CHECK(moves==1 && m.phase==8);
 in.settled=false; in.warehouse_digit_reply=PATH_OK; in.warehouse_digit=1;
 Path_Tick(&m,15,&in); CHECK(last.kind==PC_HOLD && holds==1 && groups==0);
 Path_Tick(&m,20,&in); CHECK(m.phase==9);
 in.settled=true; Path_Tick(&m,25,&in); CHECK(m.phase==4);
 Path_Tick(&m,30,&in); CHECK(last.kind==PC_MAP_SEARCH && last.y==-10 && groups==0);
 Path_Tick(&m,2030,&in); CHECK(last.kind==PC_HOLD && groups==0);
 Path_Tick(&m,2035,&in); CHECK(last.kind==PC_MAP_SEARCH && last.y==10 && groups==0);
 in.gray=6; Path_Tick(&m,2040,&in); CHECK(last.kind==PC_HOLD);
 Path_Tick(&m,2045,&in); Path_Tick(&m,2145,&in); CHECK(m.phase==1);
 in.turn_reply=PATH_OK;
 for(unsigned t=2150;t<2450 && !groups;t+=5) Path_Tick(&m,t,&in);
 CHECK(groups==1 && in.gray==6);
 return 0;
}
static int first_column_creep(void) {
 init();
 PathInput in={.armed=true,.settled=true,.gray=6,.warehouse_vision=true,.warehouse_digit_reply=PATH_WAIT};
 Path_Tick(&m,0,&in); CHECK(offsets==1 && queries==0 && m.phase==11);
 in.settled=false; Path_Tick(&m,5,&in); CHECK(queries==0);
 in.settled=true; Path_Tick(&m,10,&in); CHECK(queries==1 && m.phase==10);
 in.warehouse_ready=true; Path_Tick(&m,15,&in); CHECK(moves==1 && m.phase==8);
 in.warehouse_digit_reply=PATH_OK; in.warehouse_digit=2;
 Path_Tick(&m,20,&in); CHECK(holds==1 && m.phase==9 && m.warehouse_mode==2);
 Path_Tick(&m,25,&in); CHECK(m.phase==4 && groups==0);
 return 0;
}

static int offset_line_check(void) {
 init();
 PathInput in={.armed=true,.settled=true,.gray=0,.warehouse_vision=true,.warehouse_digit_reply=PATH_WAIT};
 Path_Tick(&m,0,&in); CHECK(offsets==1 && queries==0);
 Path_Tick(&m,5,&in); CHECK(m.phase==10 && queries==1);
 in.warehouse_ready=true; Path_Tick(&m,10,&in); CHECK(moves==1 && m.phase==8);
 return 0;
}
int main(void) {
 const unsigned orders[6][3]={{1,2,3},{1,3,2},{2,1,3},{2,3,1},{3,1,2},{3,2,1}};
 for(unsigned i=0;i<6;i++) CHECK(run(false,orders[i])==0);
 CHECK(offset_line_check()==0); CHECK(run(true,orders[0])==0); CHECK(moving_stop()==0); CHECK(prep_gate()==0); CHECK(first_column_creep()==0);
 puts("warehouse digits passed"); return 0;
}
