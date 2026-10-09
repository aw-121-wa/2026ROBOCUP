#include "path_warehouse.h"
#include <stdio.h>
#include <math.h>
#define CHECK(x) do { if(!(x)) {printf("FAIL %d: %s\n",__LINE__,#x); return 1;} } while(0)
static PathMission m;
static PathCommand last;
static unsigned queries, moves, holds, groups, turns, offsets, searches;
static unsigned codes[9];
static bool send(void *ctx,const PathCommand *c) {
 (void)ctx; last=*c;
 if(c->kind==PC_WAREHOUSE_DIGIT) queries++;
 if(c->kind==PC_MAP_SEARCH) searches++;
 if(c->kind==PC_BODY || c->kind==PC_LINE_SEARCH) return false;
 if(c->kind==PC_MOVE || c->kind==PC_FINISH_FORWARD) {if(c->x<0) return c->x==-30 && c->y==0; moves++;}
 if(c->kind==PC_MAP_LATERAL) {if(m.point==9) return c->y==-20; offsets++; if(c->y!=-19) return false;}
 if(c->kind==PC_HOLD) holds++;
 if(c->kind==PC_TURN) {turns++; for(unsigned k=0;k<(unsigned)c->x;k++) BallInventory_Step(&m.inventory,c->argument!=0);}
 if(c->kind==PC_GROUP) {if(groups>=9) return false; codes[groups++]=m.inventory.code[m.inventory.current];}
 return true;
}
static void init(void) {
 Path_Init(&m,send,0); m.result=PATH_RUNNING; m.step=13;
 queries=moves=holds=groups=turns=offsets=searches=0;
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
  if(m.point==9) in.gray=m.phase==5?0:6;
  Path_Tick(&m,t,&in);
 }
 CHECK(m.result==PATH_DONE && groups==9 && !m.inventory.occupied);
 if(!fallback) CHECK(searches==0);
 CHECK(queries==(fallback?1:2) && moves==(fallback?2U:3U) && offsets==1);
 for(unsigned i=0;i<9;i++) CHECK((codes[i]&15)==(fallback?i/3+1:digits[i/3]));
 return 0;
}
static int prep_gate(void) {
 init(); m.prep_pending=true;
 PathInput in={.armed=true,.settled=true,.gray=6,.reply=PATH_WAIT,.warehouse_vision=true};
 Path_Tick(&m,0,&in); CHECK(queries==0 && m.phase==0);
 in.reply=PATH_OK; Path_Tick(&m,4,&in); CHECK(last.kind==PC_MOVE && last.x==-30 && offsets==0 && queries==0);
 Path_Tick(&m,5,&in); CHECK(queries==0 && offsets==1 && m.phase==11);
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
 Path_Tick(&m,15,&in); CHECK(last.kind==PC_FINISH_FORWARD && last.x==25 && holds==0 && groups==0);
 Path_Tick(&m,20,&in); CHECK(m.phase==9);
 Path_Tick(&m,25,&in); CHECK(m.phase==9 && last.kind==PC_FINISH_FORWARD && last.x==25 && groups==0);
 in.settled=false; Path_Tick(&m,26,&in); CHECK(m.phase==9 && groups==0);
 in.settled=true; Path_Tick(&m,27,&in); CHECK(m.phase==WAREHOUSE_SELECT_BALL);
 in.turn_reply=PATH_OK;
 for(unsigned t=30;t<330 && !groups;t+=5) Path_Tick(&m,t,&in);
 CHECK(groups==1 && in.gray==0 && searches==0 && holds==0);

 return 0;
}
static int first_column_creep(void) {
 init();
 PathInput in={.armed=true,.settled=true,.gray=6,.warehouse_vision=true,.warehouse_digit_reply=PATH_WAIT};
 Path_Tick(&m,0,&in); CHECK(last.kind==PC_MOVE && last.x==-30 && offsets==0 && queries==0);
 Path_Tick(&m,1,&in); CHECK(offsets==1 && queries==0 && m.phase==11);
 in.settled=false; Path_Tick(&m,5,&in); CHECK(queries==0);
 in.settled=true; Path_Tick(&m,10,&in); CHECK(queries==1 && m.phase==10);
 in.warehouse_ready=true; Path_Tick(&m,15,&in); CHECK(moves==1 && m.phase==8);
 in.warehouse_digit_reply=PATH_OK; in.warehouse_digit=2;
 Path_Tick(&m,20,&in); CHECK(holds==0 && m.phase==9 && m.warehouse_mode==2 && last.kind==PC_FINISH_FORWARD && last.x==25);
 in.settled=false; Path_Tick(&m,25,&in); CHECK(m.phase==9 && last.kind==PC_FINISH_FORWARD && last.x==25 && groups==0);
 in.settled=true;
 Path_Tick(&m,30,&in); CHECK(m.phase==WAREHOUSE_SELECT_BALL && groups==0 && searches==0);
 return 0;
}

static int offset_line_check(void) {
 init();
 PathInput in={.armed=true,.settled=true,.gray=0,.warehouse_vision=true,.warehouse_digit_reply=PATH_WAIT};
 Path_Tick(&m,0,&in); CHECK(last.kind==PC_MOVE && last.x==-30 && offsets==0 && queries==0);
 Path_Tick(&m,1,&in); CHECK(offsets==1 && queries==0);
 Path_Tick(&m,5,&in); CHECK(m.phase==10 && queries==1);
 in.warehouse_ready=true; Path_Tick(&m,10,&in); CHECK(moves==1 && m.phase==8);
 return 0;
}
static int entry_back(void) {
 init();
 PathInput in={.armed=true,.settled=true,.gray=6,.warehouse_vision=true,.map_yaw_deg=90};
 Path_Tick(&m,0,&in);
 CHECK(last.kind==PC_MOVE && fabsf(last.x)<0.001f && fabsf(last.y-30)<0.001f);
 CHECK(m.phase==WAREHOUSE_ENTRY_BACK && queries==0 && offsets==0);
 in.settled=false;Path_Tick(&m,100,&in);CHECK(queries==0 && offsets==0);
 in.settled=true;Path_Tick(&m,200,&in);CHECK(offsets==1 && queries==0 && last.y==-19);
 in.settled=false;Path_Tick(&m,205,&in);CHECK(queries==0);
 in.settled=true;Path_Tick(&m,210,&in);CHECK(queries==1);
 init();in.map_yaw_deg=0;Path_Tick(&m,0,&in);in.settled=false;
 Path_Tick(&m,5000,&in);CHECK(m.result==PATH_RUNNING && offsets==0 && queries==0);
 return 0;
}
static int inferred_column(void) {
 init(); m.point=6; m.warehouse_mode=WAREHOUSE_DIGIT_ORDER;
 m.warehouse_columns[0]=3; m.warehouse_columns[1]=1; m.warehouse_used=(1U<<3)|(1U<<1);
 PathInput in={.armed=true,.settled=true,.warehouse_vision=true,.warehouse_digit_reply=PATH_WAIT};
 Path_Tick(&m,0,&in);
 CHECK(last.kind==PC_MOVE && last.x==180 && queries==0 && m.warehouse_columns[2]==2);
 CHECK(m.phase==WAREHOUSE_INFERRED_MOVE && groups==0);
 in.settled=false; Path_Tick(&m,10000,&in); CHECK(groups==0 && m.phase==WAREHOUSE_INFERRED_MOVE);
 in.settled=true; Path_Tick(&m,10005,&in); CHECK(m.phase==WAREHOUSE_SELECT_BALL && queries==0);
 /* Duplicate or missing earlier digits must never be inferred. */
 for(unsigned bad=0;bad<2;bad++) {
  init(); m.point=6; m.warehouse_mode=WAREHOUSE_DIGIT_ORDER;
  m.warehouse_columns[0]=1; m.warehouse_columns[1]=bad;
  Path_Tick(&m,0,&in); CHECK(m.phase==WAREHOUSE_FIRST_OFFSET && moves==0);
  Path_Tick(&m,5,&in); CHECK(queries==1 && m.warehouse_columns[2]==0);
 }
 return 0;
}
int main(void) {
 CHECK(inferred_column()==0);
 CHECK(entry_back()==0);
 const unsigned orders[6][3]={{1,2,3},{1,3,2},{2,1,3},{2,3,1},{3,1,2},{3,2,1}};
 for(unsigned i=0;i<6;i++) CHECK(run(false,orders[i])==0);
 CHECK(offset_line_check()==0); CHECK(run(true,orders[0])==0); CHECK(moving_stop()==0); CHECK(prep_gate()==0); CHECK(first_column_creep()==0);
 puts("warehouse digits passed"); return 0;
}
