#include "path_warehouse.h"
#include <assert.h>
#include <stdio.h>
#include <math.h>
static unsigned queries, moves;
static bool send(void *ctx,const PathCommand *c) {
 (void)ctx;
 assert(c->kind!=PC_GROUP && c->kind!=PC_TURN && c->kind!=PC_DISC && c->kind!=PC_STAIR);
 if(c->kind==PC_WAREHOUSE_DIGIT) queries++;
 if(c->kind==PC_MOVE || c->kind==PC_FINISH_FORWARD) {assert(c->x>0 || (c->x==0 && c->y==-10)); if(c->x>0) moves++;}
 if(c->kind==PC_MAP_LATERAL) assert(c->y==25 || c->y==-20);
 return true;
}
int main(void) {
 const unsigned orders[6][3]={{1,2,3},{1,3,2},{2,1,3},{2,3,1},{3,1,2},{3,2,1}};
 for(unsigned mode=0;mode<7;mode++) {
  PathMission m; Path_Init(&m,send,0); m.blue=true; m.result=PATH_RUNNING; m.step=13;
  queries=moves=0; unsigned answered=0;
  PathInput in={.map_yaw_deg=180,.armed=true,.settled=true,.gray=6,.reply=PATH_OK,.warehouse_vision=true};
  for(unsigned t=0;t<25000 && m.result==PATH_RUNNING;t+=5) {
   if(queries>answered) {in.warehouse_digit_reply=mode==6?PATH_NONE:PATH_OK; in.warehouse_digit=orders[mode%6][answered++];}
   if(m.point==9) in.gray=m.phase==5?0:6;
  Path_Tick(&m,t,&in);
  }
  assert(m.result==PATH_DONE && m.point==9);
  assert(queries==(mode==6?1U:2U));
  assert(moves==(mode==6?2U:3U));
  if(mode<6) for(unsigned i=0;i<3;i++) assert(m.warehouse_columns[i]==orders[mode][i]);
 }
 PathMission m;Path_Init(&m,send,0);m.blue=true;m.result=PATH_RUNNING;m.step=13;m.point=9;
 m.phase=WAREHOUSE_HOME_FINAL_ALIGN;
 PathInput in={.armed=true,.settled=true,.map_yaw_deg=179};
 Path_Tick(&m,10,&in);assert(m.result==PATH_RUNNING && m.waiting);
 in.settled=false;in.map_yaw_deg=180;Path_Tick(&m,20,&in);assert(m.result==PATH_RUNNING);
 in.settled=true;Path_Tick(&m,30,&in);assert(m.result==PATH_DONE);
 puts("blue warehouse digits and fallback passed");
 return 0;
}
