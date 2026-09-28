#include "path_mission.h"
#include "path_chassis.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
static PathCommand last;
static unsigned searches, turns, actions;
static bool reject;
static bool send(void *ctx,const PathCommand *c) {
 (void)ctx;last=*c;
 assert(c->kind!=PC_LINE_SEARCH && c->kind!=PC_LINE_CALIBRATE);
 if(c->kind==PC_MAP_SEARCH) { searches++; assert(c->y==-40); }
 if(c->kind==PC_MAP_AXIS || c->kind==PC_HOME_ALIGN) turns++;
 if(c->kind==PC_STAIR || c->kind==PC_GROUP) actions++;
 return !reject;
}
static void init(PathMission *m,PathInput *in,unsigned step) {
 Path_Init(m,send,0);m->result=PATH_RUNNING;m->step=step;m->phase=step==12?1:4;
 *in=(PathInput){.armed=true,.settled=true,.gray=15,.map_yaw_deg=step==9?180:0};
 searches=turns=actions=0;reject=false;
}
int main(void) {
 PathMission m;PathInput in; unsigned patterns[]={6,15,9};
 for(unsigned step=9;step<=13;step++) {
  if(step==10 || step==11)continue;
  for(unsigned i=0;i<3;i++) {
   init(&m,&in,step);in.gray=patterns[i];
   assert(!PathLine_AlignFour(&m,0,&in));
   assert(!PathLine_AlignFour(&m,5,&in));
   assert(PathLine_AlignFour(&m,105,&in));assert(!turns && !searches);
  }
  init(&m,&in,step);in.map_yaw_deg-=7;
  assert(!PathLine_AlignFour(&m,0,&in));
  assert(!PathLine_AlignFour(&m,5,&in));assert(turns==1 && !actions);
  assert(last.kind==(step==9?PC_MAP_AXIS:PC_HOME_ALIGN));
  in.settled=false;assert(!PathLine_AlignFour(&m,100,&in));assert(turns==1);
  /* Crossing the gray timeout while rotating must not cancel the rotation. */
  assert(!PathLine_AlignFour(&m,2100,&in));assert(turns==1);
  in.settled=true;in.map_yaw_deg=step==9?-180:0;
  assert(!PathLine_AlignFour(&m,2110,&in));
  assert(PathLine_AlignFour(&m,2120,&in));assert(m.line_skipped);
  in.map_yaw_deg+=3;assert(!PathLine_AlignFour(&m,2130,&in));assert(turns==2);
 }
 init(&m,&in,9);in.gray=13;
 assert(!PathLine_AlignFour(&m,0,&in));assert(searches==1 && !turns);
 in.map_yaw_deg=177;assert(!PathLine_AlignFour(&m,2000,&in));
 assert(!PathLine_AlignFour(&m,2005,&in));assert(turns==1);
 in.settled=false;assert(!PathLine_AlignFour(&m,2010,&in));
 in.settled=true;in.map_yaw_deg=180;assert(PathLine_AlignFour(&m,2020,&in));
 init(&m,&in,9);in.map_yaw_deg=175;PathLine_AlignFour(&m,0,&in);PathLine_AlignFour(&m,5,&in);
 in.settled=false;PathLine_AlignFour(&m,30005,&in);assert(m.result==PATH_TIMEOUT);
 init(&m,&in,9);in.map_yaw_deg=NAN;PathLine_AlignFour(&m,0,&in);assert(m.result==PATH_ERROR);
 init(&m,&in,9);reject=true;PathLine_AlignFour(&m,0,&in);assert(m.result==PATH_ERROR);
 /* A running arm task is never interrupted by a heading command. */
 init(&m,&in,9);m.phase=2;m.waiting=true;in.reply=PATH_WAIT;in.map_yaw_deg=174;
 Path_Tick(&m,5,&in);assert(!turns);
 in.reply=PATH_NONE;Path_Tick(&m,10,&in);assert(m.phase==3 && !turns);
 Path_Tick(&m,15,&in);assert(turns==1);
 init(&m,&in,13);m.phase=3;m.waiting=true;in.map_yaw_deg=5;in.reply=PATH_WAIT;
 assert(BallInventory_Record(&m.inventory,1,0x11)==BALL_ADDED);
 Path_Tick(&m,5,&in);assert(!turns);
 in.reply=PATH_OK;Path_Tick(&m,10,&in);assert(m.phase==7 && !turns);
 Path_Tick(&m,15,&in);assert(turns==1 && m.point==0);
 init(&m,&in,13);m.phase=3;in.gray=7;
 assert(BallInventory_Record(&m.inventory,1,0x11)==BALL_ADDED);
 Path_Tick(&m,0,&in);assert(m.phase==4 && actions==0);
 init(&m,&in,13);m.phase=3;m.line_skipped=true;in.gray=0;
 assert(BallInventory_Record(&m.inventory,1,0x11)==BALL_ADDED);
 Path_Tick(&m,0,&in);assert(m.waiting && actions==1);
 init(&m,&in,7);m.phase=0;in.map_yaw_deg=179;
 Path_Tick(&m,0,&in);assert(m.step==8 && turns==0);
 Path_Tick(&m,5,&in);assert(last.kind==PC_MOVE && last.x==-350 && turns==0);
 in.settled=false;Path_Tick(&m,10,&in);assert(m.phase==0 && turns==0);
 in.settled=true;Path_Tick(&m,15,&in);assert(m.phase==1);
 Path_Tick(&m,20,&in);assert(last.kind==PC_MAP_AXIS && turns==1 && actions==0);
 in.settled=false;Path_Tick(&m,16000,&in);assert(m.result==PATH_RUNNING && m.step==8);
 in.settled=true;in.map_yaw_deg=179.95f;Path_Tick(&m,16005,&in);
 assert(last.kind==PC_GROUP && actions==1);
 in.reply=PATH_OK;Path_Tick(&m,16010,&in);assert(m.step==9);
 puts("map heading and arm boundary checks passed");return 0;
}
