#include "stair_heading.h"
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
 assert(c->kind!=PC_LINE_CALIBRATE);
 if(c->kind==PC_MAP_SEARCH) { searches++; assert(c->y==-40 || c->y==40); }
 if(c->kind==PC_MAP_AXIS || c->kind==PC_HOME_ALIGN) turns++;
 if(c->kind==PC_STAIR || c->kind==PC_GROUP) actions++;
 return !reject;
}
static void init(PathMission *m,PathInput *in,unsigned step) {
 Path_Init(m,send,0);m->result=PATH_RUNNING;m->step=step;m->phase=step==12?1:4;
 *in=(PathInput){.armed=true,.settled=true,.gray=6,.map_yaw_deg=step==9?STAIR_MAP_TARGET_DEG:0};
 searches=turns=actions=0;reject=false;
}
int main(void) {
 PathMission m;PathInput in; unsigned patterns[]={6,15,9};
 for (unsigned step=8;step<=13;step++) {
  init(&m,&in,step);
  for(unsigned gray=0;gray<16;gray++)
   assert(PathLine_Aligned(&m,gray)==(step==13 ? (gray&6)==6 : (gray==6 || gray==15 || gray==9)));
 }
 /* Stair admission is strict: a 0.15-degree error must request alignment. */
 init(&m,&in,9);in.map_yaw_deg=STAIR_MAP_TARGET_DEG-0.15f;
 assert(!PathHeading_Ready(&m,0,&in));assert(turns==1 && last.kind==PC_MAP_AXIS);
 in.settled=false;assert(!PathHeading_Ready(&m,5,&in));
 in.settled=true;in.map_yaw_deg=STAIR_MAP_TARGET_DEG-0.02f;
 assert(PathHeading_Ready(&m,10,&in));
 init(&m,&in,9);in.map_yaw_deg=STAIR_MAP_TARGET_DEG-0.02f;
 assert(PathHeading_Ready(&m,0,&in) && turns==0);
 init(&m,&in,9);in.map_yaw_deg=STAIR_MAP_TARGET_DEG-0.2f;
 assert(!PathHeading_Ready(&m,0,&in) && turns==1);
 /* Stair 0.07-degree gate accepts 0.06 but rejects 0.09. */
 init(&m,&in,9);in.map_yaw_deg=STAIR_MAP_TARGET_DEG-0.06f;
 assert(PathHeading_Ready(&m,0,&in) && turns==0);
 init(&m,&in,9);in.map_yaw_deg=STAIR_MAP_TARGET_DEG-0.09f;
 assert(!PathHeading_Ready(&m,0,&in) && turns==1);
 /* Warehouse rejects the same 0.15-degree residual as stairs. */
 init(&m,&in,13);in.map_yaw_deg=0.15f;
 assert(!PathHeading_Ready(&m,0,&in) && turns==1 && last.kind==PC_HOME_ALIGN);
 in.map_yaw_deg=0.05f; assert(PathHeading_Ready(&m,5,&in));

 for(unsigned step=9;step<=13;step++) {
  if(step==10 || step==11)continue;
  for(unsigned i=0;i<(step==9?3U:1U);i++) {
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
  in.settled=true;in.map_yaw_deg=step==9?STAIR_MAP_TARGET_DEG-360:0;
  assert(!PathLine_AlignFour(&m,2110,&in));
  if(step==9 || step==13) { assert(PathLine_AlignFour(&m,2210,&in));assert(!m.line_skipped); }
  else { assert(PathLine_AlignFour(&m,2120,&in));assert(m.line_skipped); }
  in.map_yaw_deg+=3;assert(!PathLine_AlignFour(&m,2220,&in));
  if(step==9 || step==13) assert(!PathLine_AlignFour(&m,2225,&in));
  assert(turns==2);
 }
 /* Lost line searches both directions, stopping before each reversal. */
 init(&m,&in,9);in.gray=0;
 assert(!PathLine_AlignFour(&m,0,&in));assert(last.y==-40);
 assert(!PathLine_AlignFour(&m,2000,&in));assert(last.kind==PC_HOLD);
 in.settled=false;assert(!PathLine_AlignFour(&m,2005,&in));assert(last.kind==PC_HOLD);
 in.settled=true;assert(!PathLine_AlignFour(&m,2010,&in));assert(last.kind==PC_MAP_SEARCH && last.y==40);
 in.gray=6;assert(!PathLine_AlignFour(&m,2020,&in));
 assert(!PathLine_AlignFour(&m,2025,&in));assert(PathLine_AlignFour(&m,2125,&in));
 assert(!m.line_skipped && m.result==PATH_RUNNING);
 init(&m,&in,9);in.map_yaw_deg=175;PathLine_AlignFour(&m,0,&in);PathLine_AlignFour(&m,5,&in);
 in.settled=false;PathLine_AlignFour(&m,30005,&in);assert(m.result==PATH_TIMEOUT);
 init(&m,&in,9);in.map_yaw_deg=NAN;PathLine_AlignFour(&m,0,&in);assert(m.result==PATH_ERROR);
 init(&m,&in,9);reject=true;PathLine_AlignFour(&m,0,&in);assert(m.result==PATH_ERROR);
 /* A running arm task is never interrupted by a heading command. */
 init(&m,&in,9);m.phase=24;m.stair_scanning=true;in.reply=PATH_WAIT;in.map_yaw_deg=174;
 Path_Tick(&m,5,&in);assert(!turns && m.phase==24);
 in.resume_index=1;Path_Tick(&m,10,&in);assert(m.phase==29 && !turns && m.grabs==1);
 Path_Tick(&m,15,&in);assert(turns==1 && m.phase==29);
 in.map_yaw_deg=STAIR_MAP_TARGET_DEG-0.02f;Path_Tick(&m,20,&in);assert(m.phase==22);
 init(&m,&in,9);m.phase=23;m.stair_scanning=true;in.ball_index=1;in.map_yaw_deg=STAIR_MAP_TARGET_DEG-0.15f;
 Path_Tick(&m,0,&in);assert(m.phase==23 && turns==1 && last.kind==PC_MAP_AXIS);
 in.map_yaw_deg=STAIR_MAP_TARGET_DEG-0.02f;Path_Tick(&m,5,&in);assert(m.phase==24 && last.kind==PC_PILLAR_STOPPED);
 init(&m,&in,13);m.phase=3;m.waiting=true;in.map_yaw_deg=5;in.reply=PATH_WAIT;
 assert(BallInventory_Record(&m.inventory,1,0x11)==BALL_ADDED);
 Path_Tick(&m,5,&in);assert(!turns);
 in.reply=PATH_OK;Path_Tick(&m,10,&in);assert(m.phase==7 && !turns);
 Path_Tick(&m,15,&in);assert(turns==1 && m.point==0);
 init(&m,&in,13);m.phase=3;in.gray=5;
 assert(BallInventory_Record(&m.inventory,1,0x11)==BALL_ADDED);
 Path_Tick(&m,0,&in);assert(m.phase==4 && actions==0);
 init(&m,&in,13);m.phase=3;m.line_skipped=true;in.gray=0;
 assert(BallInventory_Record(&m.inventory,1,0x11)==BALL_ADDED);
 Path_Tick(&m,0,&in);assert(m.phase==4 && actions==0);
 init(&m,&in,7);m.phase=0;in.map_yaw_deg=STAIR_MAP_TARGET_DEG;
 Path_Tick(&m,0,&in);assert(m.step==8 && turns==0);
 Path_Tick(&m,5,&in);assert(last.kind==PC_MOVE && last.x==-300 && turns==0);
 in.settled=false;Path_Tick(&m,10,&in);assert(m.phase==0 && turns==0);
 in.motion_done=true;Path_Tick(&m,15,&in);assert(m.phase==2 && last.kind==PC_ARC);
 assert(last.x==50 && last.y==180 && last.angle==-90 && last.start_speed==80 && last.end_speed==80);
 in.motion_done=false;in.settled=false;in.gray=0;Path_Tick(&m,20,&in);assert(m.phase==2);
 in.motion_done=true;Path_Tick(&m,25,&in);assert(m.phase==3);
 Path_Tick(&m,30,&in);assert(m.step==9 && actions>=1 && turns==0);
 in.gray=6;Path_Tick(&m,35,&in);assert(last.kind==PC_HOLD);
 in.settled=true;Path_Tick(&m,40,&in);assert(m.phase==1);
 in.reply=PATH_OK;in.map_yaw_deg=179;Path_Tick(&m,45,&in);
 assert(last.kind==PC_MAP_AXIS && turns==1); /* Still align before an arm grant. */
 puts("map heading and arm boundary checks passed");return 0;
}
