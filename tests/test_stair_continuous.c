#include "stair_heading.h"
#include "path_config.h"
#include "path_mission.h"
#include "path_chassis.h"
#include "rdk_link.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
static PathCommand last;
static unsigned scans, g4, g3, stopped, headings;
static bool send(void *p,const PathCommand *c) {
 (void)p; last=*c;
 if(c->kind==PC_MAP_HEADING) headings++;
 if(c->kind==PC_STAIR_SCAN) scans++;
 if(c->kind==PC_GROUP && c->argument==4) g4++;
 if(c->kind==PC_GROUP && c->argument==3) g3++;
 if(c->kind==PC_PILLAR_STOPPED) stopped++;
 return true;
}
static void run(unsigned balls) {
 PathMission m; Path_Init(&m,send,0);m.result=PATH_RUNNING;m.step=9;
 PathInput in={.armed=true,.settled=true,.gray=6,.map_yaw_deg=STAIR_MAP_TARGET_DEG};
 scans=g4=g3=stopped=headings=0; unsigned got=0; bool event=false;
 for(unsigned t=0;t<100000 && m.step<11;t+=10) {
  in.reply=PATH_OK; in.vision_ready=true;
  if(m.phase==30) in.gray=4;
  else if(m.phase==31) in.gray=6;
  last=(PathCommand){.kind=PC_HOLD};
  Path_Tick(&m,t,&in);
  assert(m.result==PATH_RUNNING);
  if(last.kind==PC_STAIR_SCAN) { in.ball_index=in.resume_index=0; }
  if(last.kind==PC_MOVE) {
   if(last.x==5 && last.speed==20) { in.x_mm+=5; continue; }
   assert(last.speed==(m.grabs>=2 && !m.stair_scanning ? PATH_STAIR_FAST_SPEED_RPM : PATH_STAIR_SCAN_SPEED_RPM) && last.x>0);
   if(got<balls) { in.x_mm+=40;in.settled=false;in.ball_index++;event=true; }
   else { in.x_mm+=m.point==3 ? 890-in.x_mm : last.x;in.settled=true;
          if(m.point==3) in.gray=2; }
  }
  if(event && last.kind==PC_HOLD) { in.x_mm+=2;in.settled=true; }
  if(last.kind==PC_PILLAR_STOPPED) { got++;in.resume_index=in.ball_index;event=false; }
 }
 assert(m.step==11 && fabsf(in.x_mm-890)<.1f);
 assert(g4==(balls==2?0U:1U) && g3==1 && stopped==balls && m.grabs==balls);
 assert(scans==(balls==2?1:3));
 assert(headings==(balls==2?1U:3U)); /* Full inventory skips intermediate line alignment. */
}
static bool tx(void*p,const char*s,size_t n){(void)p;(void)s;(void)n;return true;}
static void feed(RdkLink*r,const char*s){while(*s)Rdk_Feed(r,*s++);}
static void entry_braking_keeps_detection(void) {
 for (unsigned blue=0;blue<2;blue++) {
  PathMission m;Path_Init(&m,send,0);m.blue=blue;m.result=PATH_RUNNING;m.step=9;
  m.stair_ready_started=true;
  PathInput in={.armed=true,.gray=6,.map_yaw_deg=STAIR_TARGET_DEG(blue)-0.2f};
  PathChassis_Tick(&m,0,&in);assert(last.kind==PC_HOLD && m.line_entry_detected);
  in.gray=7;PathChassis_Tick(&m,10,&in);assert(m.phase==0);
  in.settled=true;PathChassis_Tick(&m,20,&in);
  if(blue) assert(m.phase==1);
  else {
   assert(last.kind==PC_MAP_SEARCH && m.phase==0);
   in.gray=6;PathChassis_Tick(&m,30,&in);assert(last.kind==PC_HOLD);
   PathChassis_Tick(&m,40,&in);PathChassis_Tick(&m,140,&in);assert(m.phase==1);
  }
  PathChassis_Tick(&m,150,&in);assert(m.phase==4);
  PathChassis_Tick(&m,160,&in);assert(m.phase==30 && m.stair_heading_calibrated);
  {
   in.gray=7;PathChassis_Tick(&m,170,&in);assert(last.kind==PC_BODY && last.x==-20 && m.phase==30);
   in.gray=4;in.settled=false;PathChassis_Tick(&m,171,&in);assert(last.kind==PC_HOLD && m.phase==31);
   PathChassis_Tick(&m,172,&in);assert(m.phase==31);
   in.settled=true;in.x_mm=-30;PathChassis_Tick(&m,175,&in);
  }
  assert(m.phase==20 && m.stair_origin_x==in.x_mm && m.stair_distance==0);
  in.gray=7;
  /* Later boundaries still perform lateral reacquisition. */
  m.phase=28;m.waiting=false;m.line_active=false;
  PathChassis_Tick(&m,180,&in);assert(last.kind==PC_MAP_SEARCH && m.phase==28);
 }
}
static void early_warehouse_prep(void) {
 for(unsigned blue=0;blue<2;blue++) for(unsigned point=0;point<4;point++) {
  PathMission m; Path_Init(&m,send,0); m.blue=blue; m.result=PATH_RUNNING;
  m.step=9; m.phase=24; m.point=point; m.stair_scanning=true;
  m.grabs=1; m.stair_base_grabs=0;
  PathInput in={.armed=true,.settled=true,.gray=6,.map_yaw_deg=STAIR_TARGET_DEG(blue),
      .x_mm=50,.ball_index=2,.resume_index=1,.reply=PATH_WAIT};
  m.stair_heading_calibrated=true; m.stair_axis=0; g3=g4=0;
  Path_Tick(&m,1,&in); assert(m.phase==24 && g3==0); /* Detection alone is insufficient. */
  in.resume_index=2; Path_Tick(&m,2,&in); assert(m.grabs==2 && g3==0);
  Path_Tick(&m,3,&in); assert(m.phase==25);
  Path_Tick(&m,4,&in); assert(m.phase==26 && last.kind==PC_PILLAR_END && g3==0);
  Path_Tick(&m,5,&in); assert(m.phase==26 && g3==0); /* END must be acknowledged. */
  /* END may arrive while heading hold is applying a small correction.
   * Real ports reject G3 until settled; never treat that temporary state as failure. */
  in.reply=PATH_OK; in.settled=false; Path_Tick(&m,6,&in);
  assert(m.result==PATH_RUNNING && m.phase==26 && g3==0 && !m.warehouse_prep_started);
  assert(m.stair_scanning); /* Keep the completed scan state until the handoff is accepted. */
  in.settled=true; Path_Tick(&m,7,&in);
  assert(g3==1 && g4==0 && m.point==3 && m.phase==22 && m.prep_pending);
  assert(m.warehouse_prep_started && !m.stair_scanning);
  in.reply=PATH_WAIT; Path_Tick(&m,8,&in);
  assert(last.kind==PC_MOVE && last.x==(blue?850:840) && last.speed==PATH_STAIR_FAST_SPEED_RPM);
  assert(m.prep_pending); /* Travel does not wait for G3. */
  in.x_mm=blue?900:890;if(!blue) in.gray=2; Path_Tick(&m,9,&in); assert(m.phase==27);
  Path_Tick(&m,10,&in); Path_Tick(&m,11,&in);
  assert(m.step==11 && g3==1 && g4==0 && m.prep_pending);
  in.reply=PATH_FAILED; Path_Tick(&m,12,&in); assert(m.result==PATH_ERROR);
 }
}
int main(void) {
 early_warehouse_prep();
 entry_braking_keeps_detection();
 run(0);run(1);run(2);
 /* Boundary detection cannot authorize the previous level's action. */
 PathMission m;Path_Init(&m,send,0);m.result=PATH_RUNNING;m.step=9;m.phase=22;
 m.stair_scanning=true;m.waiting=true;
 PathInput in={.armed=true,.settled=true,.x_mm=260,.ball_index=1};
 stopped=0;Path_Tick(&m,10,&in);assert(m.phase==25 && stopped==0 && last.kind==PC_HOLD && last.argument==0);
 Path_Tick(&m,20,&in);assert(last.kind==PC_PILLAR_END && stopped==0);
 /* A ball just before the boundary may brake beyond it: still no grant. */
 m.phase=22;m.waiting=true;in.settled=false;in.x_mm=259;m.stair_distance=259;
 Path_Tick(&m,30,&in);assert(m.phase==23 && last.kind==PC_HOLD && last.argument==1);
 in.settled=true;in.x_mm=260;Path_Tick(&m,40,&in);assert(m.phase==25 && !stopped);
 /* Projection follows the starting direction, not absolute global X. */
 Path_Init(&m,send,0);m.result=PATH_RUNNING;m.step=9;m.phase=22;
 m.stair_axis=3.14159265359f;m.waiting=true;
 in=(PathInput){.armed=true,.settled=true,.x_mm=-260};
 Path_Tick(&m,1,&in);assert(m.phase==27 && fabsf(m.stair_distance-260)<.01f);
 /* No RFID resume: never restart movement. */
 m.phase=24;m.stair_scanning=true;in.x_mm=-100;m.stair_distance=100;
 Path_Tick(&m,1000,&in);assert(m.phase==24 && m.grabs==0);
 in.fault=true;Path_Tick(&m,1001,&in);assert(m.result==PATH_ERROR);
 RdkLink r;Rdk_Init(&r,0,tx,0);r.stage=2;
 assert(Rdk_Begin(&r,"STAIR_SCAN",1,0,180000));Rdk_Tick(&r,0);
 feed(&r,"PILLAR_ACK\r\nPILLAR_READY\r\nPILLAR_BALL 1\r\n");
 assert(Rdk_PillarEnd(&r)); /* boundary cancels ungranted detection */
 Rdk_Tick(&r,1);feed(&r,"PILLAR_DONE\r\n");assert(!r.locked && !r.active);
 assert(Rdk_Begin(&r,"STAIR_SCAN",2,2,180000));Rdk_Tick(&r,2);
 feed(&r,"PILLAR_ACK\r\nPILLAR_READY\r\nPILLAR_BALL 1\r\n");
 assert(Rdk_PillarStopped(&r,1));Rdk_Tick(&r,3);
 assert(!Rdk_PillarEnd(&r)); /* Already authorized action must complete RFID. */
 puts("continuous stair OK");
}
