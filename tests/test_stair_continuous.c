#include "path_mission.h"
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
 PathInput in={.armed=true,.settled=true,.gray=6,.map_yaw_deg=180};
 scans=g4=g3=stopped=headings=0; unsigned got=0; bool event=false;
 for(unsigned t=0;t<100000 && m.step<11;t+=10) {
  in.reply=PATH_OK; in.vision_ready=true;
  last=(PathCommand){.kind=PC_HOLD};
  Path_Tick(&m,t,&in);
  assert(m.result==PATH_RUNNING);
  if(last.kind==PC_STAIR_SCAN) { in.ball_index=in.resume_index=0; }
  if(last.kind==PC_MOVE) {
   assert(last.speed==20 && last.x>0);
   if(got<balls) { in.x_mm+=40;in.settled=false;in.ball_index++;event=true; }
   else { in.x_mm+=last.x;in.settled=true; }
  }
  if(event && last.kind==PC_HOLD) { in.x_mm+=2;in.settled=true; }
  if(last.kind==PC_PILLAR_STOPPED) { got++;in.resume_index=in.ball_index;event=false; }
 }
 assert(m.step==11 && fabsf(in.x_mm-860)<.1f);
 assert(g4==1 && g3==1 && stopped==balls && m.grabs==balls);
 assert(scans==(balls==2?1:4));
 assert(headings==4); /* Entry plus all three intermediate boundaries, even when full. */
}
static bool tx(void*p,const char*s,size_t n){(void)p;(void)s;(void)n;return true;}
static void feed(RdkLink*r,const char*s){while(*s)Rdk_Feed(r,*s++);}
int main(void) {
 run(0);run(1);run(2);
 /* Boundary detection cannot authorize the previous level's action. */
 PathMission m;Path_Init(&m,send,0);m.result=PATH_RUNNING;m.step=9;m.phase=22;
 m.stair_scanning=true;m.waiting=true;
 PathInput in={.armed=true,.settled=true,.x_mm=120,.ball_index=1};
 stopped=0;Path_Tick(&m,10,&in);assert(m.phase==25 && stopped==0);
 Path_Tick(&m,20,&in);assert(last.kind==PC_PILLAR_END && stopped==0);
 /* A ball just before the boundary may brake beyond it: still no grant. */
 m.phase=22;m.waiting=true;in.settled=false;in.x_mm=119;m.stair_distance=119;
 Path_Tick(&m,30,&in);assert(m.phase==23);
 in.settled=true;in.x_mm=120;Path_Tick(&m,40,&in);assert(m.phase==25 && !stopped);
 /* Projection follows the starting direction, not absolute global X. */
 Path_Init(&m,send,0);m.result=PATH_RUNNING;m.step=9;m.phase=22;
 m.stair_axis=3.14159265359f;m.waiting=true;
 in=(PathInput){.armed=true,.settled=true,.x_mm=-120};
 Path_Tick(&m,1,&in);assert(m.phase==27 && fabsf(m.stair_distance-120)<.01f);
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
