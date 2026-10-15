#include "path_mission.h"
#include "path_chassis.h"
#include "path_config.h"
#include <assert.h>
#include <math.h>
static PathCommand last;
static bool send(void *ctx,const PathCommand *c) {(void)ctx;last=*c;return true;}
int main(void) {
 PathMission m; PathInput in={.armed=true,.settled=true};
 for(unsigned blue=0;blue<2;blue++) {
  Path_Init(&m,send,0);m.blue=blue;m.result=PATH_RUNNING;m.step=1;
  Path_Tick(&m,1,&in);
  assert(fabsf(last.x-(blue?-2323.9384f:2178.9384f))<0.02f);
 }
 Path_Init(&m,send,0);m.blue=true;m.result=PATH_RUNNING;m.step=5;
 PathChassis_Tick(&m,1,&in);assert(last.kind==PC_MOVE_ROTATE && !last.continuous && last.end_speed==0);
 in.motion_done=true;PathChassis_Tick(&m,2,&in);assert(m.step==8 && m.phase==6);
 in.map_yaw_deg=40;in.motion_done=false;
 PathChassis_Tick(&m,3,&in);assert(last.kind==PC_HOME_ALIGN && last.x==0 && m.step==8);
 in.map_yaw_deg=0;m.prep_pending=false;m.stair_ready_started=true;
 PathChassis_Tick(&m,40000,&in);assert(m.step==9 && m.phase==0);
 m.phase=30;m.waiting=true;m.entered=0;in.gray=2;
 PathChassis_Tick(&m,400000,&in);assert(m.result==PATH_RUNNING && last.kind==PC_BODY && last.x==-20);
 in.gray=0;PathChassis_Tick(&m,400001,&in);assert(m.phase==31);
 PathChassis_Tick(&m,400002,&in);assert(m.phase==20);
 Path_Init(&m,send,0);m.result=PATH_RUNNING;m.step=6;m.phase=6;m.orbit_yaw=0;
 in.resume_index=1;in.yaw_deg=0;
 PathChassis_Tick(&m,1,&in);assert(last.kind==PC_BODY && fabsf(last.speed+58.653f/1.1f)<.001f);
 assert(!PATH_BLUE_STAIR_UNTIMED(&m));
 return 0;
}
