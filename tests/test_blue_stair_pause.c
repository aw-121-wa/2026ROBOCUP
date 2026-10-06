#include "path_mission.h"
#include "path_chassis.h"
#include <assert.h>
static PathCommand last;
static unsigned count;
static bool send(void *ctx,const PathCommand *c) { (void)ctx;last=*c;++count;return true; }
int main(void) {
    PathMission m;Path_Init(&m,send,0);m.result=PATH_RUNNING;m.step=6;m.phase=3;
    PathInput in={.armed=true,.map_yaw_deg=173};
    PathChassis_Tick(&m,0,&in);assert(count==0);
    in.settled=true;PathChassis_Tick(&m,100,&in);
    PathChassis_Tick(&m,299,&in);assert(count==0 && m.step==6);
    in.settled=false;PathChassis_Tick(&m,300,&in);assert(m.step==6);
    in.settled=true;PathChassis_Tick(&m,305,&in);assert(m.step==7 && count==0);
    PathChassis_Tick(&m,310,&in);PathChassis_Tick(&m,315,&in);
    assert(count==2); /* Heading target only, then travel; no stationary correction. */
    assert(last.kind==PC_MOVE && last.x==-300);
    return 0;
}
