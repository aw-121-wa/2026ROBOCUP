#include "path_mission.h"
#include "path_chassis.h"
#include <assert.h>
static PathCommand last;
static unsigned count;
static bool send(void *ctx,const PathCommand *c) { (void)ctx;if(c->kind==PC_ARC) assert(c->start_speed<=c->speed && c->end_speed<=c->speed);last=*c;++count;return true; }
int main(void) {
    PathMission m;Path_Init(&m,send,0);m.result=PATH_RUNNING;m.step=6;m.phase=3;
    PathInput in={.armed=true,.map_yaw_deg=173};
    PathChassis_Tick(&m,0,&in);
    assert(m.step==8 && count==1 && last.kind==PC_ORBIT_EXIT && last.x==10 && last.angle==0);
    Path_Init(&m,send,0);m.blue=true;m.result=PATH_RUNNING;m.step=8;
    in.map_yaw_deg=173;in.motion_done=false;in.settled=true;
    unsigned before=count;
    PathChassis_Tick(&m,1000,&in);
    assert(count==before+1 && m.waiting && !m.heading_align_active);
    assert(last.kind==PC_ORBIT_EXIT && last.x==10 && last.angle==0);
    /* Residual yaw is corrected in motion; keep the original continuous arc. */
    in.motion_done=true;in.settled=false;
    PathChassis_Tick(&m,1100,&in);
    assert(last.kind==PC_ARC && last.start_speed==15 && last.end_speed==30 && last.y==0 && last.angle==90);
    assert(m.phase==2 && !m.heading_align_active);
    in.motion_done=false;in.settled=true;
    Path_Init(&m,send,0);m.blue=true;m.result=PATH_RUNNING;m.step=11;m.entered=2000;
    PathChassis_Tick(&m,2000,&in);assert(last.kind==PC_MOVE_ROTATE && last.angle==-180);
    assert(last.end_speed==40);
    in.motion_done=true;in.settled=false;in.gray=0;
    PathChassis_Tick(&m,2100,&in);
    assert(m.step==12 && last.kind==PC_BODY && last.x==0 && last.y==40);
    assert(!m.waiting);
    for (unsigned blue=0;blue<2;blue++) {
        Path_Init(&m,send,0);m.blue=blue;m.result=PATH_RUNNING;m.step=9;m.phase=22;
        in.settled=true;in.x_mm=in.y_mm=0;
        PathChassis_Tick(&m,3000,&in);
        assert(last.kind==PC_MOVE && last.x==(blue?280:160));
    }
    Path_Init(&m,send,0);m.blue=true;m.result=PATH_RUNNING;m.step=6;m.phase=3;
    in.motion_done=false;in.map_yaw_deg=280;
    PathChassis_Tick(&m,3050,&in);
    assert(m.step==8 && m.phase==4 && last.kind==PC_ORBIT_EXIT && last.x==0 && last.y==-20 && last.angle==280);
    in.motion_done=true;PathChassis_Tick(&m,3060,&in);
    assert(m.phase==5 && last.kind==PC_ORBIT_EXIT && last.angle==280);
    assert(last.x<-245 && last.y>43 && last.y<44);
    PathChassis_Tick(&m,3070,&in);
    assert(m.phase==0 && last.kind==PC_ORBIT_EXIT && last.x==10 && last.y==0 && last.angle==0);
    Path_Init(&m,send,0);m.blue=true;m.result=PATH_RUNNING;m.step=6;m.phase=1;
    in=(PathInput){.armed=true,.settled=true,.map_yaw_deg=92};
    PathChassis_Tick(&m,3080,&in);assert(m.phase==1 && last.kind==PC_HOME_ALIGN && last.x==90);
    in.settled=false;PathChassis_Tick(&m,3085,&in);assert(m.phase==1);
    in.settled=true;in.map_yaw_deg=90.1f;PathChassis_Tick(&m,3090,&in);
    assert(m.phase==2 && last.kind==PC_BODY);
    /* Blue second action switch moves earlier; final travel endpoint is unchanged. */
    Path_Init(&m,send,0);m.blue=true;m.result=PATH_RUNNING;m.step=9;m.phase=22;m.point=2;
    in.x_mm=in.y_mm=0;in.settled=true;
    PathChassis_Tick(&m,3100,&in);assert(last.kind==PC_MOVE && last.x==675);
    Path_Init(&m,send,0);m.blue=true;m.result=PATH_RUNNING;m.step=9;m.phase=22;m.point=3;
    PathChassis_Tick(&m,3200,&in);assert(last.kind==PC_MOVE && last.x==900);
    /* Blue must meet the same absolute heading gates even with inner probes lit. */
    for (unsigned step=8;step<=13;step++) {
        Path_Init(&m,send,0);m.blue=true;m.result=PATH_RUNNING;m.step=step;
        in.settled=true;in.gray=6;in.map_yaw_deg=step<=9?-0.11f:180.15f;
        if(step==9 || step==12 || step==13) { assert(PathHeading_Ready(&m,4000,&in)); continue; }
        assert(!PathHeading_Ready(&m,4000,&in));
        assert(last.kind==(step<=9?PC_MAP_AXIS:PC_HOME_ALIGN));
        in.map_yaw_deg=step<=9?-0.03f:180.05f;
        assert(PathHeading_Ready(&m,4100,&in));
    }
    return 0;
}
