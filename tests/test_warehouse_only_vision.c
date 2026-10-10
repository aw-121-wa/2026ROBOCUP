#include "path_config.h"
#include "path_mission.h"
#include "path_chassis.h"
#include <assert.h>
static PathCommand last;
static unsigned groups;
static bool send(void *context,const PathCommand *command)
{
    (void)context;
    assert(command->kind!=PC_DISC &&
           command->kind!=PC_STAIR_SCAN && command->kind!=PC_PILLAR_END);
    if (command->kind==PC_GROUP) {assert(command->argument==3);++groups;}
    last=*command;
    return true;
}
int main(void)
{
    assert(PATH_RDK_ENABLE && PATH_VISION_ENABLE && !PATH_COLLECTION_VISION_ENABLE);
    for (unsigned blue=0;blue<2;++blue) {
        PathMission m;Path_Init(&m,send,0);m.blue=blue;
        PathInput in={.armed=true,.settled=true};
        assert(Path_Start(&m,0,&in) && m.phase==0 && !m.prep_pending);
        Path_Tick(&m,1,&in);assert(last.kind==PC_MOVE || last.kind==PC_MOVE_ROTATE);
        m.step=4;m.phase=0;m.waiting=false;
        PathChassis_Tick(&m,2,&in);assert(m.step==5 && !m.prep_pending);
        m.step=9;m.phase=20;m.point=0;m.waiting=false;
        PathChassis_Tick(&m,3,&in);assert(m.phase==22 && !m.stair_scanning);
        m.step=10;m.phase=0;m.waiting=false;
        PathChassis_Tick(&m,4,&in);assert(m.step==11 && last.kind==PC_GROUP && last.argument==3);
    }
    assert(groups==2);
    PathMission m;Path_Init(&m,send,0);m.result=PATH_RUNNING;m.step=9;m.phase=30;
    PathInput in={.armed=true,.settled=true,.gray=6};
    PathChassis_Tick(&m,10,&in);assert(last.kind==PC_BODY && last.x==-20);
    in.gray=4;in.settled=false;PathChassis_Tick(&m,11,&in);
    assert(last.kind==PC_HOLD && m.phase==31);
    in.settled=true;PathChassis_Tick(&m,12,&in);assert(m.phase==20);
    m.phase=22;m.point=3;m.waiting=false;in.gray=4;in.x_mm=650;
    PathChassis_Tick(&m,13,&in);assert(m.phase==22 && last.kind==PC_MOVE);
    in.gray=2;in.settled=false;PathChassis_Tick(&m,14,&in);
    assert(m.phase==22);
    in.settled=true;in.x_mm=880;
    PathChassis_Tick(&m,15,&in);assert(m.phase==27 && last.kind==PC_HOLD);
}
