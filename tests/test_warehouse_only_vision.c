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
}
