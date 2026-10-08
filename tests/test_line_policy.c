#include <assert.h>
#include <string.h>
#include "path_chassis.h"
#include "path_policy.h"
static PathCommand last;
static bool send(void *ctx,const PathCommand *c) { (void)ctx; last=*c; return true; }
int main(void) {
    for (unsigned blue=0;blue<2;++blue) for (unsigned step=0;step<=13;++step) {
        ChassisRoutePolicy p=PathPolicy_Chassis(blue,PATH_RUNNING,step,0);
        assert(p.travel_speed_scale==(!blue && step!=6 ? (step==9 ? 1.0f : 1.3f) : 1.0f));
        assert(p.home_x_extra_trim_mm==(blue ? 0.0f : 70.0f));
    }
    PathMission m={.send=send,.step=9,.result=PATH_RUNNING};
    for(unsigned gray=0;gray<16;++gray)
        assert(PathLine_Aligned(&m,gray)==(gray==6));
    m.step=13;
    for(unsigned gray=0;gray<16;++gray) assert(PathLine_Aligned(&m,gray)==(gray==6));
    m.step=9;
    PathInput in={.settled=true,.map_yaw_deg=180};
    assert(!PathLine_AlignFour(&m,0,&in)); assert(last.kind==PC_MAP_SEARCH && last.y==-40);
    in.settled=false;
    assert(!PathLine_AlignFour(&m,2000,&in)); assert(last.kind==PC_HOLD);
    assert(!PathLine_AlignFour(&m,2005,&in)); assert(last.kind==PC_HOLD);
    in.settled=true;
    assert(!PathLine_AlignFour(&m,2010,&in)); assert(last.kind==PC_MAP_SEARCH && last.y==40);
    assert(!PathLine_AlignFour(&m,4010,&in)); assert(last.kind==PC_HOLD);
    assert(!PathLine_AlignFour(&m,4015,&in)); assert(last.kind==PC_MAP_SEARCH && last.y==-40);
    assert(!PathLine_AlignFour(&m,6015,&in)); assert(m.result==PATH_TIMEOUT);
    memset(&m,0,sizeof m); m.send=send; m.step=9; m.result=PATH_RUNNING;
    in=(PathInput){.settled=true,.map_yaw_deg=175,.gray=6};
    assert(!PathLine_AlignFour(&m,0,&in)); assert(!PathLine_AlignFour(&m,1,&in));
    assert(PathLine_AlignFour(&m,101,&in)); assert(m.stair_heading_calibrated);
    in.map_yaw_deg=178; last.kind=PC_HOLD;
    assert(PathHeading_Ready(&m,102,&in)); assert(last.kind==PC_HOLD);
    m.step=13; m.point=9;
    assert(!PathHeading_Ready(&m,103,&in)); assert(last.kind==PC_HOME_ALIGN && last.x==0);
    return 0;
}
