#ifndef PATH_POLICY_H
#define PATH_POLICY_H
#include "chassis_policy.h"
#include "path_mission.h"
#include "stair_heading.h"
#include "path_config.h"
static inline ChassisRoutePolicy PathPolicy_Chassis(bool blue, PathResult result,
                                                   unsigned step, unsigned phase)
{
    bool running=result==PATH_RUNNING;
    return (ChassisRoutePolicy){
        .stair_target_deg=STAIR_TARGET_DEG(blue),
        .stationary_hold=running && step>=8 && step<=10,
        .hold_during_action=running && step==9 && phase==24,
        .suppress_lateral_comp=running && blue && step>=8 && step<=13};
}
static inline float PathPolicy_Target(const PathMission *m)
{
    if (m->step<=9) return STAIR_TARGET_DEG(m->blue);
    return m->step==13 && m->point>=9 ? 0.0f : PATH_WAREHOUSE_TARGET_DEG(m->blue);
}
#endif
