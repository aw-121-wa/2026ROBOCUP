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
        .home_x_extra_trim_mm=70.0f,
        .travel_speed_scale=(running && step!=6) ? (step==9 ? 1.0f : 1.3f) : 1.0f,
        .mirror_map_y=blue,
        .use_start_turn_kp=running && blue && step==0,
        .stationary_hold=running && step>=8 && step<=10,
        .hold_during_action=running && step==9 && phase==24,
        .suppress_lateral_comp=false};
}
/* Boundary speeds remain unchanged so entry into search/orbit stays continuous. */
static inline float PathPolicy_CommandBoost(unsigned step, PathCommandKind kind)
{
    if (step==6 || step==9 || step==12 || step==13) return 1.0f;
    switch (kind) {
    case PC_MOVE: case PC_MOVE_ROTATE: case PC_FINISH_FORWARD: case PC_ORBIT_EXIT:
        return step<=1 ? PATH_TRAVEL_BOOST*1.3f : PATH_TRAVEL_BOOST;
    case PC_ARC:
        return step==0 ? PATH_TRAVEL_BOOST*1.3f : 1.0f; /* Other arcs enter line search. */
    default: return 1.0f;
    }
}
static inline float PathPolicy_Target(const PathMission *m)
{
    if (m->step<=9) return STAIR_TARGET_DEG(m->blue);
    return m->step==13 && m->point>=9 ? 0.0f : PATH_WAREHOUSE_TARGET_DEG(m->blue);
}
#endif
