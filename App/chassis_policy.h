#ifndef CHASSIS_POLICY_H
#define CHASSIS_POLICY_H
#include <stdbool.h>
/* Explicit mission input. Diagnostics must never be used as control commands. */
typedef struct {
    float stair_target_deg;
    float home_x_extra_trim_mm;
    float travel_speed_scale;
    bool mirror_map_y;
    bool use_start_turn_kp;
    bool stationary_hold;
    bool hold_during_action;
    bool suppress_lateral_comp;
} ChassisRoutePolicy;
#endif
