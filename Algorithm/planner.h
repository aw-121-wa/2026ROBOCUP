#ifndef PLANNER_H
#define PLANNER_H
#include <stdbool.h>
typedef struct
{
    float distance, peak, ta, tc, td, time;
    bool active;
    float deceleration, brake_distance, brake_speed;
    bool braking;
} Planner;
bool Planner_Start(Planner *p, float distance, float vmax, float acceleration, float deceleration);
float Planner_Update(Planner *p, float dt); /* Legacy time-profile test interface. */
float Planner_UpdateProgress(Planner *p, float dt, float progress_mm);
#endif
