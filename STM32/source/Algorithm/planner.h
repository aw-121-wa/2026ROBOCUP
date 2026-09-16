#ifndef PLANNER_H
#define PLANNER_H
#include <stdbool.h>
typedef struct
{
    float distance, peak, ta, tc, td, time;
    bool active;
    float deceleration, brake_distance, brake_speed;
    bool braking;
    float brake_output;
} Planner;
bool Planner_Start(Planner *p, float distance, float vmax, float acceleration, float deceleration);
float Planner_Update(Planner *p, float dt); /* Legacy time-profile test interface. */
/* Speeds are signed projections in the same units/frame as progress_mm.
 * committed_speed includes an immutable batch not yet applied.
 * response_delay includes scheduling and wire time before a brake command applies. */
float Planner_UpdateProgress(Planner *p, float dt, float progress_mm, float applied_speed,
                             float committed_speed, float response_delay);
#endif
