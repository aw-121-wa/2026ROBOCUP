#ifndef MOTION_H
#define MOTION_H
#include <stdbool.h>
#include "planner.h"
typedef struct
{
    float radius, arm;
} Geometry;
void Mecanum_Inverse(Geometry g, float x, float y, float w, float rpm[4]);
void Mecanum_Forward(Geometry g, const float rpm[4], float velocity[3]);
float Wheel_Limit(const float translation[4], const float rotation[4], float limit, float rpm[4]);
float Angle_Wrap(float radians);
void Motion_FixedDirection(float start_x, float start_y, float yaw_delta, float *x, float *y);
void Motion_ArcDirection(float start_rad, float turn_rad, float progress_mm, float length_mm,
                         float *x, float *y);
#endif
