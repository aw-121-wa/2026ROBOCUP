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
/* Common interpolation fraction preserves body velocity ratios through a ramp. */
void Motion_SlewVelocity(float current[3], const float target[3], float dt,
                         float linear_accel, float angular_accel);
float Angle_Wrap(float radians);
void Motion_FixedDirection(float start_x, float start_y, float yaw_delta, float *x, float *y);
/* Distance-indexed quintic yaw: angle in radians, speed/length in matching units. */
void Motion_SmoothTurn(float progress, float length, float angle, float speed,
                       float *target, float *rate);
void Motion_ArcDirection(float start_rad, float turn_rad, float progress_mm, float length_mm,
                         float *x, float *y);
#endif
