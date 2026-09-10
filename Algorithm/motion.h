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
#endif
