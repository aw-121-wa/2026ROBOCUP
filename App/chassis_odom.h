#ifndef CHASSIS_ODOM_H
#define CHASSIS_ODOM_H
#include "motion.h"
typedef struct
{
    float x, y, progress;
} ChassisOdomDelta;
ChassisOdomDelta ChassisOdom_Integrate(Geometry geometry, const float rpm[4], float yaw,
                                       float left_scale, float right_scale, float ux, float uy,
                                       float dt, float velocity[3]);
#endif
