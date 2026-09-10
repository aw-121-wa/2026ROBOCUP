#include "chassis_odom.h"
#include <math.h>
ChassisOdomDelta ChassisOdom_Integrate(Geometry g, const float rpm[4], float yaw, float left_scale,
                                       float right_scale, float ux, float uy, float dt,
                                       float velocity[3])
{
    ChassisOdomDelta delta = {0};
    if (g.radius <= 0 || g.arm <= 0 || dt <= 0 || dt > 0.05f)
        return delta;
    Mecanum_Forward(g, rpm, velocity);
    float y = velocity[1] * (velocity[1] >= 0 ? left_scale : right_scale);
    delta.x = (cosf(yaw) * velocity[0] - sinf(yaw) * y) * dt;
    delta.y = (sinf(yaw) * velocity[0] + cosf(yaw) * y) * dt;
    delta.progress = (velocity[0] * ux + y * uy) * dt;
    return delta;
}
