#include "heading.h"
#include <math.h>
#define TWO_PI 6.28318530717958647692f
void HeadingEstimator_Reset(HeadingEstimator *e)
{
    *e = (HeadingEstimator){0};
}
bool HeadingEstimator_Update(HeadingEstimator *e, float angle, uint32_t frame,
                             float gyro, float dt, bool usable)
{
    if (!usable || !isfinite(angle) || !isfinite(gyro) || !isfinite(dt) || dt < 0 || dt > 0.05f)
    {
        HeadingEstimator_Reset(e);
        return false;
    }
    if (!e->ready)
    {
        e->yaw_rad = remainderf(angle, TWO_PI);
        e->last_gyro_rad_s = gyro;
        e->angle_frame = frame;
        e->ready = true;
        return true;
    }
    if (dt > 0)
        e->yaw_rad = remainderf(e->yaw_rad + 0.5f * (e->last_gyro_rad_s + gyro) * dt, TWO_PI);
    e->last_gyro_rad_s = gyro;
    if (frame != e->angle_frame)
    {
        float correction = remainderf(angle - e->yaw_rad, TWO_PI);
        e->yaw_rad = remainderf(e->yaw_rad + correction, TWO_PI);
        e->angle_frame = frame;
    }
    return true;
}
float Heading_Update(float error, float gyro, float dt, float kp, float ki, float kg, float limit,
                     float *integral)
{
    if (fabsf(error) < 0.00045f)
        error = 0;
    *integral = fmaxf(-0.5f, fminf(0.5f, *integral + error * dt));
    return fmaxf(-limit, fminf(limit, kp * error + ki * *integral - kg * gyro));
}
