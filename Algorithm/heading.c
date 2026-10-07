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

#include "heading_tuning.h"


float HeadingControl_Update(const HeadingRequest *r, const HeadingControlConfig *c,
                            float dt, float *integral)
{
    if (!r || !c || !integral) return 0;
    if (r->mode == HEADING_OFF || !isfinite(r->error) || !isfinite(r->gyro) ||
        !isfinite(r->feedforward) || !isfinite(dt) || dt <= 0) {
        *integral = 0;
        return 0;
    }
    float kp=c->kp, ki=c->ki, damping=c->damping, limit=c->limit;
    float gyro=r->gyro, feedforward=0;
    switch (r->mode) {
    case HEADING_MANUAL:
        *integral=0;
        return r->feedforward;
    case HEADING_TRAVEL:
        kp *= HEADING_TRAVEL_KP_SCALE;
        damping *= HEADING_TRAVEL_DAMPING_SCALE;
        break;
    case HEADING_DYNAMIC:
        feedforward=r->feedforward;
        gyro-=feedforward;
        break;
    case HEADING_ROTATE:
        limit=r->limit;
        break;
    case HEADING_PRECISION:
        *integral=0;
        return fmaxf(-HEADING_FINE_LIMIT, fminf(HEADING_FINE_LIMIT,
                     HEADING_FINE_KP*r->error-HEADING_FINE_DAMPING*gyro));
    case HEADING_HOLD:
        *integral=0;
        return fmaxf(-HEADING_HOLD_LIMIT, fminf(HEADING_HOLD_LIMIT,
                     kp*r->error-damping*gyro));
    case HEADING_FIXED: break;
    default: *integral=0; return 0;
    }
    float result=feedforward+Heading_Update(r->error,gyro,dt,kp,ki,damping,limit,integral);
    return fmaxf(-limit, fminf(limit,result));
}
