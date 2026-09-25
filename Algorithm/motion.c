#include "motion.h"
#include <math.h>
#define PI 3.14159265358979323846f
float Angle_Wrap(float a)
{
    return remainderf(a, 2 * PI);
}
void Motion_FixedDirection(float start_x, float start_y, float yaw_delta, float *x, float *y)
{
    *x = start_x * cosf(yaw_delta) + start_y * sinf(yaw_delta);
    *y = -start_x * sinf(yaw_delta) + start_y * cosf(yaw_delta);
}
void Motion_SmoothTurn(float progress, float length, float angle, float speed,
                       float *target, float *rate)
{
    float u = length > 0 ? fmaxf(0, fminf(1, progress / length)) : 1;
    /* Symmetry avoids cancellation near u=1 in single precision. */
    float t = fminf(u, 1 - u);
    float h = t * t * t * (10 + t * (-15 + 6 * t));
    float u2 = u * u;
    *target = angle * (u <= 0.5f ? h : 1 - h);
    *rate = length > 0 ? angle * 30 * u2 * (1 - u) * (1 - u) * speed / length : 0;
}
void Motion_ArcDirection(float start, float turn, float progress, float length, float *x, float *y)
{
    float u = length > 0 ? fmaxf(0, fminf(1, progress / length)) : 0;
    float angle = start + turn * u;
    *x = cosf(angle);
    *y = sinf(angle);
}
void Mecanum_Inverse(Geometry g, float x, float y, float w, float r[4])
{
    /* Reverse all chassis translations, preserving the yaw convention. */
    x = -x;
    y = -y;
    float k = 60 / (2 * PI * g.radius), a = g.arm * w;
    r[0] = (x - y - a) * k;
    r[1] = (x + y + a) * k;
    r[2] = (x + y - a) * k;
    r[3] = (x - y + a) * k;
}
void Mecanum_Forward(Geometry g, const float r[4], float v[3])
{
    float k = 2 * PI * g.radius / 240;
    v[0] = -(r[0] + r[1] + r[2] + r[3]) * k;
    v[1] = -(-r[0] + r[1] + r[2] - r[3]) * k;
    v[2] = (-r[0] + r[1] - r[2] + r[3]) * k / g.arm;
}
float Wheel_Limit(const float t[4], const float r[4], float limit, float out[4])
{
    float peak = 0, s = 1;
    for (int i = 0; i < 4; i++)
        peak = fmaxf(peak, fabsf(r[i]));
    float scale = peak > limit ? limit / peak : 1;
    for (int i = 0; i < 4; i++)
    {
        float a = r[i] * scale;
        if (t[i] > 0)
            s = fminf(s, (limit - a) / t[i]);
        if (t[i] < 0)
            s = fminf(s, (-limit - a) / t[i]);
    }
    s = fmaxf(0, s);
    for (int i = 0; i < 4; i++)
        out[i] = s * t[i] + scale * r[i];
    return s;
}

void Motion_SlewVelocity(float current[3], const float target[3], float dt,
                         float linear_accel, float angular_accel)
{
    if (!isfinite(dt) || dt <= 0 || linear_accel <= 0 || angular_accel <= 0) return;
    float dx = target[0] - current[0], dy = target[1] - current[1];
    float dw = target[2] - current[2];
    float linear = hypotf(dx, dy), angular = fabsf(dw), fraction = 1;
    if (linear > 0) fraction = fminf(fraction, linear_accel * dt / linear);
    if (angular > 0) fraction = fminf(fraction, angular_accel * dt / angular);
    for (unsigned i = 0; i < 3; ++i)
        current[i] = fraction >= 1 ? target[i] : current[i] + fraction * (target[i] - current[i]);
}
