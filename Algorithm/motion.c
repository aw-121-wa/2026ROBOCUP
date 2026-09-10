#include "motion.h"
#include <math.h>
#define PI 3.14159265358979323846f
float Angle_Wrap(float a)
{
    return remainderf(a, 2 * PI);
}
void Mecanum_Inverse(Geometry g, float x, float y, float w, float r[4])
{
    float k = 60 / (2 * PI * g.radius), a = g.arm * w;
    r[0] = (x - y - a) * k;
    r[1] = (x + y + a) * k;
    r[2] = (x + y - a) * k;
    r[3] = (x - y + a) * k;
}
void Mecanum_Forward(Geometry g, const float r[4], float v[3])
{
    float k = 2 * PI * g.radius / 240;
    v[0] = (r[0] + r[1] + r[2] + r[3]) * k;
    v[1] = (-r[0] + r[1] + r[2] - r[3]) * k;
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
bool Planner_Start(Planner *p, float d, float v, float a, float b)
{
    *p = (Planner){0};
    if (!isfinite(d) || !isfinite(v) || !isfinite(a) || !isfinite(b) || d <= 0 || v <= 0 ||
        a <= 0 || b <= 0)
        return false;
    p->distance = d;
    p->peak = fminf(v, sqrtf(4 * d / (PI * (1 / a + 1 / b))));
    p->ta = PI * p->peak / (2 * a);
    p->td = PI * p->peak / (2 * b);
    p->tc = fmaxf(0, (d - 0.5f * p->peak * (p->ta + p->td)) / p->peak);
    p->active = true;
    return true;
}
float Planner_Update(Planner *p, float dt)
{
    if (!p->active || dt <= 0)
        return 0;
    p->time += dt;
    float t = p->time;
    if (t < p->ta)
        return 0.5f * p->peak * (1 - cosf(PI * t / p->ta));
    t -= p->ta;
    if (t < p->tc)
        return p->peak;
    t -= p->tc;
    if (t < p->td)
        return 0.5f * p->peak * (1 + cosf(PI * t / p->td));
    p->active = false;
    return 0;
}
