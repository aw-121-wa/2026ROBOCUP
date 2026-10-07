#include "planner.h"
#include <math.h>
#define PI 3.14159265358979323846f
bool Planner_Start(Planner *p, float d, float v, float a, float b)
{
    return Planner_StartBoundary(p, d, v, a, b, 0, 0);
}
bool Planner_StartBoundary(Planner *p, float d, float v, float a, float b,
                           float start_speed, float end_speed)
{
    *p = (Planner){0};
    if (!isfinite(d) || !isfinite(v) || !isfinite(a) || !isfinite(b) ||
        !isfinite(start_speed) || !isfinite(end_speed) || d <= 0 || v <= 0 ||
        a <= 0 || b <= 0 || start_speed < 0 || end_speed < 0 ||
        start_speed > v || end_speed > v)
        return false;
    p->distance = d;
    p->deceleration = b;
    p->start_speed = start_speed;
    p->end_speed = end_speed;
    float reachable_sq =
        (4 * d / PI + start_speed * start_speed / a + end_speed * end_speed / b) /
        (1 / a + 1 / b);
    p->peak = fminf(v, sqrtf(fmaxf(reachable_sq, 0)));
    if (p->peak < start_speed || p->peak < end_speed)
        return false;
    p->ta = PI * (p->peak - start_speed) / (2 * a);
    p->td = PI * (p->peak - end_speed) / (2 * b);
    float accel_distance = 0.5f * (start_speed + p->peak) * p->ta;
    float decel_distance = 0.5f * (end_speed + p->peak) * p->td;
    p->tc = fmaxf(0, (d - accel_distance - decel_distance) / p->peak);
    p->active = true;
    return true;
}
float Planner_Update(Planner *p, float dt)
{
    if (!p->active || dt <= 0)
        return 0;
    p->time += dt;
    float t = p->time;
    if (t < p->ta && p->ta > 0)
        return p->start_speed +
               0.5f * (p->peak - p->start_speed) * (1 - cosf(PI * t / p->ta));
    t -= p->ta;
    if (t < p->tc)
        return p->peak;
    t -= p->tc;
    if (t < p->td && p->td > 0)
        return p->end_speed +
               0.5f * (p->peak - p->end_speed) * (1 + cosf(PI * t / p->td));
    p->active = false;
    return p->end_speed;
}

/* Braking phase is indexed by applied distance, not elapsed wall time.
 * 0.5 mm acceptance and 5 mm/s crawl prevent integer-RPM terminal deadlock. */
float Planner_UpdateProgress(Planner *p, float dt, float progress, float applied_speed,
                             float committed_speed, float response_delay)
{
    if (!p->active || !isfinite(dt) || dt <= 0 || !isfinite(progress) || !isfinite(applied_speed) ||
        !isfinite(committed_speed) || !isfinite(response_delay) || response_delay < 0)
        return 0;
    float remaining = p->distance - progress;
    if (remaining <= 0.5f)
    {
        p->active = false;
        return p->end_speed;
    }
    p->time += dt;
    float speed = p->time < p->ta && p->ta > 0
                      ? p->start_speed +
                            0.5f * (p->peak - p->start_speed) *
                                (1 - cosf(PI * p->time / p->ta))
                      : p->peak;
    float approach = fmaxf(0, fmaxf(applied_speed, committed_speed));
    float delay_distance = approach * fmaxf(dt, response_delay);
    float stopping = approach > p->end_speed
                         ? PI * (approach * approach - p->end_speed * p->end_speed) /
                               (4 * p->deceleration)
                         : 0;
    if (!p->braking && approach > p->end_speed && remaining <= stopping + delay_distance)
    {
        p->braking = true;
        p->brake_distance = fmaxf(0.5f, remaining - delay_distance);
        p->brake_speed = approach;
        p->brake_output = approach;
    }
    if (p->braking)
    {
        float fraction = fmaxf(0, fminf(1, 1 - remaining / p->brake_distance));
        float lo = 0, hi = 1;
        float dv = p->brake_speed - p->end_speed;
        float norm = p->end_speed + 0.5f * dv;
        for (int i = 0; i < 20; ++i)
        {
            float u = 0.5f * (lo + hi);
            float traveled =
                (p->end_speed * u + 0.5f * dv * (u + sinf(PI * u) / PI)) / norm;
            if (traveled < fraction)
                lo = u;
            else
                hi = u;
        }
        speed = p->end_speed +
                0.5f * dv * (1 + cosf(PI * (lo + hi) * 0.5f));
        /* Do not regain lost translation when rotational saturation disappears.
         * The existing terminal crawl remains a deliberate low-speed exception. */
        float crawl = fmaxf(p->end_speed, fminf(5.0f, p->peak));
        float applied_cap = fmaxf(crawl, fmaxf(0, applied_speed));
        p->brake_output = fminf(p->brake_output, fminf(applied_cap, fmaxf(crawl, speed)));
        return p->brake_output;
    }
    return fmaxf(fminf(5.0f, p->peak), speed);
}
