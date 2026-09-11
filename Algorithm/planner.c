#include "planner.h"
#include <math.h>
#define PI 3.14159265358979323846f
bool Planner_Start(Planner *p, float d, float v, float a, float b)
{
    *p = (Planner){0};
    if (!isfinite(d) || !isfinite(v) || !isfinite(a) || !isfinite(b) || d <= 0 || v <= 0 ||
        a <= 0 || b <= 0)
        return false;
    p->distance = d;
    p->deceleration = b;
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
        return 0;
    }
    p->time += dt;
    float speed = p->time < p->ta ? 0.5f * p->peak * (1 - cosf(PI * p->time / p->ta)) : p->peak;
    float approach = fmaxf(0, fmaxf(applied_speed, committed_speed));
    float delay_distance = approach * fmaxf(dt, response_delay);
    float stopping = PI * approach * approach / (4 * p->deceleration);
    if (!p->braking && approach > 0 && remaining <= stopping + delay_distance)
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
        for (int i = 0; i < 20; ++i)
        {
            float u = 0.5f * (lo + hi);
            /* Integral of half-cosine deceleration normalized to its distance. */
            if (u + sinf(PI * u) / PI < fraction)
                lo = u;
            else
                hi = u;
        }
        speed = 0.5f * p->brake_speed * (1 + cosf(PI * (lo + hi) * 0.5f));
        /* Do not regain lost translation when rotational saturation disappears.
         * The existing terminal crawl remains a deliberate low-speed exception. */
        float crawl = fminf(5.0f, p->peak);
        float applied_cap = fmaxf(crawl, fmaxf(0, applied_speed));
        p->brake_output = fminf(p->brake_output, fminf(applied_cap, fmaxf(crawl, speed)));
        return p->brake_output;
    }
    return fmaxf(fminf(5.0f, p->peak), speed);
}
