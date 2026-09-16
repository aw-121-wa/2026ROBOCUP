#include "planner.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
int main(void)
{
    Planner p;
    assert(Planner_Start(&p, 1000, 200, 400, 400));
    float progress = 0;
    float applied = 0;
    for (int i = 0; i < 30000 && p.active; ++i)
    {
        float v = Planner_UpdateProgress(&p, 0.005f, progress, applied, applied, 0.01f);
        /* Fixed available translation: cruise is capped at 40% of vmax. */
        applied = fminf(v, 80);
        progress += applied * 0.005f;
    }
    assert(!p.active);
    assert(fabsf(progress - 1000) < 0.6f);
    assert(Planner_Start(&p, 100, 100, 200, 200));
    for (int i = 0; i < 4000; ++i)
        (void)Planner_UpdateProgress(&p, 0.005f, 0, 0, 0, 0.01f);
    assert(p.active); /* Time alone never completes a stalled segment. */
    assert(Planner_UpdateProgress(&p, 0.005f, 100, 0, 0, 0.01f) == 0);
    assert(!p.active);
    /* 700 mm/s must not brake at the theoretical 1000 mm/s threshold. */
    assert(Planner_Start(&p, 5000, 1000, 1000, 1000));
    (void)Planner_UpdateProgress(&p, 2, 0, 0, 0, 0.01f);
    (void)Planner_UpdateProgress(&p, 0.005f, 4500, 700, 700, 0.01f);
    assert(!p.braking); /* remaining 500 > 384.845 + 7 */
    float first = Planner_UpdateProgress(&p, 0.005f, 4610, 700, 700, 0.01f);
    assert(p.braking && fabsf(p.brake_speed - 700) < 0.001f);
    assert(first <= 700);
    float limited = Planner_UpdateProgress(&p, 0.005f, 4613, 500, 500, 0.01f);
    float released = Planner_UpdateProgress(&p, 0.005f, 4616, 700, 700, 0.01f);
    assert(limited <= 500 && released <= limited);
    /* An immutable faster batch reserves its larger stopping distance. */
    assert(Planner_Start(&p, 5000, 1000, 1000, 1000));
    (void)Planner_UpdateProgress(&p, 0.005f, 4500, 700, 1000, 0.01f);
    assert(p.braking && p.brake_speed == 1000);
    /* Additional wire delay moves the trigger earlier. */
    assert(Planner_Start(&p, 5000, 1000, 1000, 1000));
    (void)Planner_UpdateProgress(&p, 0.005f, 4600, 700, 700, 0.06f);
    assert(p.braking); /* 400 < 384.845 + 42 */
    /* Reverse projected velocity does not become approach speed by squaring. */
    assert(Planner_Start(&p, 5000, 1000, 1000, 1000));
    (void)Planner_UpdateProgress(&p, 0.005f, 4900, -700, -700, 0.01f);
    assert(!p.braking);
    puts("progress planner passed");
    assert(Planner_Start(&p, 5000, 1000, 1000, 1000));
    progress = 0;
    applied = 0;
    bool seen_braking = false;
    for (int step = 0; step < 10000 && p.active; ++step)
    {
        float previous = applied;
        float v = Planner_UpdateProgress(&p, 0.005f, progress, applied, applied, 0.01f);
        /* Free all wheel headroom at braking onset; no applied acceleration follows. */
        applied = p.braking ? v : fminf(v, 700);
        if (seen_braking)
            assert(applied <= previous + 0.001f);
        seen_braking = p.braking;
        progress += applied * 0.005f;
    }
    assert(seen_braking && !p.active && fabsf(progress - 5000) < 0.6f);
}
