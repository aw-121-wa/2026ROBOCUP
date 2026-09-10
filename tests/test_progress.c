#include "planner.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
int main(void)
{
    Planner p;
    assert(Planner_Start(&p, 1000, 200, 400, 400));
    float progress = 0;
    for (int i = 0; i < 30000 && p.active; ++i)
    {
        float v = Planner_UpdateProgress(&p, 0.005f, progress);
        /* Translation saturation: only 40% of command reaches the wheels. */
        progress += v * 0.4f * 0.005f;
    }
    assert(!p.active);
    assert(fabsf(progress - 1000) < 0.6f);
    assert(Planner_Start(&p, 100, 100, 200, 200));
    for (int i = 0; i < 4000; ++i)
        (void)Planner_UpdateProgress(&p, 0.005f, 0);
    assert(p.active); /* Time alone never completes a stalled segment. */
    assert(Planner_UpdateProgress(&p, 0.005f, 100) == 0);
    assert(!p.active);
    puts("progress planner passed");
}
