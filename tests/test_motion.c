#include "motion.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "zdt_x42s.h"
static void near(float a, float b)
{
    assert(fabsf(a - b) < 0.02f);
}
int main(void)
{
    uint8_t frame[8], sync[4];
    const uint8_t wanted[8] = {2, 0xF6, 1, 0x0B, 0xB8, 0, 1, 0x6B};
    const uint8_t wanted_sync[4] = {0, 0xFF, 0x66, 0x6B};
    ZDT_BuildSpeed(frame, 2, -300, 0);
    assert(memcmp(frame, wanted, 8) == 0);
    ZDT_BuildSync(sync);
    assert(memcmp(sync, wanted_sync, 4) == 0);
    Geometry g = {50, 300};
    float r[4], v[3];
    Mecanum_Inverse(g, 314.159265f, 0, 0, r);
    for (int i = 0; i < 4; i++)
        near(r[i], 60);
    Mecanum_Inverse(g, 0, 314.159265f, 0, r);
    near(r[0], -60);
    near(r[1], 60);
    near(r[2], 60);
    near(r[3], -60);
    float turn[4] = {-60, 60, -60, 60};
    Mecanum_Forward(g, turn, v);
    near(v[0], 0);
    near(v[1], 0);
    near(v[2], 1.04719755f);
    float t[4] = {100, 100, 100, 100}, rot[4] = {-50, 50, -50, 50};
    near(Wheel_Limit(t, rot, 100, r), 0.5f);
    near(r[0], 0);
    near(r[1], 100);
    Planner p;
    assert(Planner_Start(&p, 1000, 200, 400, 400));
    assert(p.tc > 0);
    float distance = 0;
    while (p.active)
        distance += Planner_Update(&p, 0.0001f) * 0.0001f;
    assert(fabsf(distance - 1000) < 0.5f);
    assert(Planner_Start(&p, 10, 200, 400, 400));
    near(p.tc, 0);
    distance = 0;
    while (p.active)
    {
        distance += Planner_Update(&p, 0.0001f) * 0.0001f;
    }
    near(distance, 10);
    assert(!Planner_Start(&p, 10, 0, 400, 400));
    near(Angle_Wrap(6.2831853f), 0);
    puts("motion tests passed");
    return 0;
}
