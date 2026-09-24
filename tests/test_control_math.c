#include "heading.h"
#include "chassis_odom.h"
#include <assert.h>
#include <math.h>
int main(void)
{
    /* A fixed world-left path must become body-forward at 90 degrees,
     * then body-right at 180 degrees. Rotation must not bend that path. */
    for (unsigned n = 0; n <= 180; ++n)
    {
        float yaw = n * 0.017453292519943295f, x, y;
        Motion_FixedDirection(0, 1, yaw, &x, &y);
        assert(fabsf(x * cosf(yaw) - y * sinf(yaw)) < 1e-5f);
        assert(fabsf(x * sinf(yaw) + y * cosf(yaw) - 1) < 1e-5f);
    }
    float i = 0;
    assert(fabsf(Heading_Update(0, 1, 0.005f, 2, 0.1f, 0.2f, 1, &i) + 0.2f) < 0.001f);
    assert(Heading_Update(10, 0, 1, 2, 1, 0, 1, &i) == 1);
    assert(i == 0.5f);
    HeadingEstimator est = {0};
    assert(HeadingEstimator_Update(&est, 0, 1, 1, 0.005f, true));
    assert(HeadingEstimator_Update(&est, 0, 1, 1, 0.005f, true));
    assert(fabsf(est.yaw_rad - 0.005f) < 1e-5f); /* 200 Hz propagation between 20 Hz angles. */
    assert(HeadingEstimator_Update(&est, 0.05f, 2, 1, 0.005f, true));
    assert(fabsf(est.yaw_rad - 0.05f) < 1e-5f); /* New angle frame anchors drift. */
    assert(!HeadingEstimator_Update(&est, 0, 2, 0, 0.005f, false) && !est.ready);
    float arc_x, arc_y;
    Motion_ArcDirection(20.0f * 0.017453292519943295f,
                        -20.0f * 0.017453292519943295f, 0, 100, &arc_x, &arc_y);
    assert(fabsf(arc_x - cosf(20.0f * 0.017453292519943295f)) < 1e-6f);
    Motion_ArcDirection(20.0f * 0.017453292519943295f,
                        -20.0f * 0.017453292519943295f, 100, 100, &arc_x, &arc_y);
    assert(fabsf(arc_x - 1.0f) < 1e-6f && fabsf(arc_y) < 1e-6f);
    float rpm[4] = {60, 60, 60, 60}, velocity[3];
    ChassisOdomDelta d =
        ChassisOdom_Integrate((Geometry){35, 300}, rpm, 0, 1, 1, 1, 0, 0.01f, velocity);
    assert(fabsf(d.progress - 2.19911486f) < 0.001f);
    assert(fabsf(d.x - d.progress) < 0.001f);
    assert(fabsf(d.y) < 0.001f);
    float lateral[4] = {-60, 60, 60, -60};
    d = ChassisOdom_Integrate((Geometry){35, 300}, lateral, 0, 2, 1, 0, 1, 0.01f, velocity);
    assert(fabsf(d.progress - 4.39822972f) < 0.001f);
    assert(fabsf(ChassisOdom_PathSpeed((Geometry){35, 300}, lateral, 2, 1, 0, 1) - 439.822972f) <
           0.01f);
    assert(fabsf(ChassisOdom_PathSpeed((Geometry){35, 300}, lateral, 2, 1, 1, 0)) < 0.001f);
    return 0;
}
