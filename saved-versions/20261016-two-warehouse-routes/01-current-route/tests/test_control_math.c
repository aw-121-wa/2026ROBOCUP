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
    /* Smooth heading has zero endpoint rate/acceleration, integrates to 180 deg,
     * and its feedforward matches the derivative of its distance-indexed target. */
    const float pi=3.14159265358979323846f;
    float turn, rate, last_turn=0, integrated=0;
    Motion_SmoothTurn(0, 1600, pi, 300, &turn, &rate);
    assert(turn==0 && rate==0);
    for (unsigned n=1; n<=16000; ++n) {
        Motion_SmoothTurn(n*0.1f, 1600, pi, 300, &turn, &rate);
        assert(turn>=last_turn-1e-6f && rate>=0);
        assert(fabsf((turn-last_turn)/(.1f/300)-rate)<0.007f);
        integrated+=rate*(.1f/300); last_turn=turn;
    }
    assert(fabsf(turn-pi)<1e-6f && rate==0);
    assert(fabsf(integrated-pi)<0.001f);
    Motion_SmoothTurn(0.16f, 1600, pi, 300, &turn, &rate);
    assert(fabsf(rate)<1e-6f);
    Motion_SmoothTurn(1599.84f, 1600, pi, 300, &turn, &rate);
    assert(fabsf(rate)<1e-6f);
    Motion_SmoothTurn(1700, 1600, -pi, 300, &turn, &rate);
    assert(fabsf(turn+pi)<1e-6f && rate==0);
    Motion_SmoothTurn(800, 1600, -pi, 300, &turn, &rate);
    assert(fabsf(turn+pi/2)<1e-6f && rate<0);
    /* At a 175 deg measured yaw, the final-heading frame still puts the
     * -160 deg arc tangent at +20 deg in the original field frame. */
    float body_x,body_y;
    Motion_FixedDirection(cosf(-160*pi/180),sinf(-160*pi/180),-5*pi/180,&body_x,&body_y);
    assert(fabsf(body_x*cosf(175*pi/180)-body_y*sinf(175*pi/180)-cosf(20*pi/180))<1e-5f);
    assert(fabsf(body_x*sinf(175*pi/180)+body_y*cosf(175*pi/180)-sinf(20*pi/180))<1e-5f);
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
    float rpm[4] = {-60, -60, -60, -60}, velocity[3];
    ChassisOdomDelta d =
        ChassisOdom_Integrate((Geometry){35, 300}, rpm, 0, 1, 1, 1, 0, 0.01f, velocity);
    assert(fabsf(d.progress - 2.19911486f) < 0.001f);
    assert(fabsf(d.x - d.progress) < 0.001f);
    assert(fabsf(d.y) < 0.001f);
    float lateral[4] = {60, -60, -60, 60};
    d = ChassisOdom_Integrate((Geometry){35, 300}, lateral, 0, 2, 1, 0, 1, 0.01f, velocity);
    assert(fabsf(d.progress - 4.39822972f) < 0.001f);
    assert(fabsf(ChassisOdom_PathSpeed((Geometry){35, 300}, lateral, 2, 1, 0, 1) - 439.822972f) <
           0.01f);
    assert(fabsf(ChassisOdom_PathSpeed((Geometry){35, 300}, lateral, 2, 1, 1, 0)) < 0.001f);
    return 0;
}
