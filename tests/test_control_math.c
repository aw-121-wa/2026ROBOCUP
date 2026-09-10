#include "heading.h"
#include "chassis_odom.h"
#include <assert.h>
#include <math.h>
int main(void)
{
    float i = 0;
    assert(fabsf(Heading_Update(0, 1, 0.005f, 2, 0.1f, 0.2f, 1, &i) + 0.2f) < 0.001f);
    assert(Heading_Update(10, 0, 1, 2, 1, 0, 1, &i) == 1);
    assert(i == 0.5f);
    float rpm[4] = {60, 60, 60, 60}, velocity[3];
    ChassisOdomDelta d =
        ChassisOdom_Integrate((Geometry){35, 300}, rpm, 0, 1, 1, 1, 0, 0.01f, velocity);
    assert(fabsf(d.progress - 2.19911486f) < 0.001f);
    assert(fabsf(d.x - d.progress) < 0.001f);
    assert(fabsf(d.y) < 0.001f);
    float lateral[4] = {-60, 60, 60, -60};
    d = ChassisOdom_Integrate((Geometry){35, 300}, lateral, 0, 2, 1, 0, 1, 0.01f, velocity);
    assert(fabsf(d.progress - 4.39822972f) < 0.001f);
    return 0;
}
