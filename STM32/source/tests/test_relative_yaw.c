#include "relative_yaw.h"
#include <stdio.h>
#define CHECK(x) do { if (!(x)) { printf("FAIL line %d\n", __LINE__); return 1; } } while (0)
#define DEG 0.017453292519943295f
int main(void)
{
    RelativeYaw p = {0};
    RelativeYaw_Update(&p, 60, 10, true, false);
    CHECK(!p.ready);
    RelativeYaw_Update(&p, 60, 11, true, true);
    CHECK(p.ready && p.yaw_rad == 0);
    RelativeYaw_Update(&p, 65, 12, true, true);
    CHECK(fabsf(p.yaw_rad - 5 * DEG) < 1e-6f);
    RelativeYaw_Update(&p, 80, 12, true, true);
    CHECK(fabsf(p.yaw_rad - 5 * DEG) < 1e-6f);
    RelativeYaw_Update(&p, 60, 13, true, true);
    CHECK(fabsf(p.yaw_rad) < 1e-6f);
    RelativeYaw_Update(&p, 60, 13, false, false);
    CHECK(!p.ready);
    RelativeYaw_Update(&p, 179, 14, true, true);
    CHECK(p.ready && p.yaw_rad == 0);
    RelativeYaw_Update(&p, -179, 15, true, false);
    CHECK(fabsf(p.yaw_rad - 2 * DEG) < 1e-6f);
    RelativeYaw_Update(&p, 179, 16, true, true);
    CHECK(fabsf(p.yaw_rad) < 1e-6f);
    RelativeYaw_Update(&p, NAN, 17, true, true);
    CHECK(!p.ready);
    RelativeYaw_Update(&p, 0, UINT32_MAX, true, true);
    RelativeYaw_Update(&p, 1, 0, true, true);
    CHECK(fabsf(p.yaw_rad - DEG) < 1e-6f);
    puts("relative yaw tests passed");
    return 0;
}
