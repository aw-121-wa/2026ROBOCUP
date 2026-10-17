#include "path_yaw.h"
#include <stdio.h>
#define CHECK(x)                                                                                   \
    do                                                                                             \
    {                                                                                              \
        if (!(x))                                                                                  \
        {                                                                                          \
            printf("FAIL %d: %s\n", __LINE__, #x);                                                 \
            return 1;                                                                              \
        }                                                                                          \
    } while (0)
int main(void)
{
    PathYaw p = {0};
    PathYaw_Update(&p, 3.10f, true);
    PathYaw_Update(&p, -3.10f, true);
    CHECK(p.continuous > 3.18f && p.continuous < 3.19f);
    PathYaw_Update(&p, -2.0f, true);
    CHECK(p.continuous > 4.28f && p.continuous < 4.29f);
    PathYaw_Update(&p, 0, false);
    CHECK(!p.ready);
    PathYaw_Update(&p, 0, true);
    CHECK(p.continuous == 0);
    puts("continuous path yaw passed");
    return 0;
}
