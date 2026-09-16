#include "path_mission.h"
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
static PathCommand last;
static bool send(void *ctx, const PathCommand *c)
{
    (void)ctx;
    last = *c;
    return true;
}
int main(void)
{
    PathMission m;
    PathInput in = {.armed = true, .settled = true, .reply = PATH_OK};
    for (unsigned gray = 0; gray < 16; gray++)
    {
        Path_Init(&m, send, 0);
        m.result = PATH_RUNNING;
        m.step = 3;
        in.gray = gray;
        Path_Tick(&m, 5, &in);
        if ((gray & 6) == 6)
        {
            CHECK(last.kind == PC_HOLD);
            Path_Tick(&m, 54, &in);
            CHECK(m.phase == 0);
            Path_Tick(&m, 55, &in);
            CHECK(last.kind == PC_DISC && m.phase == 1);
        }
        else
        {
            CHECK(last.kind == PC_BODY && last.y == -25);
            Path_Tick(&m, 5000, &in);
            CHECK(m.result == PATH_TIMEOUT);
        }
    }
    Path_Init(&m, send, 0);
    CHECK(Path_Start(&m, 0, &in));
    CHECK(last.kind == PC_HELLO);
    in.reply = PATH_WAIT;
    Path_Tick(&m, 5, &in);
    CHECK(m.phase == 99);
    in.reply = PATH_OK;
    Path_Tick(&m, 10, &in);
    Path_Tick(&m, 15, &in);
    CHECK(last.kind == PC_MOVE);
    Path_Cancel(&m);
    CHECK(m.result == PATH_CANCELED);
    return 0;
}
