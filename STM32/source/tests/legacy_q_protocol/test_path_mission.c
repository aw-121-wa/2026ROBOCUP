#include "path_mission.h"
#include <stdio.h>
#include <math.h>
#define CHECK(x)                                                                                   \
    do                                                                                             \
    {                                                                                              \
        if (!(x))                                                                                  \
        {                                                                                          \
            printf("FAIL %d: %s\n", __LINE__, #x);                                                 \
            return 1;                                                                              \
        }                                                                                          \
    } while (0)
typedef struct
{
    PathCommand commands[512];
    unsigned n;
} Port;
static bool send(void *ctx, const PathCommand *c)
{
    Port *p = ctx;
    if (p->n == 512)
        return false;
    p->commands[p->n++] = *c;
    return true;
}
static PathInput ready(void)
{
    return (PathInput){.armed = true,
                       .settled = true,
                       .gray = 6,
                       .ir = true,
                       .reply = PATH_OK,
                       .turn_reply = PATH_OK,
                       .interrupted_reply = PATH_OK};
}
static unsigned count(const Port *p, PathCommandKind kind, unsigned arg)
{
    unsigned n = 0;
    for (unsigned i = 0; i < p->n; i++)
        if (p->commands[i].kind == kind && p->commands[i].argument == arg)
            n++;
    return n;
}
static int full_route(void)
{
    Port p = {0};
    PathMission m;
    PathInput in = ready();
    Path_Init(&m, send, &p);
    CHECK(Path_Start(&m, 0, &in));
    unsigned seen = 0, disc = 0;
    for (unsigned t = 0; t < 180000 && m.result == PATH_RUNNING; t += 10)
    {
        in.rfid = 0;
        while (seen < p.n)
        {
            const PathCommand *c = &p.commands[seen++];
            if (c->kind == PC_DISC)
            {
                disc++;
                in.rfid = (uint16_t)(1U << disc);
            }
        }
        if (m.step == 6)
            in.yaw_deg += 1.0f;
        Path_Tick(&m, t, &in);
    }
    CHECK(m.result == PATH_DONE);
    CHECK(disc == 5);
    CHECK(m.ids == 0x3e);
    CHECK(count(&p, PC_ROTATE, 0) == 3);
    CHECK(count(&p, PC_GROUP, 101) == 1);
    CHECK(count(&p, PC_GROUP, 11) == 1 && count(&p, PC_GROUP, 8) == 1 &&
          count(&p, PC_GROUP, 5) == 1);
    unsigned g11 = 999, g8 = 999, g5 = 999;
    for (unsigned i = 0; i < p.n; i++)
    {
        if (p.commands[i].kind == PC_GROUP)
        {
            if (p.commands[i].argument == 11)
                g11 = i;
            if (p.commands[i].argument == 8)
                g8 = i;
            if (p.commands[i].argument == 5)
                g5 = i;
        }
    }
    CHECK(g11 < g8 && g8 < g5);
    CHECK(count(&p, PC_GROUP, 12) == 2);
    CHECK(count(&p, PC_GROUP, 9) == 4);
    CHECK(count(&p, PC_GROUP, 6) == 2);
    CHECK(count(&p, PC_TURN, 1) == 6); /* six warehouse reverse steps */
    CHECK(count(&p, PC_GROUP, 13) == 6 && count(&p, PC_GROUP, 14) == 3 &&
          count(&p, PC_GROUP, 15) == 3);
    return 0;
}
static int disc_timeout(void)
{
    Port p = {0};
    PathMission m;
    PathInput in = ready();
    Path_Init(&m, send, &p);
    CHECK(Path_Start(&m, 0, &in));
    unsigned t = 0;
    for (; t < 10000 && count(&p, PC_DISC, 0) == 0; t += 5)
        Path_Tick(&m, t, &in);
    CHECK(count(&p, PC_DISC, 0) == 1);
    in.reply = PATH_WAIT;
    in.rfid = (1U << 2);
    Path_Tick(&m, t, &in);
    in.rfid = 0;
    Path_Tick(&m, 30000, &in);
    CHECK(m.result == PATH_RUNNING);
    CHECK(count(&p, PC_CANCEL, 0) == 1);
    Path_Tick(&m, 30001, &in);
    CHECK(count(&p, PC_CANCEL, 0) == 1);
    CHECK(count(&p, PC_TURN, 0) == 0);
    in.reply = PATH_OK;
    Path_Tick(&m, 30005, &in);
    for (t = 30010; t < 30100; t += 5)
        Path_Tick(&m, t, &in);
    CHECK(count(&p, PC_DISC, 0) == 1);
    CHECK(count(&p, PC_TURN, 0) == 1);
    CHECK(m.result == PATH_TIMEOUT);
    return 0;
}
int main(void)
{
    {
        Port p = {0};
        PathMission q;
        PathInput input = ready();
        Path_Init(&q, send, &p);
        q.result = PATH_RUNNING;
        q.step = 3;
        q.phase = 3;
        input.reply = PATH_WAIT;
        input.rfid = 2;
        Path_Tick(&q, 20000, &input);
        CHECK(q.result == PATH_RUNNING);
        input.rfid = 0;
        input.reply = PATH_OK;
        input.interrupted_reply = PATH_NONE;
        Path_Tick(&q, 20005, &input);
        CHECK(q.result == PATH_TIMEOUT && q.grabs == 0 && count(&p, PC_TURN, 0) == 0);
    }
    {
        Port p = {0};
        PathMission q;
        PathInput input = ready();
        Path_Init(&q, send, &p);
        q.result = PATH_RUNNING;
        q.step = 3;
        q.phase = 3;
        q.ids = 2;
        q.grabs = 1;
        input.rfid = 2 | 1 | (1U << 10);
        input.reply = PATH_OK;
        Path_Tick(&q, 100, &input);
        CHECK(q.grabs == 1 && count(&p, PC_TURN, 0) == 0);
        input.rfid = 4;
        Path_Tick(&q, 105, &input);
        CHECK(q.grabs == 2);
        input.rfid = 0;
        Path_Tick(&q, 110, &input);
        CHECK(count(&p, PC_TURN, 0) == 1);
        Path_Init(&q, send, &p);
        q.result = PATH_RUNNING;
        q.step = 3;
        q.phase = 2;
        q.entered = 0xfffffff0U;
        Path_Tick(&q, 19983, &input);
        CHECK(q.result == PATH_RUNNING);
        Path_Tick(&q, 19984, &input);
        CHECK(q.expired); /* exact unsigned 20,000 ms rollover boundary */
        Path_Init(&q, send, &p);
        q.result = PATH_RUNNING;
        q.step = 6;
        q.phase = 4;
        q.grabs = 4;
        input.yaw_deg = 180;
        Path_Tick(&q, 5, &input);
        CHECK(q.step == 6 && q.phase == 4);
        input.yaw_deg = 352;
        Path_Tick(&q, 10, &input);
        CHECK(q.phase == 8);
    }
    {
        Port p = {0};
        PathMission q;
        PathInput input = ready();
        Path_Init(&q, send, &p);
        q.result = PATH_RUNNING;
        q.step = 3;
        q.phase = 1;
        q.waiting = true;
        input.reply = PATH_WAIT;
        Path_Tick(&q, 20000, &input);
        CHECK(q.result == PATH_RUNNING);
        input.reply = PATH_OK;
        Path_Tick(&q, 20005, &input);
        CHECK(q.result == PATH_TIMEOUT);
        Path_Init(&q, send, &p);
        q.result = PATH_RUNNING;
        q.step = 3;
        q.phase = 5;
        q.waiting = true;
        q.grabs = 5;
        Path_Tick(&q, 20800, &input);
        CHECK(q.result == PATH_RUNNING && q.step == 4);
    }
    Port port = {0};
    PathMission m;
    PathInput in = ready();
    Path_Init(&m, send, &port);
    in.armed = false;
    CHECK(!Path_Start(&m, 0, &in));
    in.armed = true;
    CHECK(Path_Start(&m, 0, &in));
    CHECK(!Path_Start(&m, 1, &in));
    for (unsigned t = 0; t < 100; t += 5)
        Path_Tick(&m, t, &in);
    CHECK(port.n >= 2);
    CHECK(port.commands[0].kind == PC_HELLO);
    CHECK(port.commands[1].kind == PC_GROUP && port.commands[1].argument == 0);
    CHECK(port.commands[2].kind == PC_MOVE);
    CHECK(fabsf(port.commands[2].x - 1691.4467f) < 0.02f);
    CHECK(fabsf(port.commands[2].y - 615.6363f) < 0.02f);
    Path_Cancel(&m);
    CHECK(m.result == PATH_CANCELED);
    CHECK(port.commands[port.n - 1].kind == PC_CANCEL);
    unsigned n = port.n;
    Path_Tick(&m, 100, &in);
    CHECK(port.n == n);
    CHECK(full_route() == 0);
    CHECK(disc_timeout() == 0);
    puts("PATH mission tests passed");
    return 0;
}
