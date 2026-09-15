#include "path_mission.h"
#include "disc_task_config.h"
#include <math.h>

/* Task-owned cooperative interpreter. No HAL, sleep, allocation or polling loops.
 * x/y are mm for MOVE, wheel RPM equivalents for BODY; speed is RPM (MOVE)
 * or rotation wheel RPM (BODY). The adapter converts using current geometry. */
static bool emit(PathMission *m, PathCommandKind k, float x, float y, float v, uint32_t arg,
                 uint32_t timeout)
{
    PathCommand c = {k, x, y, v, arg, timeout};
    if (m->send(m->context, &c))
        return true;
    m->result = PATH_ERROR;
    return false;
}
static void hold(PathMission *m)
{
    (void)emit(m, PC_HOLD, 0, 0, 0, 0, 0);
}
static void fail(PathMission *m, PathResult result)
{
    hold(m);
    (void)emit(m, PC_CANCEL, 0, 0, 0, 0, 30000);
    m->result = result;
}
static void next(PathMission *m, uint32_t now)
{
    m->step++;
    m->phase = 0;
    m->waiting = false;
    m->stable = false;
    m->entered = now;
}
void Path_Init(PathMission *m, PathSend send, void *ctx)
{
    *m = (PathMission){.send = send, .context = ctx};
}
bool Path_Start(PathMission *m, uint32_t now, const PathInput *in)
{
    if (m->result == PATH_RUNNING || !in->armed || in->fault || !in->settled)
        return false;
    PathSend send = m->send;
    void *ctx = m->context;
    *m = (PathMission){.send = send,
                       .context = ctx,
                       .result = PATH_RUNNING,
                       .phase = 99,
                       .entered = now,
                       .previous = now};
    return emit(m, PC_HELLO, 0, 0, 0, 0, 3000);
}
void Path_Cancel(PathMission *m)
{
    if (m->result == PATH_RUNNING)
        fail(m, PATH_CANCELED);
}
static bool remote(PathMission *m, const PathInput *in, PathCommandKind k, uint32_t arg,
                   uint32_t timeout)
{
    if (!m->waiting)
    {
        m->waiting = emit(m, k, 0, 0, 0, arg, timeout);
        return false;
    }
    if (in->reply == PATH_FAILED)
    {
        fail(m, PATH_ERROR);
        return false;
    }
    if (in->reply == PATH_WAIT)
        return false;
    if (in->reply != PATH_OK)
    {
        fail(m, PATH_ERROR);
        return false;
    }
    m->waiting = false;
    return true;
}
static bool move(PathMission *m, const PathInput *in, float x, float y, float rpm)
{
    if (!m->waiting)
    {
        m->waiting = emit(m, PC_MOVE, x, y, rpm, 0, 30000);
        return false;
    }
    if (!in->settled)
        return false;
    m->waiting = false;
    return true;
}
static bool rotate(PathMission *m, const PathInput *in, float deg)
{
    if (!m->waiting)
    {
        m->waiting = emit(m, PC_ROTATE, deg, 0, 0, 0, 15000);
        return false;
    }
    if (!in->settled)
        return false;
    m->waiting = false;
    return true;
}
static bool turn(PathMission *m, const PathInput *in, bool reverse)
{
    if (!m->waiting)
    {
        m->waiting = emit(m, PC_TURN, 0, 0, 0, reverse, 2000);
        return false;
    }
    if (in->turn_reply == PATH_FAILED)
    {
        fail(m, PATH_ERROR);
        return false;
    }
    if (in->turn_reply != PATH_OK)
        return false;
    m->waiting = false;
    return true;
}
static bool align(PathMission *m, uint32_t now, const PathInput *in)
{
    if ((uint32_t)(now - m->entered) >= 5000)
    {
        fail(m, PATH_TIMEOUT);
        return false;
    }
    if (in->gray == 6)
    {
        if (!m->stable)
        {
            m->stable = true;
            m->stable_since = now;
            hold(m);
        }
        return (uint32_t)(now - m->stable_since) >= 50 && in->settled;
    }
    m->stable = false;
    (void)emit(m, PC_BODY, 0, (in->gray & 9) ? 25 : -25, 0, 0, 0);
    return false;
}
static void disc(PathMission *m, uint32_t now, const PathInput *in)
{
    uint32_t elapsed = now - m->entered;
    if (elapsed >= DISC_TASK_TIMEOUT_MS && m->grabs < 5)
        m->expired = true;
    /* Capture raw IDs throughout complete G102. Accept only after its DONE (drop/return). */
    if (m->phase == 3 || m->phase == 4)
        m->candidate |= in->rfid & 0x3feU & ~m->ids;
    if (m->expired && m->phase < 3 && !(m->phase == 1 && m->waiting))
    {
        fail(m, PATH_TIMEOUT);
        return;
    }
    switch (m->phase)
    {
    case 0:
        if (align(m, now, in))
        {
            m->phase = 1;
            m->stable = false;
        }
        break;
    case 1:
        if (remote(m, in, PC_GROUP, 101, 30000))
        {
            if (m->expired)
                fail(m, PATH_TIMEOUT);
            else
                m->phase = 2;
        }
        break;
    case 2:
        m->candidate = 0;
        if (emit(m, PC_DISC, 0, 0, 0, 0, DISC_TASK_TIMEOUT_MS - elapsed))
            m->phase = 3;
        break;
    case 3:
        if (m->expired && !m->waiting)
        {
            /* Withdraw even a delayed/retried recognition request at the total
             * deadline. STOP waits for a G102 already underway without splitting it. */
            m->waiting = emit(m, PC_CANCEL, 0, 0, 0, 0, 30000);
            return;
        }
        if (in->reply == PATH_FAILED)
        {
            fail(m, PATH_ERROR);
            break;
        }
        if (in->reply == PATH_WAIT)
            break;
        if (m->waiting)
        {
            m->waiting = false;
            if (in->reply != PATH_OK || in->interrupted_reply != PATH_OK)
            {
                fail(m, PATH_TIMEOUT);
                break;
            }
        }
        if (in->reply == PATH_NONE)
        {
            if (m->expired)
                fail(m, PATH_TIMEOUT);
            else
                m->phase = 2;
            break;
        }
        m->phase = 4;
        /* fall through */
    case 4:
        if (m->candidate)
        {
            /* Exactly one ID belongs to one deposited ball. Multiple different IDs
             * in one grab are ambiguous: never assign or advance a wrong slot. */
            if (m->candidate & (m->candidate - 1U))
            {
                fail(m, PATH_ERROR);
                break;
            }
            m->ids |= m->candidate;
            m->grabs++;
            m->phase = 5;
            m->waiting = false;
        }
        else if (m->expired)
            fail(m, PATH_TIMEOUT);
        break;
    case 5:
        if (turn(m, in, false))
        {
            if (m->expired)
                fail(m, PATH_TIMEOUT);
            else if (m->grabs == 5)
            {
                next(m, now);
                m->grabs = 0;
            }
            else
                m->phase = 2;
        }
        break;
    default:
        fail(m, PATH_ERROR);
        break;
    }
}
static void pillar(PathMission *m, uint32_t now, const PathInput *in, uint32_t delta)
{
    switch (m->phase)
    {
    case 0:
        if ((uint32_t)(now - m->entered) >= 5000)
        {
            fail(m, PATH_TIMEOUT);
            break;
        }
        if (in->ir)
        {
            if (!m->stable)
            {
                m->stable = true;
                m->stable_since = now;
            }
            if ((uint32_t)(now - m->stable_since) >= 30)
            {
                hold(m);
                m->phase = 1;
                break;
            }
        }
        else
            m->stable = false;
        (void)emit(m, PC_BODY, 0, -25, 0, 0, 0);
        break;
    case 1:
        if (in->settled)
            m->phase = 2;
        break;
    case 2:
        if (remote(m, in, PC_GROUP, 3, 30000))
        {
            m->phase = 3;
            m->orbit_yaw = in->yaw_deg;
            m->orbit_ms = 0;
            m->grabs = 0;
        }
        break;
    case 3:
        if (m->grabs < 4 && !emit(m, PC_VISION, 0, 0, 0, 0, 1000))
            break;
        (void)emit(m, PC_BODY, -62, 0, 49, 0, 0);
        m->phase = 4;
        break;
    case 4:
        m->orbit_ms += delta;
        if (m->orbit_ms >= 15000)
        {
            fail(m, PATH_TIMEOUT);
            break;
        }
        if (in->yaw_deg - m->orbit_yaw >= 352)
        {
            hold(m);
            m->phase = 8;
            m->waiting = false;
            break;
        }
        if (m->grabs < 4)
        {
            if (in->reply == PATH_FAILED)
            {
                fail(m, PATH_ERROR);
                break;
            }
            if (in->reply == PATH_OK)
            {
                hold(m);
                m->phase = 5;
                break;
            }
            if (in->reply == PATH_NONE)
                m->phase = 3;
        }
        break;
    case 5:
        if (in->settled)
            m->phase = 6;
        break;
    case 6:
        if (remote(m, in, PC_GROUP, 4, 30000))
            m->phase = 7;
        break;
    case 7:
        if (turn(m, in, false))
        {
            m->grabs++;
            m->phase = 3;
        }
        break;
    case 8:
        if (!in->settled)
            break;
        /* Retire any outstanding vision request before the next group. */
        if (remote(m, in, PC_CANCEL, 0, 30000))
            next(m, now);
        break;
    default:
        fail(m, PATH_ERROR);
        break;
    }
}
static void stair(PathMission *m, uint32_t now, const PathInput *in)
{
    const unsigned pose[] = {11, 8, 5}, grab[] = {12, 9, 6}, transition[] = {10, 7, 0},
                   points[] = {2, 4, 2};
    switch (m->phase)
    {
    case 0:
        if (align(m, now, in))
        {
            m->phase = 1;
            m->part = 0;
            m->point = 0;
        }
        break;
    case 1:
        if (move(m, in, -18, 0, 40))
            m->phase = 2;
        break;
    case 2:
        if (remote(m, in, PC_GROUP, pose[m->part], 30000))
            m->phase = 3;
        break;
    case 3:
        if (emit(m, PC_VISION, 0, 0, 0, 0, 1000))
            m->phase = 4;
        break;
    case 4:
        if (in->reply == PATH_FAILED)
            fail(m, PATH_ERROR);
        else if (in->reply == PATH_OK)
            m->phase = 5;
        else if (in->reply == PATH_NONE)
            m->phase = 7;
        break;
    case 5:
        if (in->settled && remote(m, in, PC_GROUP, grab[m->part], 30000))
            m->phase = 6;
        break;
    case 6:
        if (turn(m, in, false))
            m->phase = 7;
        break;
    case 7:
        m->point++;
        if (m->point == points[m->part])
            m->phase = 11;
        else if (m->part == 1)
        {
            if (emit(m, PC_VISION, 0, 0, 0, 0, 1000) && emit(m, PC_MOVE, -90, 0, 40, 0, 10000))
                m->phase = 9;
        }
        else
            m->phase = 8;
        break;
    case 8:
        if (move(m, in, -90, 0, 40))
            m->phase = 3;
        break;
    case 9:
        if (in->reply == PATH_FAILED)
        {
            fail(m, PATH_ERROR);
            break;
        }
        if (in->reply == PATH_OK)
            hold(m);
        if (in->settled)
        {
            m->phase = 10;
            m->waiting = false;
        }
        break;
    case 10:
        /* Retire moving request. Fresh stationary request is the ONLY grab gate. */
        if (remote(m, in, PC_CANCEL, 0, 30000))
            m->phase = 3;
        break;
    case 11:
        if (remote(m, in, PC_GROUP, transition[m->part], 30000))
            m->phase = 12;
        break;
    case 12:
        if (m->part == 2)
        {
            next(m, now);
            break;
        }
        if (move(m, in, -117, 0, 40))
        {
            m->part++;
            m->point = 0;
            m->phase = 2;
        }
        break;
    default:
        fail(m, PATH_ERROR);
        break;
    }
}
typedef struct
{
    PathCommandKind kind;
    float x, y;
    unsigned arg;
} WarehouseStep;
static const WarehouseStep unload[] = {
    {PC_MOVE, 0, 50, 0},  {PC_MOVE, -200, 0, 0}, {PC_GROUP, 0, 0, 13}, {PC_GROUP, 0, 0, 14},
    {PC_TURN, 0, 0, 1},   {PC_MOVE, -200, 0, 0}, {PC_GROUP, 0, 0, 14}, {PC_GROUP, 0, 0, 13},
    {PC_TURN, 0, 0, 1},   {PC_MOVE, -200, 0, 0}, {PC_GROUP, 0, 0, 14}, {PC_GROUP, 0, 0, 13},
    {PC_TURN, 0, 0, 1},   {PC_GROUP, 0, 0, 15},  {PC_GROUP, 0, 0, 13}, {PC_TURN, 0, 0, 1},
    {PC_MOVE, 200, 0, 0}, {PC_GROUP, 0, 0, 15},  {PC_GROUP, 0, 0, 13}, {PC_TURN, 0, 0, 1},
    {PC_MOVE, 200, 0, 0}, {PC_GROUP, 0, 0, 15},  {PC_GROUP, 0, 0, 13}, {PC_TURN, 0, 0, 1}};
static void warehouse(PathMission *m, uint32_t now, const PathInput *in)
{
    if (m->phase == 0)
    {
        if (rotate(m, in, 180))
        {
            m->phase = 1;
            m->entered = now;
        }
        return;
    }
    if (m->phase == 1)
    {
        if (align(m, now, in))
        {
            m->phase = 2;
            m->point = 0;
        }
        return;
    }
    const WarehouseStep *s = &unload[m->point];
    bool done = false;
    if (s->kind == PC_MOVE)
        done = move(m, in, s->x, s->y, 130);
    else if (s->kind == PC_GROUP)
        done = remote(m, in, PC_GROUP, s->arg, 30000);
    else
        done = turn(m, in, true);
    if (done && ++m->point == sizeof(unload) / sizeof(unload[0]))
    {
        hold(m);
        m->result = PATH_DONE;
    }
}
void Path_Tick(PathMission *m, uint32_t now, const PathInput *in)
{
    if (m->result != PATH_RUNNING)
        return;
    uint32_t delta = now - m->previous;
    m->previous = now;
    if (!in->armed || in->fault)
    {
        fail(m, PATH_ERROR);
        return;
    }
    if (m->phase == 99)
    {
        if (in->reply == PATH_FAILED)
            fail(m, PATH_ERROR);
        else if (in->reply == PATH_OK)
            m->phase = 98;
        return;
    }
    if (m->phase == 98)
    {
        /* Legacy PATH readiness assumes the startup G0 posture. Establish it on
         * the RDK and wait for actual completion before the first chassis move. */
        if (remote(m, in, PC_GROUP, 0, 30000))
            m->phase = 0;
        return;
    }
    switch (m->step)
    {
    case 0:
        if (move(m, in, 1691.4467f, 615.6363f, 85))
            next(m, now);
        break;
    case 1:
        if (move(m, in, 2300, 0, 130))
            next(m, now);
        break;
    case 2:
    case 4:
        if (rotate(m, in, 178))
            next(m, now);
        break;
    case 3:
        disc(m, now, in);
        break;
    case 5:
        if (move(m, in, -1810, 0, 130))
            next(m, now);
        break;
    case 6:
        pillar(m, now, in, delta);
        break;
    case 7:
    case 10:
        if (remote(m, in, PC_GROUP, 0, 30000))
            next(m, now);
        break;
    case 8:
        if (move(m, in, 330, 0, 130))
            next(m, now);
        break;
    case 9:
        stair(m, now, in);
        break;
    case 11:
        if (move(m, in, 0, 1650, 130))
            next(m, now);
        break;
    case 12:
        warehouse(m, now, in);
        break;
    default:
        fail(m, PATH_ERROR);
        break;
    }
}
