#include "path_chassis.h"
/* Added after ZHY disc completion. No RDK, arm, RFID quota or turntable commands. */
static bool emit(PathMission *m, PathCommandKind k, float x, float y, float v, uint32_t arg,
                 uint32_t timeout)
{
    if (m->global_translation_inverted && (k == PC_MOVE || k == PC_BODY))
    {
        x = -x;
        y = -y;
    }
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
static bool align(PathMission *m, uint32_t now, const PathInput *in, uint32_t timeout_ms)
{
    if ((uint32_t)(now - m->entered) >= timeout_ms)
    {
        fail(m, PATH_TIMEOUT);
        return false;
    }
    if ((in->gray & 6U) == 6U)
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
    (void)emit(m, PC_BODY, 0, 25, 0, 0, 0);
    return false;
}
static void pillar(PathMission *m, uint32_t now, const PathInput *in)
{
    switch (m->phase)
    {
    case 0:
        if ((uint32_t)(now - m->entered) >= 10000)
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
                hold(m); /* Stop at first detection; debounce without advancing. */
            }
            if ((uint32_t)(now - m->stable_since) >= 30)
            {
                m->phase = 1;
                break;
            }
            break; /* Keep stopped while validating the IR level. */
        }
        else
            m->stable = false;
        (void)emit(m, PC_BODY, 0, 25, 0, 0, 10000);
        break;
    case 1:
        if (in->settled)
        {
            m->orbit_yaw = in->yaw_deg;
            m->started = now;
            if (emit(m, PC_BODY, 58.9f, 0, 49, 0, 15000))
                m->phase = 2;
        }
        break;
    case 2:
        m->orbit_ms = now - m->started;
        if (m->orbit_ms >= 15000)
            fail(m, PATH_TIMEOUT);
        else if (in->yaw_deg - m->orbit_yaw >= 352)
        {
            hold(m);
            m->phase = 3;
        }
        break;
    case 3:
        if (in->settled)
            next(m, now);
        break;
    default:
        fail(m, PATH_ERROR);
        break;
    }
}
static void stair(PathMission *m, uint32_t now, const PathInput *in)
{
    /* Original three scanning sections (2, 4, 2 positions), chassis only. */
    const unsigned points[] = {2, 4, 2};
    switch (m->phase)
    {
    case 0:
        if (align(m, now, in, 100000U))
        {
            m->phase = 1;
            m->part = 0;
            m->point = 0;
        }
        break;
    case 1:
        if (move(m, in, 18, 0, 40))
            m->phase = 2;
        break;
    case 2:
        if (++m->point == points[m->part])
        {
            if (m->part == 2)
                next(m, now);
            else
                m->phase = 4;
        }
        else
            m->phase = 3;
        break;
    case 3:
        if (move(m, in, 90, 0, 40))
            m->phase = 2;
        break;
    case 4:
        if (move(m, in, 117, 0, 40))
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
static void warehouse(PathMission *m, uint32_t now, const PathInput *in)
{
    static const float x[] = {0, 200, 200, 200, -200, -200};
    static const float y[] = {-50, 0, 0, 0, 0, 0};
    if (m->phase == 0)
    {
        if (rotate(m, in, 180))
        {
            m->phase = 1;
            m->entered = now;
        }
    }
    else if (m->phase == 1)
    {
        if (align(m, now, in, 5000U))
        {
            m->phase = 2;
            m->point = 0;
        }
    }
    else if (m->phase == 2 && m->point < 6)
    {
        if (move(m, in, x[m->point], y[m->point], 130) && ++m->point == 6)
        {
            hold(m);
            m->result = PATH_DONE;
        }
    }
    else
        fail(m, PATH_ERROR);
}
void PathChassis_Tick(PathMission *m, uint32_t now, const PathInput *in)
{
    switch (m->step)
    {
    case 4:
        next(m, now); /* Keep the heading after the disc task. */
        break;
    case 5:
        if (move(m, in, 1750, 0, 130)) next(m, now);
        break;
    case 6:
        pillar(m, now, in);
        break;
    case 7:
    case 10:
        next(m, now); /* No arm reset in the chassis-only extension. */
        break;
    case 8:
        if (m->phase == 0)
        {
            if (move(m, in, -330, 0, 130))
                m->phase = 1;
        }
        else if (m->phase == 1)
        {
            if (rotate(m, in, 180))
            {
                m->global_translation_inverted = true;
                next(m, now);
            }
        }
        else
            fail(m, PATH_ERROR);
        break;
    case 9:
        stair(m, now, in);
        break;
    case 11:
        if (move(m, in, 0, -1650, 130)) next(m, now);
        break;
    case 12:
        warehouse(m, now, in);
        break;
    default:
        fail(m, PATH_ERROR);
        break;
    }
}
