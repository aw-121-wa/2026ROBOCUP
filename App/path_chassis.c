#include "path_config.h"
#include "path_chassis.h"
#include "stair_heading.h"
#include "path_warehouse.h"
/* Post-disc route, including RDK preparation and stop/grab/resume at the pillar. */
static bool emit(PathMission *m, PathCommandKind k, float x, float y, float v, uint32_t arg,
                 uint32_t timeout)
{
    PathCommand c = {.kind = k, .x = x, .y = y, .speed = v,
                     .argument = arg, .timeout_ms = timeout};
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
static bool align(PathMission *m, uint32_t now, const PathInput *in, uint32_t timeout, float lateral)
{
    if ((uint32_t)(now - m->entered) >= timeout)
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
    (void)emit(m, PC_BODY, 0, lateral, 0, 0, timeout);
    return false;
}
static void pillar(PathMission *m, uint32_t now, const PathInput *in)
{
    if ((PATH_VISION_ENABLE && in->reply == PATH_FAILED) ||
        (m->phase >= 4 && (uint32_t)(now - m->entered) >=
                          (m->phase == 7 ? 5000U : 60000U)))
    {
        fail(m, in->reply == PATH_FAILED ? PATH_ERROR : PATH_TIMEOUT);
        return;
    }
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
            if (!PATH_VISION_ENABLE)
            {
                m->orbit_yaw = in->yaw_deg;
                m->orbit_ms = 0;
                m->previous = now;
                if (emit(m, PC_BODY, -58.9f, 0, -49, 0, 15000)) m->phase = 2;
                break;
            }
            if (emit(m, PC_VISION, 0, 0, 0, 0, 300000))
            {
                m->phase = 4; /* RDK runs G103 + camera warmup, then READY. */
                m->entered = now;
            }
        }
        break;
    case 2:
        m->orbit_ms += now - m->previous;
        m->previous = now;
        if (m->orbit_ms >= 15000)
            fail(m, PATH_TIMEOUT);
        else if (PATH_VISION_ENABLE && in->ball_index > m->grabs)
        {
            hold(m);
            m->phase = 5;
            m->entered = now;
        }
        else if (m->orbit_yaw - in->yaw_deg >= 356)
        {
            hold(m);
            m->entered = now;
            m->phase = 3;
        }
        break;
    case 3:
        if (!PATH_VISION_ENABLE)
        {
            /* Let the post-orbit angle settle before capturing the retreat heading. */
            if (in->settled && (uint32_t)(now - m->entered) >= 300U) next(m, now);
            break;
        }
        if (PATH_VISION_ENABLE && in->ball_index > m->grabs)
        {
            hold(m);
            m->phase = 5;
            m->entered = now;
        }
        else if (in->settled)
        {
            if (emit(m, PC_PILLAR_END, 0, 0, 0, 0, 5000))
            {
                m->phase = 7;
                m->entered = now;
            }
        }
        break;
    case 4:
        if (in->vision_ready)
        {
            m->orbit_yaw = in->yaw_deg;
            m->orbit_ms = 0;
            m->previous = now;
            if (PATH_VISION_ENABLE && in->ball_index > m->grabs)
            {
                hold(m);
                m->phase = 5;
                m->entered = now;
            }
            else if (emit(m, PC_BODY, -58.9f, 0, -49, 0, 15000)) m->phase = 2;
        }
        break;
    case 5:
        if (in->settled && emit(m, PC_PILLAR_STOPPED, 0, 0, 0, in->ball_index, 0))
        {
            m->phase = 6;
            m->entered = now;
        }
        break;
    case 6:
        if (in->resume_index > m->grabs)
        {
            m->grabs = in->resume_index;
            m->previous = now;
            if (PATH_VISION_ENABLE && in->ball_index > m->grabs)
            {
                hold(m);
                m->phase = 5;
                m->entered = now;
            }
            else if (m->orbit_yaw - in->yaw_deg >= 356)
                m->phase = 3;
            else if (emit(m, PC_BODY, -58.9f, 0, -49, 0, 15000 - m->orbit_ms)) m->phase = 2;
        }
        break;
    case 7:
        if (in->reply == PATH_OK) next(m, now);
        break;
    default:
        fail(m, PATH_ERROR);
        break;
    }
}
static bool group(PathMission *m, uint32_t now, const PathInput *in, unsigned id)
{
    if (!PATH_VISION_ENABLE) return true;
    if (!m->waiting)
    {
        m->entered = now;
        m->waiting = emit(m, PC_GROUP, 0, 0, 0, id, 30000);
    }
    else if (in->reply == PATH_FAILED || (uint32_t)(now - m->entered) >= 30000)
        fail(m, in->reply == PATH_FAILED ? PATH_ERROR : PATH_TIMEOUT);
    else if (in->reply == PATH_OK)
    {
        m->waiting = false;
        return true;
    }
    return false;
}
static void stair(PathMission *m, uint32_t now, const PathInput *in)
{
    static const float retreat[] = {-90, -117, -90, -90, -90, -117, -90};
    switch (m->phase)
    {
    case 0:
        if (align(m, now, in, 50000, -25)) /* Body -Y is right. */
        {
            m->phase = 5;
            m->point = m->grabs = 0; /* Stair count excludes disc and pillar balls. */
            m->waiting = false;
        }
        break;
    case 5: /* Retreat once after line alignment, before G105. */
        if (move(m, in, -55, 0, 40)) m->phase = 1;
        break;
    case 1:
        if (group(m, now, in, 105))
        {
            m->phase = 4;
            m->entered = now;
            m->stable = false;
        }
        break;
    case 2:
        if (!PATH_VISION_ENABLE)
        {
            if (in->settled) m->phase = 3;
            break;
        }
        if (m->grabs >= 2)
            m->phase = 3; /* Still visit every remaining point. */
        else if (!m->waiting)
        {
            if (in->settled)
            {
                m->entered = now;
                m->waiting = emit(m, PC_STAIR, 0, 0, 0, m->point + 1, 70000);
            }
        }
        else if (in->reply == PATH_FAILED || (uint32_t)(now - m->entered) >= 70000)
            fail(m, in->reply == PATH_FAILED ? PATH_ERROR : PATH_TIMEOUT);
        else if (in->reply == PATH_OK || in->reply == PATH_NONE)
        {
            if (in->reply == PATH_OK) ++m->grabs; /* DONE requires confirmed RFID. */
            m->waiting = false;
            m->phase = 3;
        }
        break;
    case 3:
        if (m->point == 7)
        {
            hold(m);
            if (m->result == PATH_RUNNING) next(m, now);
        }
        else if (move(m, in, retreat[m->point], 0, 40))
        {
            ++m->point;
            m->phase = 4;
            m->entered = now;
            m->stable = false;
        }
        break;
    case 4: /* Every point: settle and verify heading before RDK recognition. */
        if (!isfinite(in->imu_yaw_deg))
        {
            fail(m, PATH_ERROR);
            break;
        }
        if ((uint32_t)(now - m->entered) >= STAIR_HEADING_TIMEOUT_MS)
        {
            /* Best effort: stop correcting, then recognize without restarting alignment. */
            hold(m);
            m->waiting = false;
            m->stable = false;
            m->phase = 2;
            break;
        }
        if (!in->settled)
        {
            m->stable = false;
            break;
        }
        m->waiting = false;
        if (fabsf(StairHeading_Error(in->imu_yaw_deg)) > STAIR_HEADING_TOLERANCE_DEG)
        {
            m->stable = false;
            m->waiting = emit(m, PC_ALIGN_ZERO, 0, 0, 0, 0,
                              /* Mission owns the calibration deadline; watchdog must not fault first. */
                              STAIR_HEADING_TIMEOUT_MS + 1000U);
        }
        else if (!m->stable)
        {
            m->stable = true;
            m->stable_since = now;
        }
        else if ((uint32_t)(now - m->stable_since) >= STAIR_HEADING_STABLE_MS)
            m->phase = 2;
        break;
    default:
        fail(m, PATH_ERROR);
        break;
    }
}
void PathChassis_Tick(PathMission *m, uint32_t now, const PathInput *in)
{
    switch (m->step)
    {
    case 4:
        if (!PATH_VISION_ENABLE)
        {
            next(m, now);
            break;
        }
        if ((uint32_t)(now - m->entered) >= 30000U || in->reply == PATH_FAILED)
            fail(m, PATH_ERROR);
        else if (!m->waiting)
            m->waiting = emit(m, PC_GROUP, 0, 0, 0, 1, 30000);
        else if (in->reply == PATH_OK) next(m, now);
        break;
    case 5:
        if (move(m, in, 1750, 0, 130)) next(m, now);
        break;
    case 6:
        pillar(m, now, in);
        break;
    case 7:
        next(m, now); /* No arm reset in the chassis-only extension. */
        break;
    case 8:
        if (m->phase == 0)
        {
            if (move(m, in, -350, 0, 130)) m->phase = 1;
        }
        else if (m->phase == 1)
        {
            if (group(m, now, in, 2)) m->phase = 2;
        }
        else if (m->phase == 2)
        {
            if (rotate(m, in, 180)) next(m, now);
        }
        else fail(m, PATH_ERROR);
        break;
    case 9:
        stair(m, now, in);
        break;
    case 10:
        if (m->phase == 0)
        {
            if (group(m, now, in, 3))
            {
                m->phase = 1;
                m->entered = now;
            }
        }
        else if ((uint32_t)(now - m->entered) >= 30000U)
            fail(m, PATH_TIMEOUT);
        else if (move(m, in, -100, 0, 40)) next(m, now);
        break;
    case 11:
        if ((uint32_t)(now - m->entered) >= 30000U)
            fail(m, PATH_TIMEOUT);
        else if (m->phase == 1)
        {
            if (move(m, in, -200, 0, 40)) next(m, now);
        }
        else if (!m->waiting)
        {
            PathCommand c = {.kind = PC_MOVE_ROTATE, .x = 0, .y = 1500,
                             .angle = 180, .speed = 60, .timeout_ms = 30000};
            if (!(m->waiting = m->send(m->context, &c))) fail(m, PATH_ERROR);
        }
        else if (in->settled)
        {
            m->phase = 1;
            m->waiting = false;
            m->entered = now;
        }
        break;
    case 12:
        if (align(m, now, in, 50000, -25))
        {
            hold(m);
            if (m->result == PATH_RUNNING)
            {
                m->point = 0;
                next(m, now);
            }
        }
        break;
    case 13:
        PathWarehouse_Tick(m, now, in);
        break;
    default:
        fail(m, PATH_ERROR);
        break;
    }
}
