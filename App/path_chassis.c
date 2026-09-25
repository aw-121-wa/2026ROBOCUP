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
/* Detect the two inner probes, brake lateral search, then continue.
 * Outer probes remain available in telemetry but do not block the route. */
bool PathLine_Align(PathMission *m, uint32_t now, const PathInput *in,
                    uint32_t timeout, float lateral)
{
    if ((uint32_t)(now-m->entered)>=timeout) {
        fail(m,PATH_TIMEOUT); return false;
    }
    if (!m->stable) {
        if ((in->gray & 6U)!=6U) {
            (void)emit(m,PC_BODY,0,lateral,0,0,timeout);
            return false;
        }
        hold(m);
        m->stable=true; /* Latch detection; braking may carry probes past the line. */
        return false;
    }
    if (!in->settled) return false;
    /* Preserve the current heading for later stair checks; do not turn to calibrate. */
    if (!emit(m,PC_LINE_REFERENCE,0,0,0,0,0)) return false;
    m->stable=false;
    return true;
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
                if (emit(m, PC_BODY, -64.4f, 0, -49, 0, 15000)) m->phase = 2;
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
        else if (m->orbit_yaw - in->yaw_deg >= 355)
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
            else if (emit(m, PC_BODY, -64.4f, 0, -49, 0, 15000)) m->phase = 2;
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
            else if (m->orbit_yaw - in->yaw_deg >= 355)
                m->phase = 3;
            else if (emit(m, PC_BODY, -64.4f, 0, -49, 0, 15000 - m->orbit_ms)) m->phase = 2;
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
/* Four-probe gate: bounded yaw sweep at each nearby lateral position.
 * Bidirectional search avoids assuming which side a missing digital probe is on. */
bool PathLine_AlignFour(PathMission *m, uint32_t now, const PathInput *in)
{
    if (!isfinite(in->yaw_deg) || !isfinite(in->imu_yaw_deg)) {
        fail(m, PATH_ERROR); return false;
    }
    unsigned count = 0;
    for (unsigned bits = in->gray & 15U; bits; bits >>= 1) count += bits & 1U;
    if (!m->line_active) {
        m->line_active = true;
        m->line_since = now;
        m->line_best_count = count;
        m->line_recovery = m->line_retries = 0;
        m->line_losing = false;
        m->line_scan_yaw = in->yaw_deg;
        m->line_scan_stage = m->line_shift_count = 0;
        m->line_stopping = m->stable = false;
    }
    if (fabsf(in->yaw_deg - m->line_scan_yaw) > 10.0f) {
        fail(m, PATH_ERROR); return false;
    }
    if ((uint32_t)(now - m->line_since) >= 30000U) {
        fail(m, PATH_TIMEOUT); return false;
    }
    /* Stair-only map-left recovery: current chassis right, y < 0.
     * Keep the original timeout/yaw bounds across retries. */
    if (m->line_recovery) {
        if (!in->settled) return false;
        if (m->line_recovery == 1) {
            if (emit(m, PC_MOVE, 0, -10, 8, 0, 5000)) m->line_recovery = 2;
            return false;
        }
        m->line_recovery = 0;
        m->line_best_count = count;
        m->line_losing = m->line_stopping = m->stable = false;
        m->line_scan_stage = m->line_shift_count = 0;
        m->line_shift_since = 0;
    }
    /* Do not keep sweeping when coverage is demonstrably getting worse.
     * Ignore brief sensor chatter; never calibrate a degraded pose. */
    if (count < m->line_best_count) {
        if (!m->line_losing) {
            m->line_losing = true;
            m->line_loss_since = now;
        }
        if ((uint32_t)(now - m->line_loss_since) >= 100U) {
            if (m->step == 9 && m->line_retries < 2) {
                hold(m);
                ++m->line_retries;
                m->line_recovery = 1;
                m->stable = false;
                return false;
            }
            fail(m, PATH_ERROR); return false;
        }
    } else {
        m->line_losing = false;
        m->line_best_count = count;
    }
    if (in->gray == 15) {
        if (!m->line_stopping) {
            hold(m); m->line_stopping = true; m->stable = false;
            return false;
        }
        if (!in->settled) { m->stable = false; return false; }
        if (!m->stable) { m->stable = true; m->stable_since = now; }
        if ((uint32_t)(now - m->stable_since) < 100U) return false;
        if (!emit(m, PC_LINE_CALIBRATE, 0, 0, 0, 0, 0)) return false;
        m->line_active = m->line_stopping = m->stable = false;
        return true;
    }
    m->stable = false;
    if (m->line_stopping) {
        if (!in->settled) return false;
        m->line_stopping = false;
    }
    if (m->line_scan_stage < 3) {
        static const float offsets[] = {-6, 6, 0};
        float side = (m->line_retries & 1U) ? -1.0f : 1.0f;
        float error = m->line_scan_yaw + side * offsets[m->line_scan_stage] - in->yaw_deg;
        if (fabsf(error) <= 0.5f) {
            hold(m); m->line_stopping = true;
            ++m->line_scan_stage;
            m->line_shift_since = 0;
        } else {
            (void)emit(m, PC_LINE_SEARCH, 0, 0, error > 0 ? 4 : -4, 0, 31000);
        }
    } else {
        if (!m->line_shift_since) m->line_shift_since = now;
        /* +7.5, -7.5, +15, -15 mm approximately, capped by total timeout. */
        uint32_t duration = 250U * (m->line_shift_count + 1U);
        if (duration > 1000U) duration = 1000U;
        if ((uint32_t)(now - m->line_shift_since) >= duration) {
            hold(m); m->line_stopping = true;
            ++m->line_shift_count; m->line_scan_stage = 0;
        } else {
            (void)emit(m, PC_LINE_SEARCH, 0, (m->line_shift_count & 1U) ? -8 : 8,
                       0, 0, 31000);
        }
    }
    return false;
}
static void stair(PathMission *m, uint32_t now, const PathInput *in)
{
    static const float retreat[] = {90, 117, 90, 90, 90, 117, 90};
    switch (m->phase)
    {
    case 0:
        if (PathLine_Align(m, now, in, 50000, 25)) /* Reverse only the post-orbit stair line approach. */
        {
            m->phase = 5;
            m->point = m->grabs = 0; /* Stair count excludes disc and pillar balls. */
            m->waiting = false;
        }
        break;
    case 5: /* Retreat once after line alignment, before G105. */
        if (move(m, in, 55, 0, 40)) m->phase = 1;
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
        if (!m->waiting && in->gray != 15) {
            m->line_active = false; m->phase = 4;
            break;
        }
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
    case 4: /* Every stopped stair point must physically cover all four probes. */
        if (PathLine_AlignFour(m, now, in)) {
            m->waiting = false;
            m->phase = 2;
        }
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
        if ((uint32_t)(now - m->entered) >= 30000U)
            fail(m, PATH_TIMEOUT);
        else if (!m->waiting)
        {
            PathCommand c = {.kind = PC_MOVE_ROTATE, .x = -1797, .y = 0,
                             .angle = 180, .speed = 130, .timeout_ms = 30000};
            if (!(m->waiting = m->send(m->context, &c))) fail(m, PATH_ERROR);
        }
        else if (in->settled) next(m, now);
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
            if (group(m, now, in, 2)) next(m, now);
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
        else if (move(m, in, 100, 0, 40)) next(m, now);
        break;
    case 11:
        if ((uint32_t)(now - m->entered) >= 30000U)
            fail(m, PATH_TIMEOUT);
        else if (m->phase == 1)
        {
            if (move(m, in, 200, 0, 40)) next(m, now);
        }
        else if (!m->waiting)
        {
            PathCommand c = {.kind = PC_MOVE_ROTATE, .x = 0, .y = -1500,
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
        if (m->phase == 0)
        {
            if (PathLine_Align(m, now, in, 50000, 25))
            {
                m->phase = 1;
                m->line_active = false;
                m->stable = false;
                m->entered = now;
            }
        }
        else if (PathLine_AlignFour(m, now, in))
        {
            m->point = 0;
            next(m, now);
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
