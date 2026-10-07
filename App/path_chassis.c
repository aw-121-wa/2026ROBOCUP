#include "path_config.h"
#include "path_chassis.h"
#include "stair_heading.h"
#include "path_warehouse.h"
#define PILLAR_ENTRY_RADIUS_MM 100.0f
#define PILLAR_ENTRY_SPEED_RPM 45.0f
#define PILLAR_SEARCH_SPEED_RPM 30.0f
/* Post-disc route, including RDK preparation and stop/grab/resume at the pillar. */
static bool emit(PathMission *m, PathCommandKind k, float x, float y, float v, uint32_t arg,
                 uint32_t timeout)
{
    PathCommand c = {.kind = k, .x = x, .y = y, .speed = v,
                     .argument = arg, .timeout_ms = timeout};
    /* Blue field test uses negative body X and negative orbit rotation too;
     * only its approach is mirrored. Keep IR search along positive body Y. */
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
/* Brake when the station-specific lateral position condition is met. */
bool PathLine_Align(PathMission *m, uint32_t now, const PathInput *in,
                    uint32_t timeout, float lateral)
{
    if ((uint32_t)(now-m->entered)>=timeout) {
        fail(m,PATH_TIMEOUT); return false;
    }
    if (m->step == 9) {
        if (!isfinite(in->x_mm) || !isfinite(in->y_mm)) { fail(m,PATH_ERROR); return false; }
        if (!m->approach_started) {
            m->approach_started=true;
            m->approach_x=in->x_mm; m->approach_y=in->y_mm;
        }
        float x=in->x_mm-m->approach_x, y=in->y_mm-m->approach_y;
        /* Command odometry only limits fast travel; gray detection always takes priority. */
        if (in->gray || x*x+y*y >= 1080.0f*1080.0f) m->approach_slow=true;
        if (!m->approach_slow) lateral=80;
    }
    if (!m->stable) {
        if (m->step == 3 ? (in->gray & 6U) != 6U : !PathLine_Aligned(m, in->gray)) {
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
/* Orbit translation and rotation both scaled by 1.323, preserving the command radius. */
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
        (void)emit(m, PC_BODY, 0, 30, 0, 0, 10000);
        break;
    case 1:
        if (in->settled && !m->prep_pending)
        {
            if (!PATH_VISION_ENABLE)
            {
                m->orbit_yaw = in->yaw_deg;
                m->orbit_ms = 0;
                m->previous = now;
                if (emit(m, PC_BODY, -85.2012f, 0, -64.827f, 0, 15000)) m->phase = 2;
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
        else if (m->orbit_yaw - in->yaw_deg >= 353.0f)
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
            if (in->settled && (uint32_t)(now - m->entered) >= 300U) {
                if (PATH_BLUE_PILLAR_TEST && !PATH_BLUE_STAIR_TEST) { hold(m); m->result=PATH_DONE; }
                else next(m, now);
            }
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
            else if (emit(m, PC_BODY, -85.2012f, 0, -64.827f, 0, 15000)) m->phase = 2;
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
            else if (m->orbit_yaw - in->yaw_deg >= 353.0f)
                m->phase = 3;
            else if (emit(m, PC_BODY, -85.2012f, 0, -64.827f, 0, 15000 - m->orbit_ms)) m->phase = 2;
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
/* Absolute heading is checked only at safe boundaries, never during an arm task. */
bool PathHeading_Ready(PathMission *m, uint32_t now, const PathInput *in)
{
    /* During warehouse columns, the two inner white probes admit alignment.
     * Preserve absolute heading alignment for the final return-home stage. */
    if (m->step == 13 && m->point < 9 && (in->gray & 6U) == 6U) {
        if (!in->settled) {
            if (m->heading_align_active) hold(m);
            return false;
        }
        m->heading_align_active=false;
        return true;
    }
    float tolerance = m->blue ? 0.5f : (m->step <= 9 ? STAIR_HEADING_TOLERANCE_DEG : 0.1f);
    float target = m->step <= 9 ? STAIR_MAP_TARGET_DEG : 0.0f;
    float error = remainderf(target - in->map_yaw_deg, 360.0f);
    if (!isfinite(error)) { fail(m, PATH_ERROR); return false; }
    if (m->heading_align_active) {
        if ((uint32_t)(now-m->heading_align_since) >= 30000U) {
            fail(m, PATH_TIMEOUT); return false;
        }
        if (!in->settled) return false;
        if (fabsf(error) >= tolerance) {
            if (!emit(m, m->step <= 9 ? PC_MAP_AXIS : PC_HOME_ALIGN, 0,0,0,0,30000)) return false;
            return false;
        }
        m->heading_align_active = false;
        return true;
    }
    if (!in->settled) return false;
    if (fabsf(error) < tolerance) return true;
    if (!emit(m, m->step <= 9 ? PC_MAP_AXIS : PC_HOME_ALIGN, 0, 0, 0, 0, 30000))
        return false;
    m->heading_align_active = true;
    m->heading_align_since = now;
    return false;
}
bool PathLine_Aligned(const PathMission *m, uint8_t gray)
{
    /* Gray selects lateral position only; heading is checked separately. */
    gray &= 15U;
    if (m->step==13) return (gray & 6U)==6U;
    return gray == 6U || gray == 15U || gray == 9U;
}
static bool line_skip(PathMission *m, uint32_t now, const PathInput *in)
{
    if (m->line_recovery != 6) {
        hold(m); m->line_recovery = 6;
        return false;
    }
    if (!PathHeading_Ready(m, now, in)) return false;
    if (!emit(m, PC_MAP_HEADING, m->step == 9 ? STAIR_MAP_TARGET_DEG : 0, 0, 0, 0, 0)) return false;
    m->line_skipped = true;
    if (m->step == 9) m->stair_heading_locked = true;
    else m->warehouse_heading_locked = true;
    m->line_active = m->line_stopping = m->stable = false;
    return true;
}
bool PathLine_AlignFour(PathMission *m, uint32_t now, const PathInput *in)
{
    const bool bidirectional = m->step==9 || m->step==13;
    if (!isfinite(in->map_yaw_deg)) { fail(m, PATH_ERROR); return false; }
    if (m->step == 9 ? m->stair_heading_locked : (m->step != 13 && m->warehouse_heading_locked)) {
        m->line_skipped = true;
        return PathHeading_Ready(m, now, in);
    }
    if (!m->line_active) {
        if (!in->settled) return false;
        if (!emit(m, PC_MAP_HEADING, m->step == 9 ? STAIR_MAP_TARGET_DEG : 0, 0, 0, 0, 0)) return false;
        m->line_active = true; m->line_skipped = false;
        m->line_since = now; m->line_recovery = 0;
        m->line_stopping = m->stable = false;
    }
    /* Stop fully before reversing the lateral search. */
    if (m->line_recovery == 7 || (bidirectional && m->line_recovery == 9)) {
        if (!in->settled) return false;
        ++m->line_recovery;
        m->line_since=now;
        m->line_stopping=m->stable=false;
    }
    if (m->heading_align_active && !PathHeading_Ready(m, now, in)) return false;
    const uint32_t search_ms = bidirectional ? (m->step==13 && m->line_recovery==8 ? 4000U : 2000U) :
                               (m->line_recovery == 8 ? 5000U : 1000U);
    if (m->line_recovery == 6) return line_skip(m,now,in);
    if ((uint32_t)(now-m->line_since) >= search_ms &&
        ((!bidirectional && m->line_recovery == 0) || !PathLine_Aligned(m, in->gray))) {
        if (bidirectional) {
            if (m->line_recovery == 10) { fail(m,PATH_TIMEOUT); return false; }
            hold(m);
            m->line_recovery = m->line_recovery == 8 ? 9 : 7;
            return false;
        }
        if (m->step != 9 && m->line_recovery == 8) {
            fail(m,PATH_TIMEOUT); return false;
        }
        if (m->step != 9 && m->line_recovery == 1) {
            hold(m);
            m->line_recovery = 7;
            return false;
        }
        return line_skip(m,now,in);
    }
    if (PathLine_Aligned(m, in->gray) || m->heading_align_active) {
        if (!m->line_stopping) {
            hold(m); m->line_stopping = true; m->stable = false;
            return false;
        }
        if (!PathHeading_Ready(m, now, in)) { m->stable = false; return false; }
        /* A rotation may change gray; re-evaluate before admitting vision. */
        if (!PathLine_Aligned(m, in->gray)) return false;
        if (!m->stable) { m->stable = true; m->stable_since = now; }
        if ((uint32_t)(now-m->stable_since) < 100U) return false;
        m->line_active = m->line_stopping = m->stable = false;
        return true;
    }
    m->stable = false;
    if (m->line_stopping) {
        if (!in->settled) return false;
        m->line_stopping = false;
    }
    /* Gray controls lateral position; map search retains the gyro heading loop.
     * Warehouse sweeps right, left across the start, then right again. */
    if (!bidirectional && m->line_recovery == 0) m->line_recovery=1;
    (void)emit(m, PC_MAP_SEARCH, 0, m->step == 9 ? (m->line_recovery == 8 ? 40 : -40) : (m->line_recovery == 8 ? 10 : -10), 0, 0, m->step==13 ? 12000 : 3000);
    return false;
}
/* Continuous stair scan. Distances include braking and survive RFID pauses. */
static void stair(PathMission *m, uint32_t now, const PathInput *in)
{
    const float ends[] = {m->blue ? 100.0f : 120.0f, 500, 520, 860};
    if (m->phase >= 20) {
        if (m->point >= 4 || !isfinite(in->x_mm) || !isfinite(in->y_mm)) {
            fail(m, PATH_ERROR); return;
        }
        float d = (in->x_mm-m->stair_origin_x)*cosf(m->stair_axis) +
                  (in->y_mm-m->stair_origin_y)*sinf(m->stair_axis);
        if (!isfinite(d) || d < -10 || d > ends[3] + 20.0f) { fail(m,PATH_ERROR); return; }
        m->stair_distance = fmaxf(m->stair_distance,d);
        if ((uint32_t)(now-m->stair_started)>=300000U) { fail(m,PATH_TIMEOUT);return; }
        if (m->stair_scanning && in->reply==PATH_FAILED) { fail(m,PATH_ERROR);return; }
    }
    switch(m->phase) {
    case 0:
        if (PATH_VISION_ENABLE && !m->prep_pending && !m->stair_ready_started) {
            if (!emit(m,PC_GROUP,0,0,0,105,30000)) break;
            m->stair_ready_started=true; m->prep_pending=true; m->prep_since=now;
        }
        if (PathLine_Align(m,now,in,50000,25)) {
            if (PATH_STOP_AT_STAIR_LINE) {
                hold(m); m->result=PATH_DONE;
                break;
            }
            m->phase=1; m->point=m->grabs=0; m->waiting=false;
        }
        break;
    case 1:
        if (m->prep_pending) break; /* G2 may still be running during line approach. */
        if (!m->waiting && !PathHeading_Ready(m,now,in)) break;
        if (m->stair_ready_started || group(m,now,in,105)) {
            m->phase=4; m->line_active=false; m->stable=false;
        }
        break;
    case 4:
        if (PathLine_AlignFour(m,now,in)) {
            m->stair_origin_x=in->x_mm; m->stair_origin_y=in->y_mm;
            m->stair_axis=in->yaw_deg*0.01745329252f;
            m->stair_distance=0; m->stair_started=now;
            m->phase=30; m->waiting=false;
        }
        break;
    case 30: /* Advance five millimetres on the locked stair heading before vision. */
        if (!m->waiting) {
            if (!in->settled || !PathHeading_Ready(m,now,in)) break;
            m->waiting=emit(m,PC_MOVE,PATH_STAIR_ENTRY_ADVANCE_MM,0,20,0,5000); m->entered=now;
        } else if ((uint32_t)(now-m->entered)>=5000U) fail(m,PATH_TIMEOUT);
        else if (in->settled) {
            m->stair_origin_x=in->x_mm; m->stair_origin_y=in->y_mm;
            m->stair_distance=0; m->stair_started=now;
            m->phase=20; m->waiting=false;
        }
        break;
    case 20: /* Start a level only when stopped, with a fresh detection session. */
        if (!in->settled) break;
        m->stair_base_grabs=m->grabs;
        if (PATH_VISION_ENABLE && m->grabs<2) {
            if (emit(m,PC_STAIR_SCAN,0,0,0,(m->point == 0 ? 1 : m->point < 3 ? 2 : 3),180000)) {
                m->stair_scanning=true; m->phase=21;
            }
        } else { m->stair_scanning=false; m->phase=22; }
        break;
    case 21:
        if (in->vision_ready) { m->phase=22; m->waiting=false; }
        break;
    case 22: {
        float remaining=ends[m->point]-m->stair_distance;
        /* Boundary wins over a simultaneous BALL; never authorize old-level grabs. */
        if (remaining<=0.5f || (m->waiting && in->settled)) {
            if (remaining>2.0f) { fail(m,PATH_ERROR);break; }
            hold(m); m->phase=m->stair_scanning?25:27; m->waiting=false;
            break;
        }
        if (m->stair_scanning && in->ball_index>in->resume_index) {
            hold(m);m->phase=23;m->waiting=false;break;
        }
        if (!m->waiting) {
            if (!in->settled) break;
            /* Fast transit only after two confirmed grabs and scan END acknowledgement. */
            float speed = m->grabs>=2 && !m->stair_scanning ? 110 : 35;
            PathCommand c={.kind=PC_MOVE,.x=remaining,.speed=speed,
                .acceleration=(speed==110 ? PATH_STAIR_FAST_ACCEL : 650),.deceleration=650,.timeout_ms=30000};
            m->waiting=m->send(m->context,&c);
            if (!m->waiting) fail(m,PATH_ERROR);
        }
        break;
    }
    case 23:
        if (!in->settled) break;
        if (m->stair_distance>=ends[m->point]-0.5f) { m->phase=25;break; }
        if (!PathHeading_Ready(m,now,in)) break;
        if (emit(m,PC_PILLAR_STOPPED,0,0,0,in->ball_index,0)) m->phase=24;
        break;
    case 24:
        if (in->resume_index>m->grabs-m->stair_base_grabs) {
            m->grabs=m->stair_base_grabs+in->resume_index;
            m->phase=29; m->waiting=false;
        }
        break;
    case 29: /* RFID resume confirms the arm task has ended; check yaw before travel. */
        if (PathHeading_Ready(m,now,in)) m->phase=m->grabs>=2?25:22;
        break;
    case 25:
        if (in->settled && emit(m,PC_PILLAR_END,0,0,0,0,5000)) {
            m->phase=26; m->entered=now;
        }
        break;
    case 26:
        if ((uint32_t)(now-m->entered)>=5000U) { fail(m,PATH_TIMEOUT);break; }
        if (in->reply==PATH_OK) {
            m->stair_scanning=false; m->waiting=false;
            m->phase=m->stair_distance>=ends[m->point]-0.5f?27:22;
        }
        break;
    case 27:
        if (!in->settled) break;
        if (!m->waiting && !PathHeading_Ready(m,now,in)) break;
        if (m->point==0 && !group(m,now,in,4)) break;
        if (m->point==3) {
            if (PATH_BLUE_STAIR_TEST && !PATH_BLUE_WAREHOUSE_TEST) { hold(m); m->result=PATH_DONE; }
            else next(m,now);
            break;
        } /* G3 in formal mode, no extra retreat. */
        /* Each intermediate boundary must reacquire the line, even after a skip. */
        m->stair_heading_locked=false;
        m->line_active=m->line_stopping=m->stable=m->line_skipped=false;
        m->line_recovery=0;
        m->waiting=false;
        m->phase=28;
        break;
    case 28:
        if (PathLine_AlignFour(m,now,in)) {
            ++m->point;
            m->phase=20;
            m->waiting=false;
        }
        break;
    default: fail(m,PATH_ERROR);break;
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
        if (m->disc_depart_pending) {
            next(m, now); /* Travel now; send G1 only after DISC_DONE. */
        } else if (emit(m, PC_GROUP, 0, 0, 0, 1, 30000)) {
            m->prep_pending = true;
            m->prep_since = now;
            next(m, now); /* Run G1 while travelling to the pillar. */
        } else break;
        /* Begin travel in the same tick as G1. */
        /* fall through */
    case 5:
        if ((uint32_t)(now - m->entered) >= 30000U)
            fail(m, PATH_TIMEOUT);
        else if (m->phase == 1) {
            /* Body heading is now 180 degrees. The arc turns translation from
             * body +X to +Y, reaching the -1745 mm approach axis. */
            if (in->ir || in->motion_done) {
                next(m,now);
                pillar(m,now,in); /* IR wins; otherwise carry the arc exit speed. */
            }
        }
        else if (!m->waiting)
        {
            PathCommand c = {.kind = PC_MOVE_ROTATE, .x = -1745 + PILLAR_ENTRY_RADIUS_MM, .y = 0,
                             .angle = 180, .speed = 185, .end_speed = PILLAR_ENTRY_SPEED_RPM,
                             .continuous = true, .timeout_ms = 30000};
            if (m->blue || PATH_BLUE_PILLAR_TEST) { c.x=-c.x+70.0f; c.angle=-c.angle; }
            if (!(m->waiting = m->send(m->context, &c))) fail(m, PATH_ERROR);
        }
        else if (in->motion_done) {
            if (in->ir) {
                next(m,now); pillar(m,now,in);
            } else {
                PathCommand c = {.kind=PC_ARC,.x=PILLAR_ENTRY_RADIUS_MM,.y=0,.angle=90,
                                 .speed=PILLAR_ENTRY_SPEED_RPM,.start_speed=PILLAR_ENTRY_SPEED_RPM,
                                 .end_speed=PILLAR_SEARCH_SPEED_RPM,.continuous=true,.timeout_ms=10000};
                if (m->blue || PATH_BLUE_PILLAR_TEST) { c.y=180.0f-c.y; c.angle=-c.angle; }
                if (!m->send(m->context,&c)) fail(m,PATH_ERROR);
                else { m->phase=1; m->entered=now; }
            }
        }
        break;
    case 6:
        pillar(m, now, in);
        break;
    case 7:
        next(m, now);
        break;
    case 8:
        if (PATH_VISION_ENABLE && !m->prep_pending) {
            unsigned id = !m->stair_prep_started ? 2 : !m->stair_ready_started ? 105 : 0;
            if (id) {
                if (!emit(m,PC_GROUP,0,0,0,id,30000)) break;
                m->prep_pending=true; m->prep_since=now;
                if (id==2) m->stair_prep_started=true; else m->stair_ready_started=true;
            }
        }
        if (m->phase == 0)
        {
            if (!m->waiting && !emit(m,PC_MAP_HEADING,180,0,0,0,0)) break;
            if (!m->waiting) {
                PathCommand c={.kind=PC_MOVE,.x=-300,.speed=195,.end_speed=80,
                               .continuous=true,.timeout_ms=30000};
                if (!(m->waiting=m->send(m->context,&c))) fail(m,PATH_ERROR);
            } else if (in->motion_done) {
                PathCommand c={.kind=PC_ARC,.x=50,.y=180,.angle=-90,
                               .speed=80,.start_speed=80,.end_speed=80,
                               .continuous=true,.timeout_ms=10000};
                if (!m->send(m->context,&c)) { fail(m,PATH_ERROR); break; }
                m->approach_started=true; m->approach_x=in->x_mm; m->approach_y=in->y_mm;
                m->phase=2; m->entered=now;
            }
            if ((uint32_t)(now-m->entered)>=30000U) fail(m,PATH_TIMEOUT);
        }
        else if (m->phase == 2) {
            if ((uint32_t)(now-m->entered)>=10000U) { fail(m,PATH_TIMEOUT); break; }
            if (in->gray && !m->approach_slow) {
                /* Brake on an early line; never carry the fast arc across it. */
                hold(m); m->approach_slow=true;
                m->stable=PathLine_Aligned(m, in->gray);
            }
            if (m->approach_slow ? in->settled : in->motion_done) m->phase=3;
        }
        else if (m->phase == 1 || m->phase == 3)
        {
            if (m->phase == 1 && !m->waiting && !PathHeading_Ready(m, now, in)) break;
            bool detected=m->stable;
            if (!PATH_VISION_ENABLE) { next(m,now); m->stable=detected; break; }
            next(m,now); m->stable=detected;
        }
        else fail(m, PATH_ERROR);
        break;
    case 9:
        stair(m, now, in);
        break;
    case 10:
        if (!PATH_VISION_ENABLE) { next(m,now); break; }
        if (emit(m,PC_GROUP,0,0,0,3,30000)) {
            m->prep_pending=true; m->prep_since=now;
            next(m,now);
        }
        break;
    case 11:
        if ((uint32_t)(now-m->entered)>=30000U) { fail(m,PATH_TIMEOUT); break; }
        if (m->phase == 0) {
            if (!m->waiting) {
                m->approach_slow=false;
                PathCommand c={.kind=PC_MOVE_ROTATE,.y=-1425,.angle=180,.speed=155,
                               .end_speed=45,.continuous=true,.timeout_ms=30000};
                if (m->blue || PATH_BLUE_WAREHOUSE_TEST) { c.y += 50.0f; c.angle=-c.angle; }
                if (!(m->waiting=m->send(m->context,&c))) fail(m,PATH_ERROR);
            } else if (in->motion_done) {
                if (m->blue) { hold(m); m->phase=4; m->waiting=false; break; }
                PathCommand c={.kind=PC_ARC,.x=50,.y=90,.angle=-90,.speed=45,
                               .start_speed=45,.end_speed=45,.continuous=true,.timeout_ms=10000};
                if (!m->send(m->context,&c)) { fail(m,PATH_ERROR); break; }
                m->phase=2;
            }
        } else if (m->phase == 4 && m->blue) {
            if (!PathHeading_Ready(m,now,in)) break;
            /* Blue enters warehouse line search directly after heading settles. */
            next(m,now);
        } else if (m->phase == 2) {
            if (in->motion_done) { m->phase=1; m->waiting=false; }
        } else if (m->phase == 1) {
            if (!m->waiting) {
                PathCommand c={.kind=PC_MOVE,.x=110,.speed=125,.start_speed=45,
                               .end_speed=40,.continuous=true,.timeout_ms=10000};
                if (!(m->waiting=m->send(m->context,&c))) fail(m,PATH_ERROR);
            } else if (in->motion_done) {
                PathCommand c={.kind=PC_ARC,.x=25,.y=0,.angle=90,.speed=40,
                               .start_speed=40,.end_speed=40,.continuous=true,.timeout_ms=10000};
                if (!m->send(m->context,&c)) { fail(m,PATH_ERROR); break; }
                m->phase=3; m->stable=false;
            }
        } else if (m->phase == 3) {
            if (in->gray && !m->approach_slow) {
                hold(m); m->approach_slow=true; m->stable=PathLine_Aligned(m, in->gray);
            }
            if (m->approach_slow ? in->settled : in->motion_done) {
                bool detected=m->stable; next(m,now); m->stable=detected;
            }
        } else fail(m,PATH_ERROR);
        break;
    case 12:
        if (m->phase == 0)
        {
            if (PathLine_Align(m, now, in, 50000, 40))
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
