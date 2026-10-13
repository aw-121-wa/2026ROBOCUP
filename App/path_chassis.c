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
static void leave_pillar(PathMission *m, uint32_t now, const PathInput *in)
{
    if (PATH_BLUE_PILLAR_TEST && !PATH_BLUE_STAIR_TEST) { hold(m);m->result=PATH_DONE;return; }
    if (PATH_ROUTE_VISION_ENABLE) {
        if (!emit(m,PC_PILLAR_END,0,0,0,0,5000)) { fail(m,PATH_ERROR);return; }
        m->pillar_depart_pending=m->prep_pending=true;
        m->prep_since=now;
    }
    m->step=8; m->phase=m->blue ? 4 : 0; m->waiting=false; m->stable=false; m->entered=now;
    PathChassis_Tick(m,now,in); /* Same tick, no stop or stationary-angle admission. */
}
/* Keep reduced translation speed; angular speed / 1.05 enlarges radius by 5%. */
static void pillar(PathMission *m, uint32_t now, const PathInput *in)
{
    const float orbit_degrees=m->blue ? ROUTE_ORBIT_BLUE_DEG : ROUTE_ORBIT_RED_DEG;
    const float orbit_x=m->blue ? ROUTE_ORBIT_BLUE_X_RPM : ROUTE_ORBIT_RED_X_RPM;
    const uint32_t orbit_limit=m->blue ? 25000U : 15000U;
    if ((PATH_ROUTE_VISION_ENABLE && in->reply == PATH_FAILED) ||
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
        (void)emit(m, PC_BODY, 0, ROUTE_PILLAR_IR_SEARCH_RPM, 0, 0, 10000);
        break;
    case 1:
        if (in->settled && !m->prep_pending)
        {
            if (m->blue) {
                float error=remainderf(90.0f-in->map_yaw_deg,360.0f);
                if (!isfinite(error)) { fail(m,PATH_ERROR);break; }
                if (m->heading_align_active && (uint32_t)(now-m->heading_align_since)>=PATH_HEADING_TIMEOUT_MS) {
                    fail(m,PATH_TIMEOUT);break;
                }
                if (fabsf(error)>0.2f) {
                    if (!m->heading_align_active) { m->heading_align_active=true;m->heading_align_since=now; }
                    (void)emit(m,PC_HOME_ALIGN,90,0,0,0,PATH_HEADING_TIMEOUT_MS);
                    break;
                }
                m->heading_align_active=false;
            }
            if (!PATH_ROUTE_VISION_ENABLE)
            {
                m->orbit_yaw = in->yaw_deg;
                m->orbit_ms = 0;
                m->previous = now;
                if (emit(m, PC_BODY, orbit_x, 0, ROUTE_ORBIT_ROTATION_RPM, 0, orbit_limit)) m->phase = 2;
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
        if (m->orbit_ms >= orbit_limit)
            fail(m, PATH_TIMEOUT);
        else if (PATH_ROUTE_VISION_ENABLE && in->ball_index > m->grabs)
        {
            hold(m);
            m->phase = 5;
            m->entered = now;
        }
        else if (m->orbit_yaw - in->yaw_deg >= orbit_degrees)
        {
            leave_pillar(m,now,in);
        }
        break;
    case 3:
        if (PATH_ROUTE_VISION_ENABLE && in->ball_index>m->grabs) {
            hold(m);m->phase=5;m->entered=now;
        } else leave_pillar(m,now,in);
        break;
    case 4:
        if (in->vision_ready)
        {
            m->orbit_yaw = in->yaw_deg;
            m->orbit_ms = 0;
            m->previous = now;
            if (PATH_ROUTE_VISION_ENABLE && in->ball_index > m->grabs)
            {
                hold(m);
                m->phase = 5;
                m->entered = now;
            }
            else if (emit(m, PC_BODY, orbit_x, 0, ROUTE_ORBIT_ROTATION_RPM, 0, orbit_limit)) m->phase = 2;
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
            if (PATH_ROUTE_VISION_ENABLE && in->ball_index > m->grabs)
            {
                hold(m);
                m->phase = 5;
                m->entered = now;
            }
            else if (m->orbit_yaw - in->yaw_deg >= orbit_degrees)
                leave_pillar(m,now,in);
            else if (emit(m, PC_BODY, orbit_x, 0, ROUTE_ORBIT_ROTATION_RPM, 0, orbit_limit - m->orbit_ms)) m->phase = 2;
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
    if (!PATH_ROUTE_VISION_ENABLE) return true;
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
/* Continuous stair scan. Distances include braking and survive RFID pauses. */
static void stair(PathMission *m, uint32_t now, const PathInput *in)
{
    const float ends[] = {m->blue ? ROUTE_STAIR_BLUE_FIRST_MM : ROUTE_STAIR_RED_FIRST_MM, ROUTE_STAIR_UNUSED_MM, m->blue ? ROUTE_STAIR_BLUE_SECOND_MM : ROUTE_STAIR_RED_SECOND_MM, m->blue ? ROUTE_STAIR_BLUE_END_MM : ROUTE_STAIR_RED_END_MM};
    if (m->phase >= 20 && m->phase != 30 && m->phase != 31) {
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
        if (PATH_ROUTE_VISION_ENABLE && !m->prep_pending && !m->stair_ready_started) {
            if (!emit(m,PC_GROUP,0,0,0,105,30000)) break;
            m->stair_ready_started=true; m->prep_pending=true; m->prep_since=now;
        }
        if (PathLine_Align(m,now,in,50000,PATH_STAIR_SEARCH_SLOW_RPM_COMMAND)) {
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
        /* Recheck strict 0110 after braking, using lateral search only. */
        if (m->blue ? in->settled : ((!m->line_active && in->settled && PathLine_Aligned(m,in->gray)) || PathLine_AlignFour(m,now,in))) {
            if (!emit(m,PC_MAP_HEADING,STAIR_TARGET_DEG(m->blue),0,0,0,0)) break;
            m->stair_heading_calibrated=true;
            m->stair_origin_x=in->x_mm; m->stair_origin_y=in->y_mm;
            m->stair_axis=in->yaw_deg*0.01745329252f;
            m->stair_distance=0; m->stair_started=now;
            m->phase=30; m->waiting=false;
        }
        break;
    case 30: /* Both sides back up to the rear PD1 falling edge. */
        if (!m->waiting) { m->entered=now;m->waiting=true; }
        if ((uint32_t)(now-m->entered)>=10000U) { fail(m,PATH_TIMEOUT);break; }
        if (!(in->gray&2U)) { hold(m);m->phase=31;break; }
        (void)emit(m,PC_BODY,ROUTE_STAIR_ENTRY_BACK_RPM,0,0,0,10000);
        break;
    case 31:
        if ((uint32_t)(now-m->entered)>=10000U) { fail(m,PATH_TIMEOUT);break; }
        if (in->settled) {
            m->stair_origin_x=in->x_mm;m->stair_origin_y=in->y_mm;
            m->stair_axis=in->yaw_deg*0.01745329252f;
            m->stair_distance=0;m->stair_started=now;m->phase=20;m->waiting=false;
        }
        break;
    case 20: /* Start a level only when stopped, with a fresh detection session. */
        if (!in->settled) break;
        m->stair_base_grabs=m->grabs;
        if (PATH_ROUTE_VISION_ENABLE && m->grabs<2) {
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
            /* Capture braking only for a detected ball, not line/level boundaries. */
            if (!emit(m,PC_HOLD,0,0,0,1,0)) break;
            m->phase=23;m->waiting=false;break;
        }
        if (!m->waiting) {
            if (!in->settled) break;
            /* Fast transit only after two confirmed grabs and scan END acknowledgement. */
            bool fast = m->grabs>=2 && !m->stair_scanning;
            float speed = m->blue ? (fast ? PATH_STAIR_FAST_SPEED_RPM : PATH_STAIR_SCAN_SPEED_RPM)
                                 : (fast ? RED_STAIR_FAST_SPEED_RPM : RED_STAIR_SCAN_SPEED_RPM);
            PathCommand c={.kind=PC_MOVE,.x=remaining,.speed=speed,
                .acceleration=(fast ? PATH_STAIR_FAST_ACCEL : ROUTE_STAIR_ACCEL_MM_S2),.deceleration=ROUTE_STAIR_DECEL_MM_S2,.timeout_ms=30000};
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
        /* Heading hold may resume after END was sent. G3 requires zero wheel
         * output: wait for settled instead of issuing a command the port rejects. */
        if (in->reply==PATH_OK && (m->grabs<2 || in->settled)) {
            m->stair_scanning=false; m->waiting=false;
            if (m->grabs>=2) {
                /* RESUME and END confirm that capture/storage is complete. Prepare
                 * the warehouse while crossing the entire remaining stair span. */
                if (!emit(m,PC_GROUP,0,0,0,3,30000)) break;
                m->warehouse_prep_started=true;
                m->prep_pending=true; m->prep_since=now;
                m->point=3; /* Keep the original 860 mm endpoint, skip level stops/G4. */
            }
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
        m->line_search_state=LINE_SEARCH_IDLE;
        m->waiting=false;
        m->phase=28;
        break;
    case 28:
        if (PathLine_AlignFour(m,now,in)) {
            m->point += (m->point==0) ? 2U : 1U; /* Skip the unused 500 mm intermediate boundary on both sides. */
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
        if (!PATH_ROUTE_VISION_ENABLE)
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
        if ((uint32_t)(now-m->entered)>=30000U) { fail(m,PATH_TIMEOUT); break; }
        if (!m->waiting) {
            /* Blue uses the opposite quarter-turn to face its side-mounted IR toward the pillar. */
            PathCommand c={.kind=PC_MOVE_ROTATE,.x=ROUTE_PILLAR_RED_X_MM,.y=ROUTE_PILLAR_RED_Y_MM,
                .angle=90,.speed=ROUTE_PILLAR_TRAVEL_RPM_COMMAND,.end_speed=PILLAR_SEARCH_SPEED_RPM_COMMAND,
                .continuous=true,.timeout_ms=30000};
            if (m->blue || PATH_BLUE_PILLAR_TEST) { c.x=ROUTE_PILLAR_BLUE_X_MM; c.y=ROUTE_PILLAR_BLUE_Y_MM; c.angle=-90; }
            if (!(m->waiting=m->send(m->context,&c))) fail(m,PATH_ERROR);
        } else if (in->motion_done) {
            next(m,now); pillar(m,now,in); /* Orbit remains gated by debounced IR. */
        }
        break;
    case 6:
        pillar(m, now, in);
        break;
    case 7:
        next(m, now);
        break;
    case 8:
        if (PATH_ROUTE_VISION_ENABLE && !m->prep_pending) {
            unsigned id = !m->stair_prep_started ? 2 : !m->stair_ready_started ? 105 : 0;
            if (id) {
                if (!emit(m,PC_GROUP,0,0,0,id,30000)) break;
                m->prep_pending=true; m->prep_since=now;
                if (id==2) m->stair_prep_started=true; else m->stair_ready_started=true;
            }
        }
        if (m->phase == 4) {
            /* Blue: shift toward the current body right before the stair exit turn. */
            if (!m->waiting) {
                PathCommand c={.kind=PC_ORBIT_EXIT,.y=ROUTE_BLUE_EXIT_RIGHT_MM,.angle=in->map_yaw_deg,
                               .speed=ROUTE_EXIT_CLEAR_RPM_COMMAND,.end_speed=ROUTE_EXIT_BOUNDARY_RPM_COMMAND,.continuous=true,.timeout_ms=5000};
                if (!(m->waiting=m->send(m->context,&c))) { fail(m,PATH_ERROR); break; }
            } else if (in->motion_done) {
                m->phase=5;m->waiting=false;m->entered=now;
                PathChassis_Tick(m,now,in);
            }
            if ((uint32_t)(now-m->entered)>=5000U) fail(m,PATH_TIMEOUT);
        }
        else if (m->phase == 5) {
            if (!m->waiting) {
                /* Blue stair approach points along map +Y; retain current heading until clear. */
                float yaw=in->map_yaw_deg*0.01745329252f;
                PathCommand c={.kind=PC_ORBIT_EXIT,.x=ROUTE_BLUE_EXIT_CLEAR_MM*sinf(yaw),.y=ROUTE_BLUE_EXIT_CLEAR_MM*cosf(yaw),
                               .angle=in->map_yaw_deg,.speed=ROUTE_EXIT_CLEAR_RPM_COMMAND,.end_speed=ROUTE_EXIT_BOUNDARY_RPM_COMMAND,
                               .continuous=true,.timeout_ms=10000};
                if (!(m->waiting=m->send(m->context,&c))) { fail(m,PATH_ERROR);break; }
            } else if (in->motion_done) {
                m->phase=0;m->waiting=false;m->entered=now;
                PathChassis_Tick(m,now,in);
            }
            if ((uint32_t)(now-m->entered)>=10000U) fail(m,PATH_TIMEOUT);
        }
        else if (m->phase == 0)
        {
            if (!m->waiting) {
                PathCommand c={.kind=PC_ORBIT_EXIT,.x=(m->blue || PATH_BLUE_STAIR_TEST) ? ROUTE_EXIT_BLUE_X_MM : ROUTE_EXIT_RED_X_MM,
                               .angle=STAIR_TARGET_DEG(m->blue || PATH_BLUE_STAIR_TEST),.speed=ROUTE_EXIT_TURN_RPM_COMMAND,
                               .end_speed=ROUTE_EXIT_BOUNDARY_RPM_COMMAND,
                               .continuous=true,.timeout_ms=30000};
                if (!(m->waiting=m->send(m->context,&c))) fail(m,PATH_ERROR);
            } else if (in->motion_done) {
                float speed=ROUTE_STAIR_ARC_RPM_COMMAND;
                PathCommand c={.kind=PC_ARC,.x=ROUTE_STAIR_ARC_RADIUS_MM,.y=180,.angle=-90,
                               .speed=speed,.start_speed=ROUTE_EXIT_BOUNDARY_RPM_COMMAND,.end_speed=speed,
                               .continuous=true,.timeout_ms=10000};
                if (m->blue || PATH_BLUE_STAIR_TEST) { c.y=180-c.y; c.angle=-c.angle; }
                if (!m->send(m->context,&c)) { fail(m,PATH_ERROR); break; }
                m->approach_started=true; m->approach_x=in->x_mm; m->approach_y=in->y_mm;
                m->phase=2; m->entered=now;
            }
            if ((uint32_t)(now-m->entered)>=30000U) fail(m,PATH_TIMEOUT);
        }
        else if (m->phase == 2) {
            if ((uint32_t)(now-m->entered)>=10000U) { fail(m,PATH_TIMEOUT); break; }
            /* Accept entry gray only after the applied translation is at slow speed. */
            if (in->gray && !m->approach_slow && isfinite(in->travel_rpm) &&
                in->travel_rpm<=PATH_STAIR_SEARCH_SLOW_RPM_COMMAND+2.0f) {
                hold(m);m->approach_slow=true;
                m->stable=PathLine_Aligned(m,in->gray);
            }
            if (m->approach_slow ? in->settled : in->motion_done) m->phase=3;
        }
        else if (m->phase == 1 || m->phase == 3)
        {
            if (m->phase == 1 && !m->waiting && !PathHeading_Ready(m, now, in)) break;
            bool detected=m->stable;
            if (!PATH_ROUTE_VISION_ENABLE) { next(m,now); m->stable=detected; break; }
            next(m,now); m->stable=detected;
        }
        else fail(m, PATH_ERROR);
        break;
    case 9:
        stair(m, now, in);
        break;
    case 10:
        if (!PATH_VISION_ENABLE || m->warehouse_prep_started) { next(m,now); break; }
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
                PathCommand c={.kind=PC_MOVE_ROTATE,.y=ROUTE_WAREHOUSE_TRAVEL_Y_MM,.angle=180,.speed=ROUTE_WAREHOUSE_TRAVEL_RPM_COMMAND,
                               .end_speed=m->blue ? ROUTE_WAREHOUSE_SEARCH_RPM_COMMAND : ROUTE_WAREHOUSE_RED_ENTRY_RPM_COMMAND,.continuous=true,.timeout_ms=30000};
                if (m->blue || PATH_BLUE_WAREHOUSE_TEST) c.angle=-c.angle;
                if (!(m->waiting=m->send(m->context,&c))) fail(m,PATH_ERROR);
            } else if (in->motion_done) {
                if (m->blue) {
                    next(m,now);PathChassis_Tick(m,now,in);
                    break; /* No entry arc: 40 rpm transition directly to line acquisition. */
                }
                PathCommand c={.kind=PC_ARC,.x=ROUTE_WAREHOUSE_RED_ARC_RADIUS_MM,.y=90,.angle=-90,.speed=ROUTE_WAREHOUSE_RED_ENTRY_RPM_COMMAND,
                               .start_speed=ROUTE_WAREHOUSE_RED_ENTRY_RPM_COMMAND,.end_speed=m->blue ? ROUTE_WAREHOUSE_SEARCH_RPM_COMMAND : ROUTE_WAREHOUSE_RED_ENTRY_RPM_COMMAND,.continuous=true,.timeout_ms=10000};
                if (m->blue) { c.y=180-c.y; c.angle=-c.angle; }
                if (!m->send(m->context,&c)) { fail(m,PATH_ERROR); break; }
                m->phase=2;m->entered=now;
            }
        } else if (m->phase == 2) {
            if (in->motion_done) {
                if (m->blue) {
                    next(m,now);
                    PathChassis_Tick(m,now,in); /* Carry 40 rpm into line acquisition without Hold. */
                } else { m->phase=1;m->waiting=false;m->entered=now; }
            }
        } else if (m->phase == 1) {
            if (m->waiting && in->motion_done) {
                PathCommand c={.kind=PC_ARC,.x=ROUTE_WAREHOUSE_RED_SECOND_ARC_MM,.y=0,.angle=90,.speed=ROUTE_WAREHOUSE_SEARCH_RPM_COMMAND,
                               .start_speed=ROUTE_WAREHOUSE_SEARCH_RPM_COMMAND,.end_speed=ROUTE_WAREHOUSE_SEARCH_RPM_COMMAND,.continuous=true,.timeout_ms=10000};
                if (!m->send(m->context,&c)) { fail(m,PATH_ERROR); break; }
                m->phase=3;m->stable=false;m->entered=now;
            } else if (!m->waiting) {
                PathCommand c={.kind=PC_MOVE,.x=ROUTE_WAREHOUSE_RED_FORWARD_MM,.speed=ROUTE_WAREHOUSE_RED_FORWARD_RPM_COMMAND,.start_speed=ROUTE_WAREHOUSE_RED_ENTRY_RPM_COMMAND,
                               .end_speed=ROUTE_WAREHOUSE_SEARCH_RPM_COMMAND,.continuous=true,.timeout_ms=10000};
                if (!(m->waiting=m->send(m->context,&c))) fail(m,PATH_ERROR);

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
            if (PathLine_Align(m, now, in, 50000, ROUTE_WAREHOUSE_SEARCH_RPM_COMMAND))
            {
                m->phase = 1;
                m->line_active = false;
                m->stable = false;
                m->entered = now;
            }
        }
        else if (m->phase==1)
        {
            if (PathLine_AlignFour(m,now,in)) {
                if (emit(m,PC_HOME_ALIGN,PATH_WAREHOUSE_TARGET_DEG(m->blue),0,
                         PATH_WAREHOUSE_ENTRY_ALIGN_DEG,0,PATH_HEADING_TIMEOUT_MS)) {
                    m->phase=2;m->entered=now;
                }
            }
        }
        else if (m->phase==2)
        {
            /* The deadline is a release condition, not a task error. */
            /* Settled acknowledges the controller's requested 0.05-degree precision. */
            if ((uint32_t)(now-m->entered)<PATH_WAREHOUSE_ENTRY_ALIGN_MS && !in->settled) break;
            hold(m);m->phase=3;
        }
        else if (in->settled)
        {
            if (PATH_STOP_AT_WAREHOUSE_LINE) { m->result=PATH_DONE;break; }
            m->point=0;next(m,now);
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
