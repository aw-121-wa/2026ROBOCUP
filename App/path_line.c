#include "path_chassis.h"
#include "path_policy.h"
/* Nonblocking line acquisition: gray decides position, IMU decides heading.
 * Every reversal waits for the existing chassis settled acknowledgement. */
static bool emit(PathMission *m, PathCommandKind kind, float x, float y, float speed,
                 uint32_t argument, uint32_t timeout)
{
    PathCommand command={.kind=kind,.x=x,.y=y,.speed=speed,
                         .argument=argument,.timeout_ms=timeout};
    if (m->send(m->context,&command)) return true;
    m->result=PATH_ERROR;
    return false;
}
static void hold(PathMission *m) { (void)emit(m,PC_HOLD,0,0,0,0,0); }
static void fail(PathMission *m, PathResult result)
{
    hold(m);
    (void)emit(m,PC_CANCEL,0,0,0,0,30000);
    m->result=result;
}
/* Brake when the station-specific lateral position condition is met. */
bool PathLine_Align(PathMission *m, uint32_t now, const PathInput *in,
                    uint32_t timeout, float lateral)
{
    if (!PATH_WAREHOUSE_UNTIMED(m) && (uint32_t)(now-m->entered)>=timeout) {
        fail(m,PATH_TIMEOUT); return false;
    }
    if (m->step == 9) {
        if (!isfinite(in->x_mm) || !isfinite(in->y_mm)) { fail(m,PATH_ERROR); return false; }
        if (!m->approach_started) {
            m->approach_started=true;
            m->approach_x=in->x_mm; m->approach_y=in->y_mm;
        }
        float x=in->x_mm-m->approach_x, y=in->y_mm-m->approach_y;
        /* Ignore gray during fast approach and while the slow command is ramping down. */
        if (x*x+y*y >= PATH_STAIR_SEARCH_FAST_DISTANCE_MM*PATH_STAIR_SEARCH_FAST_DISTANCE_MM ||
            (in->gray && isfinite(in->travel_rpm) && in->travel_rpm<=PATH_STAIR_SEARCH_SLOW_RPM+2.0f)) m->approach_slow=true;
        if (!m->approach_slow) lateral=PATH_STAIR_SEARCH_FAST_RPM;
        if (!m->line_entry_detected && (!m->approach_slow ||
            !isfinite(in->travel_rpm) || in->travel_rpm>PATH_STAIR_SEARCH_SLOW_RPM+2.0f)) {
            (void)emit(m,PC_BODY,0,lateral,0,0,timeout);
            return false;
        }
    }
    if (!m->line_entry_detected) {
        if (!in->gray) {
            (void)emit(m,PC_BODY,0,lateral,0,0,timeout);
            return false;
        }
        hold(m);
        m->line_entry_detected=true;
        return false;
    }
    /* Blue stair arrival must reach rear-off retreat, even if braking changes gray. */
    if (m->blue && m->step==9) {
        if (!in->settled) return false;
        m->line_entry_detected=false;
        return true;
    }
    if (!m->line_active && in->settled && PathLine_Aligned(m,in->gray)) {
        if (!emit(m,PC_LINE_REFERENCE,0,0,0,0,0)) return false;
    } else if (!PathLine_AlignFour(m,now,in)) return false;
    m->line_entry_detected=false;
    return true;
}
/* Admission gate: calibrate once per work area; continuous hold is independent. */
bool PathHeading_Ready(PathMission *m, uint32_t now, const PathInput *in)
{
    /* Work-area gray acquisition never requests a stationary rotation.
     * The existing gyro controller retains heading during lateral motion. */
    if (m->step==9 || m->step==12 || (m->step==13 && m->point<9))
        return in->settled;
    float tolerance = m->step <= 9 ? STAIR_HEADING_TOLERANCE_DEG : PATH_WAREHOUSE_HEADING_TOLERANCE_DEG;
    float target = PathPolicy_Target(m);
    float error = remainderf(target - in->map_yaw_deg, 360.0f);
    if (!isfinite(error)) { fail(m, PATH_ERROR); return false; }
    if (m->heading_align_active) {
        if ((uint32_t)(now-m->heading_align_since) >= PATH_HEADING_TIMEOUT_MS) {
            fail(m, PATH_TIMEOUT); return false;
        }
        if (!in->settled) return false;
        if (fabsf(error) >= tolerance) {
            if (!emit(m, m->step <= 9 ? PC_MAP_AXIS : PC_HOME_ALIGN, target,0,0,0,PATH_HEADING_TIMEOUT_MS)) return false;
            return false;
        }
        m->heading_align_active = false;
        return true;
    }
    if (!in->settled) return false;
    if (fabsf(error) < tolerance) return true;
    if (!emit(m, m->step <= 9 ? PC_MAP_AXIS : PC_HOME_ALIGN, target, 0, 0, 0, PATH_HEADING_TIMEOUT_MS))
        return false;
    m->heading_align_active = true;
    m->heading_align_since = now;
    return false;
}
typedef struct {
    uint16_t accepted_patterns;
    bool bidirectional;
    float lateral_mm_s;
    uint32_t first_ms, reverse_ms, motion_timeout_ms;
} LinePolicy;
static LinePolicy line_policy(const PathMission *m)
{
    bool stair=m->step==9, warehouse=m->step==13;
    return (LinePolicy){
        .accepted_patterns=(1U<<6),
        .bidirectional=true,
        .lateral_mm_s=stair ? PATH_STAIR_LINE_MM_S : PATH_WAREHOUSE_LINE_MM_S,
        .first_ms=stair || warehouse ? PATH_LINE_SWEEP_MS : PATH_LINE_ENTRY_SWEEP_MS,
        .reverse_ms=warehouse ? PATH_WAREHOUSE_REVERSE_MS :
                    stair ? PATH_LINE_SWEEP_MS : PATH_LINE_ENTRY_REVERSE_MS,
        .motion_timeout_ms=warehouse ? PATH_WAREHOUSE_SEARCH_TIMEOUT_MS : PATH_LINE_SEARCH_TIMEOUT_MS};
}
bool PathLine_Aligned(const PathMission *m, uint8_t gray)
{
    return (line_policy(m).accepted_patterns & (1U<<(gray&15U))) != 0;
}
bool PathLine_AlignFour(PathMission *m, uint32_t now, const PathInput *in)
{
    const LinePolicy policy=line_policy(m);
    const bool bidirectional=policy.bidirectional;
    if (!isfinite(in->map_yaw_deg)) { fail(m, PATH_ERROR); return false; }
    if (!m->line_active) {
        if (!in->settled) return false;
        if (!emit(m, PC_MAP_HEADING, m->step == 9 ? STAIR_TARGET_DEG(m->blue) : PATH_WAREHOUSE_TARGET_DEG(m->blue), 0, 0, 0, 0)) return false;
        m->line_active = true; m->line_skipped = false;
        m->line_since = now; m->line_search_state = LINE_SEARCH_IDLE;
        m->line_stopping = m->stable = false;
    }
    /* Stop fully before reversing the lateral search. */
    if (m->line_search_state == LINE_BRAKE_REVERSE || (bidirectional && m->line_search_state == LINE_BRAKE_FINAL)) {
        if (!in->settled) return false;
        m->line_search_state = m->line_search_state == LINE_BRAKE_REVERSE
                               ? LINE_SWEEP_REVERSE : LINE_SWEEP_FINAL;
        m->line_since=now;
        m->line_stopping=m->stable=false;
    }
    const uint32_t search_ms=m->line_search_state==LINE_SWEEP_REVERSE ? policy.reverse_ms : policy.first_ms;
    if ((uint32_t)(now-m->line_since) >= search_ms &&
        ((!bidirectional && m->line_search_state == LINE_SEARCH_IDLE) || !PathLine_Aligned(m, in->gray))) {
        if (bidirectional) {
            if (m->line_search_state == LINE_SWEEP_FINAL && !PATH_WAREHOUSE_UNTIMED(m)) { fail(m,PATH_TIMEOUT); return false; }
            hold(m);
            m->line_search_state = m->line_search_state == LINE_SWEEP_REVERSE ? LINE_BRAKE_FINAL : LINE_BRAKE_REVERSE;
            return false;
        }

    }
    if (PathLine_Aligned(m, in->gray)) {
        if (!m->line_stopping) {
            hold(m); m->line_stopping = true; m->stable = false;
            return false;
        }
        if (!in->settled) { m->stable = false; return false; }
        /* Braking may change gray; only strict 0110 permits admission. */
        if (!PathLine_Aligned(m, in->gray)) return false;
        if (!m->stable) { m->stable = true; m->stable_since = now; }
        if ((uint32_t)(now-m->stable_since) < PATH_LINE_STABLE_MS) return false;
        m->line_active = m->line_stopping = m->stable = false;
        if (m->step==9) m->stair_heading_calibrated=true;
        else if (m->step==12 || m->step==13) m->warehouse_heading_calibrated=true;
        return true;
    }
    m->stable = false;
    if (m->line_stopping) {
        if (!in->settled) return false;
        m->line_stopping = false;
    }
    /* Gray controls lateral position; map search retains the gyro heading loop.
     * Warehouse sweeps right, left across the start, then right again. */
    if (!bidirectional && m->line_search_state == LINE_SEARCH_IDLE) m->line_search_state=LINE_SWEEP_FIRST;
    float direction=m->line_search_state==LINE_SWEEP_REVERSE ? 1.0f : -1.0f;
    (void)emit(m,PC_MAP_SEARCH,0,(m->blue ? -direction : direction)*policy.lateral_mm_s,0,0,policy.motion_timeout_ms);
    return false;
}

