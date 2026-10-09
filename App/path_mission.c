#include "path_config.h"
#include "path_mission.h"
#include "disc_task_config.h"
#include "path_chassis.h"
#define START_BLEND_RADIUS_MM 800.0f
#define START_BLEND_SPEED_RPM 155.0f
#define START_DIAG_X_MM 1558.8922f
#define START_DIAG_Y_MM 567.3904f
#define DISC_ENTRY_RADIUS_MM 50.0f
#define DISC_ENTRY_SPEED_RPM 25.0f
#define START_FORWARD_MM 2028.9384f
#define BLUE_START_FORWARD_EXTRA_MM 195.0f
#define BLUE_START_TURN_SPEED_SCALE 0.9f
/* Approach + G100 + disc; RDK owns G101, vision and five G102 actions. */
static bool emit(PathMission *m, PathCommandKind k, float x, float y, float speed, uint32_t t)
{
    PathCommand c = {.kind = k, .x = x, .y = y, .speed = speed, .timeout_ms = t};
    if (m->send(m->context, &c))
        return true;
    m->result = PATH_ERROR;
    return false;
}
static bool emit_move(PathMission *m, float x, float y, float speed,
                      float start_speed, float end_speed, bool continuous)
{
    PathCommand c = {.kind = PC_MOVE, .x = x, .y = y, .speed = speed,
                     .start_speed = start_speed, .end_speed = end_speed,
                     .continuous = continuous, .timeout_ms = 30000};
    if (m->blue || PATH_BLUE_DISC_TEST) {
        if (m->step==0 && m->part==0) {
            c.kind=PC_MOVE_ROTATE; c.y=-c.y; c.angle=180.0f;
            c.speed*=BLUE_START_TURN_SPEED_SCALE;
            c.end_speed*=BLUE_START_TURN_SPEED_SCALE;
        } else {
            if (m->step==1) c.x+=BLUE_START_FORWARD_EXTRA_MM;
            c.x=-c.x;
        } /* Body is reversed after the opening half-turn. */
    }
    if (m->send(m->context, &c))
        return true;
    m->result = PATH_ERROR;
    return false;
}
static bool emit_arc(PathMission *m)
{
    /* 800 mm circular fillet: tangent to the incoming +20 deg line and outgoing 0 deg line.
     * Tangent offset is R*tan(10 deg)=141.0616 mm, so the final global endpoint is unchanged.
     * Body heading stays fixed: arc starts at +20 deg and keeps the diagonal boundary speed. */
    PathCommand c = {.kind = PC_ARC, .x = START_BLEND_RADIUS_MM, .y = 20.0f, .angle = -20.0f,
                     .speed = 155.0f, .start_speed = START_BLEND_SPEED_RPM,
                     .end_speed = START_BLEND_SPEED_RPM, .continuous = true,
                     .timeout_ms = 30000};
    if (m->blue || PATH_BLUE_DISC_TEST) {
        c.y = 180.0f - c.y; c.angle = -c.angle;
        c.start_speed*=BLUE_START_TURN_SPEED_SCALE; /* Match the opening turn exit. */
    }
    if (m->send(m->context, &c))
        return true;
    m->result = PATH_ERROR;
    return false;
}
static void fail(PathMission *m, PathResult r)
{
    (void)emit(m, PC_HOLD, 0, 0, 0, 0);
    (void)emit(m, PC_CANCEL, 0, 0, 0, 0);
    m->result = r;
}
void Path_Init(PathMission *m, PathSend s, void *c)
{
    *m = (PathMission){.send = s, .context = c};
}
bool Path_Start(PathMission *m, uint32_t now, const PathInput *in)
{
    if (m->result == PATH_RUNNING || !in->armed || in->fault || !in->settled ||
        m->inventory.occupied || m->inventory.uncertain)
        return false;
    bool blue = m->blue || PATH_BLUE_DISC_TEST;
    PathSend s = m->send;
    void *c = m->context;
    *m =
        (PathMission){.blue = blue, .send = s, .context = c, .result = PATH_RUNNING, .phase = 99, .entered = now};
    if (!PATH_VISION_ENABLE)
    {
        m->phase = 0;
        return true;
    }
    return emit(m, PC_HELLO, 0, 0, 0, 2000);
}
void Path_Cancel(PathMission *m)
{
    if (m->result == PATH_RUNNING)
        fail(m, PATH_CANCELED);
}
void Path_Tick(PathMission *m, uint32_t now, const PathInput *in)
{
    if (m->result != PATH_RUNNING)
        return;
    if (!in->armed || in->fault)
    {
        fail(m, PATH_ERROR);
        return;
    }
    if (m->prep_pending) {
        if (in->reply == PATH_FAILED || (uint32_t)(now-m->prep_since)>=(m->pillar_depart_pending ? 5000U : 30000U)) {
            fail(m,in->reply==PATH_FAILED?PATH_ERROR:PATH_TIMEOUT); return;
        }
        if (in->reply == PATH_OK) {
            if (m->disc_depart_pending) {
                PathCommand c = {.kind=PC_GROUP,.argument=1,.timeout_ms=30000};
                if (!m->send(m->context,&c)) { fail(m,PATH_ERROR); return; }
                m->disc_depart_pending=false;
                m->prep_since=now;
            } else { m->prep_pending=false; m->pillar_depart_pending=false; }
        }
    }
    if (m->step >= 4)
    {
        PathChassis_Tick(m, now, in);
        return;
    }
    uint32_t limit = m->step == 3 ? (m->phase == 0 ? PATH_DISC_LINE_TIMEOUT_MS :
                                    m->phase == 1 ? DISC_TASK_TIMEOUT_MS : 30000U) : 30000U;
    if ((uint32_t)(now - m->entered) >= limit)
    {
        fail(m, PATH_TIMEOUT);
        return;
    }
    if (m->phase == 99)
    {
        if (in->reply == PATH_FAILED)
            fail(m, PATH_ERROR);
        else if (in->reply == PATH_OK)
        {
            PathCommand prep = {.kind=PC_GROUP,.argument=100,.timeout_ms=30000};
            if (!m->send(m->context,&prep)) { fail(m,PATH_ERROR); return; }
            m->prep_pending=true; m->prep_since=now;
            m->phase = 0;
            m->entered = now;
        }
        return;
    }
    /* Fillet into +Y line search without stopping at the straight endpoint. */
    if (m->step == 1 && m->part == 1) {
        if ((in->gray & 6U) == 6U || in->motion_done) {
            bool detected = (in->gray & 6U) == 6U;
            if (detected && !emit(m, PC_HOLD, 0, 0, 0, 0)) return;
            m->step=3; m->phase=2; m->part=0; m->waiting=false;
            m->stable=detected; m->entered=now;
        }
        return;
    }
    if (m->step < 3)
    {
        if (!m->waiting)
        {
            if (m->step == 0 && m->part == 0)
                m->waiting = emit_move(m, START_DIAG_X_MM, START_DIAG_Y_MM, 155.0f,
                                       0, START_BLEND_SPEED_RPM, true);
            else if (m->step == 0)
                m->waiting = emit_arc(m);
            else if (m->step == 1)
                m->waiting = emit_move(m, START_FORWARD_MM, 0, 230.0f,
                                       START_BLEND_SPEED_RPM, DISC_ENTRY_SPEED_RPM, true);
            else
            {
                m->step = 3; /* No startup body rotation. */
                m->phase = 2;
                m->entered = now;
            }
        }
        else if (m->step == 0 && in->motion_done)
        {
            if (m->part == 0)
            {
                m->part = 1;
                m->waiting = emit_arc(m);
            }
            else
            {
                m->step = 1;
                m->part = 0;
                m->waiting = emit_move(m, START_FORWARD_MM, 0, 230.0f,
                                       START_BLEND_SPEED_RPM, DISC_ENTRY_SPEED_RPM, true);
            }
            m->entered = now;
        }
        else if (m->step == 1 && in->motion_done)
        {
            PathCommand c = {.kind=PC_ARC,.x=DISC_ENTRY_RADIUS_MM,.y=0,.angle=90,
                             .speed=DISC_ENTRY_SPEED_RPM,.start_speed=DISC_ENTRY_SPEED_RPM,
                             .end_speed=DISC_ENTRY_SPEED_RPM,.continuous=true,.timeout_ms=30000};
            if (m->blue || PATH_BLUE_DISC_TEST) { c.y=180.0f-c.y; c.angle=-c.angle; }
            if (!m->send(m->context,&c)) { fail(m,PATH_ERROR); return; }
            m->part=1; m->entered=now;
        }
        else if (m->step != 0 && in->settled)
        {
            m->step = 3; /* Skip the former standalone 180-degree turn. */
            m->waiting = false;
            m->entered = now;
            if (m->step == 3) m->phase = 2; /* Enter disc line alignment. */
        }
        return;
    }
    if (m->phase == 2)
    {
        /* G100 was issued at departure; do not repeat it at the line. */
        m->waiting=false; m->phase=0; m->entered=now;
        return;
    }
    if (m->phase == 0)
    {
        if (PathLine_Align(m, now, in, PATH_DISC_LINE_TIMEOUT_MS, 25))
        {
            if (!PATH_VISION_ENABLE) { m->result=PATH_DONE; return; }
            m->phase=3; m->entered=now; /* Line reached; serialize the next arm/vision request. */
        }
    }
    else if (m->phase == 3) {
        if (!m->prep_pending && in->settled) {
            m->phase=1; m->entered=now;
            if (!emit(m,PC_DISC,0,0,0,DISC_TASK_TIMEOUT_MS)) fail(m,PATH_ERROR);
        }
    }
    else
    {
        if (!in->settled || in->reply == PATH_FAILED)
            fail(m, PATH_ERROR);
        else if (m->id_count >= DISC_REQUIRED_RFID_COUNT || in->reply == PATH_OK)
        {
            if (in->disc_completed != DISC_REQUIRED_RFID_COUNT &&
                (PATH_SKIP_MATERIAL(m) || m->id_count < DISC_REQUIRED_RFID_COUNT))
                fail(m, PATH_ERROR);
            else
            {
                (void)emit(m, PC_HOLD, 0, 0, 0, 0);
                m->disc_depart_pending = in->reply != PATH_OK;
                m->prep_pending = m->disc_depart_pending;
                m->prep_since = now;
                m->result = PATH_DONE;
            }
        }
    }
}

void Path_RecordId(PathMission *m, uint32_t id)
{
    if (!PATH_VISION_ENABLE) return;
    if (m->result != PATH_RUNNING ||
        !((m->step == 3 && m->phase == 1) || (m->step == 6 && m->phase == 6) ||
          (m->step == 9 && m->phase == 24 && m->stair_scanning && m->grabs < 2)))
        return;
    for (unsigned i = 0; i < m->id_count; ++i)
        if (m->id_list[i] == id) return;
    if (m->id_count == sizeof(m->id_list) / sizeof(m->id_list[0]))
    {
        m->id_overflow = true;
        return;
    }
    m->id_list[m->id_count++] = id;
}
