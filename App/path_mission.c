#include "path_config.h"
#include "path_mission.h"
#include "disc_task_config.h"
#include "path_chassis.h"
#define START_BLEND_RADIUS_MM 800.0f
#define START_BLEND_SPEED_RPM 60.0f
#define START_DIAG_X_MM 1558.8922f
#define START_DIAG_Y_MM 567.3904f
#define START_FORWARD_MM 2008.9384f
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
    if (m->send(m->context, &c))
        return true;
    m->result = PATH_ERROR;
    return false;
}
static bool emit_arc(PathMission *m)
{
    /* 800 mm circular fillet: tangent to the incoming +20 deg line and outgoing 0 deg line.
     * Tangent offset is R*tan(10 deg)=141.0616 mm, so the final global endpoint is unchanged. */
    PathCommand c = {.kind = PC_ARC, .x = START_BLEND_RADIUS_MM, .y = 20.0f, .angle = -20.0f,
                     .speed = 85.0f, .start_speed = START_BLEND_SPEED_RPM,
                     .end_speed = START_BLEND_SPEED_RPM, .continuous = true,
                     .timeout_ms = 30000};
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
    PathSend s = m->send;
    void *c = m->context;
    *m =
        (PathMission){.send = s, .context = c, .result = PATH_RUNNING, .phase = 99, .entered = now};
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
    if (m->step >= 4)
    {
        PathChassis_Tick(m, now, in);
        return;
    }
    uint32_t limit = m->step == 3 ? (m->phase == 0 ? 5000U :
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
            m->phase = 0;
            m->entered = now;
        }
        return;
    }
    if (m->step < 3)
    {
        if (!m->waiting)
        {
            if (m->step == 0 && m->part == 0)
                m->waiting = emit_move(m, START_DIAG_X_MM, START_DIAG_Y_MM, 85.0f, 0,
                                       START_BLEND_SPEED_RPM, true);
            else if (m->step == 0)
                m->waiting = emit_arc(m);
            else if (m->step == 1)
                m->waiting = emit_move(m, START_FORWARD_MM, 0, 130.0f,
                                       START_BLEND_SPEED_RPM, 0, false);
            else
                m->waiting = emit(m, PC_ROTATE, 180.0f, 0, 0, 15000);
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
                m->waiting = emit_move(m, START_FORWARD_MM, 0, 130.0f,
                                       START_BLEND_SPEED_RPM, 0, false);
            }
            m->entered = now;
        }
        else if (m->step != 0 && in->settled)
        {
            m->step++;
            m->waiting = false;
            m->entered = now;
            if (m->step == 3) m->phase = 2; /* G100 before disc alignment/start. */
        }
        return;
    }
    if (m->phase == 2)
    {
        if (!PATH_VISION_ENABLE)
        {
            m->phase = 0;
            m->entered = now;
            return;
        }
        if (!m->waiting)
        {
            PathCommand c = {.kind = PC_GROUP, .argument = 100, .timeout_ms = 30000};
            if (!(m->waiting = m->send(m->context, &c))) fail(m, PATH_ERROR);
        }
        else if (in->reply == PATH_FAILED) fail(m, PATH_ERROR);
        else if (in->reply == PATH_OK)
        {
            m->waiting = false;
            m->phase = 0;
            m->entered = now;
        }
        return;
    }
    if (m->phase == 0)
    {
        if ((in->gray & 6U) == 6U)
        {
            if (!m->stable)
            {
                m->stable = true;
                m->stable_since = now;
                (void)emit(m, PC_HOLD, 0, 0, 0, 0);
            }
            if ((uint32_t)(now - m->stable_since) >= 50 && in->settled)
            {
                if (!PATH_VISION_ENABLE)
                {
                    m->result = PATH_DONE;
                    return;
                }
                m->phase = 1;
                m->entered = now;
                if (!emit(m, PC_DISC, 0, 0, 0, DISC_TASK_TIMEOUT_MS))
                    fail(m, PATH_ERROR);
            }
        }
        else
        {
            m->stable = false;
            (void)emit(m, PC_BODY, 0, -25, 0, 0);
        }
    }
    else
    {
        if (!in->settled || in->reply == PATH_FAILED)
            fail(m, PATH_ERROR);
        else if (in->reply == PATH_OK)
        {
            if (m->id_count < DISC_REQUIRED_RFID_COUNT)
                fail(m, PATH_ERROR);
            else
            {
                (void)emit(m, PC_HOLD, 0, 0, 0, 0);
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
          (m->step == 9 && m->phase == 2 && m->waiting && m->grabs < 2)))
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
