#include "path_mission.h"
#include "disc_task_config.h"
#include "path_chassis.h"
/* Disc only; RDK owns G101, vision and five G102 actions. */
static bool emit(PathMission *m, PathCommandKind k, float x, float y, float speed, uint32_t t)
{
    PathCommand c = {.kind = k, .x = x, .y = y, .speed = speed, .timeout_ms = t};
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
    if (m->result == PATH_RUNNING || !in->armed || in->fault || !in->settled)
        return false;
    PathSend s = m->send;
    void *c = m->context;
    *m =
        (PathMission){.send = s, .context = c, .result = PATH_RUNNING, .phase = 99, .entered = now};
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
    uint32_t limit = m->step == 3 ? (m->phase == 0 ? 5000U : DISC_TASK_TIMEOUT_MS) : 30000U;
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
            if (m->step == 0)
                m->waiting = emit(m, PC_MOVE, 1691.4467f, 615.6363f, 85, 30000);
            else if (m->step == 1)
                m->waiting = emit(m, PC_MOVE, 2200, 0, 130, 30000);
            else
                m->waiting = emit(m, PC_ROTATE, 180, 0, 0, 15000);
        }
        else if (in->settled)
        {
            m->step++;
            m->waiting = false;
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
            (void)emit(m, PC_HOLD, 0, 0, 0, 0);
            m->result = PATH_DONE;
        }
    }
}

void Path_RecordId(PathMission *m, uint32_t id)
{
    if (m->result != PATH_RUNNING || m->step != 3 || m->phase != 1)
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
