#include "path_warehouse.h"
#include "path_chassis.h"
static void fail(PathMission *m, PathResult result)
{
    if (m->inventory.occupied || m->phase == 2) m->inventory.uncertain = true;
    PathCommand hold = {.kind=PC_HOLD}, cancel = {.kind=PC_CANCEL};
    (void)m->send(m->context,&hold);
    (void)m->send(m->context,&cancel);
    m->result=result;
}
static bool emit(PathMission *m, PathCommand command)
{
    if (m->send(m->context,&command)) return true;
    fail(m,PATH_ERROR);
    return false;
}
static void advance(PathMission *m, uint32_t now)
{
    ++m->point;
    m->waiting=false;
    m->entered=now;
    m->phase=m->point%3 == 0 ? 0 : 1;
    if (m->point==9)
    {
        if (m->inventory.occupied) fail(m,PATH_ERROR);
        else if (emit(m,(PathCommand){.kind=PC_HOLD})) m->phase=5;
    }
}
void PathWarehouse_Tick(PathMission *m, uint32_t now, const PathInput *in)
{
    if (m->point==9 && m->phase==5 && !m->inventory.occupied && !m->inventory.uncertain) {
        if (!m->waiting) {
            if (!in->settled) return;
            m->entered=now;
            m->waiting=emit(m,(PathCommand){.kind=PC_RETURN_HOME,.timeout_ms=90000});
        } else if ((uint32_t)(now-m->entered)>=90000) fail(m,PATH_TIMEOUT);
        else if (in->settled) {
            if (emit(m,(PathCommand){.kind=PC_HOLD})) m->result=PATH_DONE;
        }
        return;
    }
    if (m->inventory.uncertain || m->point>=9)
    {
        fail(m,PATH_ERROR);
        return;
    }
    uint8_t code=(uint8_t)(((m->point%3+1)<<4) | (m->point/3+1));
    switch(m->phase)
    {
    case 0: /* First approach: 100 mm; subsequent column changes: 200 mm. */
        if (!m->waiting)
        {
            m->entered=now;
            m->waiting=emit(m,(PathCommand){.kind=PC_MOVE,.x=m->point==0 ? 100 : 200,
                                          .speed=40,.timeout_ms=30000});
        }
        else if ((uint32_t)(now-m->entered)>=30000) fail(m,PATH_TIMEOUT);
        else if (in->settled)
        {
            m->waiting=false;
            m->phase=4;
            m->line_active=false;
            m->stable=false;
            m->entered=now;
        }
        break;
    case 4: /* Every chassis move must finish with four-probe alignment. */
        if (PathLine_AlignFour(m,now,in)) {
            m->phase=1;
            m->entered=now;
        } else if (m->result!=PATH_RUNNING && m->inventory.occupied) {
            m->inventory.uncertain=true;
        }
        break;
    case 1:
    {
        if (!in->settled) { fail(m,PATH_ERROR); break; }
        int slot=BallInventory_Find(&m->inventory,code);
        if (slot<0) advance(m,now); /* Missing ball: no arm action, still visit all columns. */
        else if (slot==m->inventory.current)
        {
            m->phase=3;
            m->waiting=false;
            m->entered=now;
        }
        else if (emit(m,(PathCommand){.kind=PC_TURN,
                     .argument=BallInventory_ReverseTo(m->inventory.current,(uint8_t)slot),
                     .timeout_ms=2000}))
        {
            m->phase=2;
            m->entered=now;
        }
        break;
    }
    case 2: /* Adapter updates the current slot only on successful turn completion. */
        if (!in->settled || in->turn_reply==PATH_FAILED) fail(m,PATH_ERROR);
        else if ((uint32_t)(now-m->entered)>=2000) fail(m,PATH_TIMEOUT);
        else if (in->turn_reply==PATH_OK) m->phase=1;
        break;
    case 3:
        if (!in->settled) { fail(m,PATH_ERROR); break; }
        if (!m->waiting)
        {
            m->entered=now;
            m->waiting=emit(m,(PathCommand){.kind=PC_GROUP,.argument=109+m->point%3,.timeout_ms=30000});
        }
        else if (in->reply==PATH_FAILED) fail(m,PATH_ERROR);
        else if ((uint32_t)(now-m->entered)>=30000) fail(m,PATH_TIMEOUT);
        else if (in->reply==PATH_OK)
        {
            if (!BallInventory_Unload(&m->inventory,code)) fail(m,PATH_ERROR);
            else advance(m,now);
        }
        break;
    default:
        fail(m,PATH_ERROR);
        break;
    }
}
