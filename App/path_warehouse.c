#include "path_warehouse.h"
#include "path_chassis.h"
/* Compare all 6^3 row orders once, including the transitions between columns.
 * Cache pocket lookups before enumeration; do not plan again after unloading. */
static void plan_warehouse(PathMission *m)
{
    static const uint8_t orders[6][3]={{1,2,3},{1,3,2},{2,1,3},{2,3,1},{3,1,2},{3,2,1}};
    int slots[9];
    for(unsigned col=0;col<3;col++)
        for(unsigned row=0;row<3;row++)
            slots[col*3+row]=BallInventory_Find(&m->inventory,(uint8_t)(((row+1)<<4)|(col+1)));
    unsigned best=~0U, chosen=0;
    for(unsigned candidate=0;candidate<216;candidate++) {
        unsigned current=m->inventory.current, cost=0, order=candidate;
        for(unsigned col=0;col<3;col++,order/=6) {
            if(col<m->point/3) continue;
            for(unsigned i=0;i<3;i++) {
                int slot=slots[col*3+orders[order%6][i]-1];
                if(slot<0) continue;
                unsigned forward=((unsigned)slot+BALL_SLOT_COUNT-current)%BALL_SLOT_COUNT;
                unsigned reverse=(current+BALL_SLOT_COUNT-(unsigned)slot)%BALL_SLOT_COUNT;
                cost+=forward<reverse?forward:reverse;
                current=(unsigned)slot;
            }
        }
        if(cost<best) { best=cost; chosen=candidate; }
    }
    for(unsigned col=0;col<3;col++,chosen/=6)
        for(unsigned i=0;i<3;i++) m->warehouse_order[col*3+i]=orders[chosen%6][i];
    m->warehouse_plan_ready=true;
}
uint8_t PathWarehouse_Code(const PathMission *m)
{
    if(m->point>=9) return 0;
    unsigned col=m->point/3+1;
    unsigned row=m->warehouse_plan_ready ? m->warehouse_order[m->point] : m->point%3+1;
    return (uint8_t)((row<<4)|col);
}
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
        else if (emit(m,(PathCommand){.kind=PC_HOLD})) m->phase=6;
    }
}
void PathWarehouse_Tick(PathMission *m, uint32_t now, const PathInput *in)
{
    if (m->point==9 && m->phase==6 && !m->inventory.occupied && !m->inventory.uncertain) {
        if (!m->waiting) {
            if (!in->settled) return;
            m->entered=now;
            m->waiting=emit(m,(PathCommand){.kind=PC_HOME_ALIGN,.timeout_ms=15000});
        } else if ((uint32_t)(now-m->entered)>=15000) fail(m,PATH_TIMEOUT);
        else if (in->settled) { m->waiting=false; m->phase=5; m->entered=now; }
        return;
    }
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
    if(!m->warehouse_plan_ready) plan_warehouse(m);
    uint8_t code=PathWarehouse_Code(m);
    if (m->prep_pending && (m->phase==1 || m->phase==3)) return;
    switch(m->phase)
    {
    case 0: /* Start at the detected line; only subsequent columns require a move. */
        if (m->point==0 && !m->waiting) {
            if (!in->settled) break;
            m->phase=4;
            m->line_active=m->stable=false;
            m->entered=now;
            break;
        }
        if (!m->waiting)
        {
            m->entered=now;
            m->waiting=emit(m,(PathCommand){.kind=PC_MOVE,.x=200,
                                          .speed=100,.acceleration=750,.deceleration=750,.timeout_ms=30000});
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
        if (!PathHeading_Ready(m,now,in)) break;
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
                     .x=BallInventory_ReverseTo(m->inventory.current,(uint8_t)slot)
                         ? (m->inventory.current+BALL_SLOT_COUNT-slot)%BALL_SLOT_COUNT
                         : (slot+BALL_SLOT_COUNT-m->inventory.current)%BALL_SLOT_COUNT,
                     .argument=BallInventory_ReverseTo(m->inventory.current,(uint8_t)slot),
                     .timeout_ms=3000}))
        {
            m->phase=2;
            m->entered=now;
        }
        break;
    }
    case 2: /* Adapter updates the current slot only on successful turn completion. */
        if (!in->settled || in->turn_reply==PATH_FAILED) fail(m,PATH_ERROR);
        else if ((uint32_t)(now-m->entered)>=3000) fail(m,PATH_TIMEOUT);
        else if (in->turn_reply==PATH_OK) m->phase=1;
        break;
    case 3:
        if (!m->waiting && !PathHeading_Ready(m,now,in)) break;
        if (!m->waiting && !m->line_skipped && !PathLine_Aligned(in->gray)) {
            m->line_active=false;
            m->phase=4;
            break;
        }
        if (!in->settled) { fail(m,PATH_ERROR); break; }
        if (!m->waiting)
        {
            m->entered=now;
            m->waiting=emit(m,(PathCommand){.kind=PC_GROUP,.argument=108+(code>>4),.timeout_ms=30000});
        }
        else if (in->reply==PATH_FAILED) fail(m,PATH_ERROR);
        else if ((uint32_t)(now-m->entered)>=30000) fail(m,PATH_TIMEOUT);
        else if (in->reply==PATH_OK)
        {
            if (!BallInventory_Unload(&m->inventory,code)) fail(m,PATH_ERROR);
            else { m->waiting=false; m->phase=7; }
        }
        break;
    case 7: /* Arm completion acknowledged; now it is safe to correct yaw. */
        if (PathHeading_Ready(m,now,in)) advance(m,now);
        break;
    default:
        fail(m,PATH_ERROR);
        break;
    }
}
