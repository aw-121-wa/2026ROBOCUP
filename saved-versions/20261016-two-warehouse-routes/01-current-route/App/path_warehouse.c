#include "path_warehouse.h"
#include "path_chassis.h"
#include "path_config.h"
#include <math.h>
/* Compare all 6^3 row orders once, including the transitions between columns.
 * Cache pocket lookups before enumeration; replan when a new digit changes the mapping. */
static unsigned column_code(const PathMission *m,unsigned col)
{
    return m->warehouse_mode==WAREHOUSE_DIGIT_ORDER ? m->warehouse_columns[col] : col+1;
}
static void plan_warehouse(PathMission *m)
{
    (void)BallInventory_AssignMissing(&m->inventory);
    static const uint8_t orders[6][3]={{1,2,3},{1,3,2},{2,1,3},{2,3,1},{3,1,2},{3,2,1}};
    int slots[9];
    for(unsigned col=0;col<3;col++)
        for(unsigned row=0;row<3;row++)
            slots[col*3+row]=BallInventory_Find(&m->inventory,(uint8_t)(((row+1)<<4)|column_code(m,col)));
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
    unsigned col=column_code(m,m->point/3);
    unsigned row=m->warehouse_plan_ready ? m->warehouse_order[m->point] : m->point%3+1;
    return (uint8_t)((row<<4)|col);
}
static void fail(PathMission *m, PathResult result)
{
    if (m->inventory.occupied || m->phase == WAREHOUSE_TURN) m->inventory.uncertain = true;
    PathCommand hold = {.kind=PC_HOLD}, cancel = {.kind=PC_CANCEL};
    (void)m->send(m->context,&hold);
    (void)m->send(m->context,&cancel);
    m->result=result;
}
static bool emit(PathMission *m, PathCommand command)
{
    /* A recognized blue digit prohibits lateral search; unloading remains gyro-controlled. */
    if (m->blue && m->point<9 && m->warehouse_mode==WAREHOUSE_DIGIT_ORDER &&
        (command.kind==PC_MAP_SEARCH || command.kind==PC_MAP_LATERAL)) {
        fail(m,PATH_ERROR);return false;
    }
    if (m->blue && command.kind==PC_MAP_LATERAL) command.y=-command.y;
    if (m->send(m->context,&command)) return true;
    fail(m,PATH_ERROR);
    return false;
}
static void start_scan(PathMission *m,uint32_t now)
{
    m->warehouse_mode=WAREHOUSE_DIGIT_ORDER;
    m->warehouse_query=emit(m,(PathCommand){.kind=PC_WAREHOUSE_DIGIT,
        .argument=m->warehouse_used,.timeout_ms=PATH_WAREHOUSE_DIGIT_TIMEOUT_MS});
    if (m->warehouse_query) {m->phase=WAREHOUSE_WAIT_CAMERA;m->entered=now;}
}
static void fixed_move(PathMission *m,uint32_t now,float distance)
{
    if (emit(m,(PathCommand){.kind=PC_MOVE,.x=distance,
        .speed=m->phase==WAREHOUSE_BRAKE ? PATH_WAREHOUSE_CREEP_SPEED_RPM : 120,
        .acceleration=850,.deceleration=850,.timeout_ms=30000})) {
        m->phase=WAREHOUSE_INFERRED_MOVE;m->entered=now;
    }
}
static bool valid_digit(const PathInput *in)
{
    return in->warehouse_digit_reply==PATH_OK &&
           in->warehouse_digit>=1 && in->warehouse_digit<=3;
}
static void remember_digit(PathMission *m, uint8_t digit)
{
    m->warehouse_columns[m->point/3]=digit;
    m->warehouse_plan_ready=false;
}
static void advance(PathMission *m, uint32_t now)
{
    if (m->point%3==2 && m->warehouse_mode==WAREHOUSE_DIGIT_ORDER)
        m->warehouse_used|=(uint8_t)(1U<<column_code(m,m->point/3));
    ++m->point;
    m->waiting=false;
    m->entered=now;
    m->phase=m->point%3 == 0 ? WAREHOUSE_MOVE : WAREHOUSE_SELECT_BALL;
    if (m->point==9)
    {
        if (BallInventory_HasKnown(&m->inventory)) fail(m,PATH_ERROR);
        else if (emit(m,(PathCommand){.kind=PC_HOLD})) {m->phase=WAREHOUSE_EXIT_RIGHT;m->stable=false;}
    }
}
void PathWarehouse_Tick(PathMission *m, uint32_t now, const PathInput *in)
{
    if (m->point==9 && !BallInventory_HasKnown(&m->inventory) && !m->inventory.uncertain) {
        if (m->phase==WAREHOUSE_EXIT_RIGHT) {
            if (!m->waiting) {
                if (in->settled) m->waiting=emit(m,(PathCommand){.kind=PC_MOVE,.y=-10,
                    .speed=30,.acceleration=650,.deceleration=650,.timeout_ms=30000});
            } else if (in->settled) {
                m->phase=WAREHOUSE_ALIGN_HOME;m->waiting=false;m->stable=false;m->entered=now;
            }
        } else if (m->phase==WAREHOUSE_ALIGN_HOME) {
            if (!m->stable) {
                unsigned middle=in->gray&6U;
                if (middle==2U || middle==4U) {
                    if (emit(m,(PathCommand){.kind=PC_HOLD})) m->stable=true;
                } else (void)emit(m,(PathCommand){.kind=PC_BODY,.x=m->blue ? 20 : -20,.timeout_ms=10000});
            } else if (in->settled) {
                m->phase=WAREHOUSE_RETURN_HOME;m->waiting=false;m->entered=now;
            }
        } else if (m->phase==WAREHOUSE_RETURN_HOME || m->phase==WAREHOUSE_HOME_RIGHT) {
            unsigned leg=m->phase==WAREHOUSE_RETURN_HOME ? 0 : 1;
            if (!m->waiting) {
                if(in->settled) m->waiting=emit(m,(PathCommand){.kind=PC_RETURN_HOME,.argument=leg,.timeout_ms=30000});
            } else if(in->settled) {
                m->phase=leg==0 ? WAREHOUSE_HOME_RIGHT : WAREHOUSE_HOME_FINAL_ALIGN;
                m->waiting=false;m->entered=now;
            }
        } else if (m->phase==WAREHOUSE_HOME_FINAL_ALIGN) {
            float target=m->blue ? 180.0f : 0.0f;
            float error=remainderf(target-in->map_yaw_deg,360.0f);
            if (!isfinite(error)) {fail(m,PATH_ERROR);return;}
            if (!in->settled) return;
            if (fabsf(error)<0.1f) m->result=PATH_DONE;
            else m->waiting=emit(m,(PathCommand){.kind=PC_HOME_ALIGN,.x=target,.timeout_ms=PATH_HEADING_TIMEOUT_MS});
        } else {fail(m,PATH_ERROR);return;}
        if (m->result==PATH_RUNNING && !PATH_WAREHOUSE_UNTIMED(m) &&
            (uint32_t)(now-m->entered)>=(m->phase==WAREHOUSE_HOME_FINAL_ALIGN ? PATH_HEADING_TIMEOUT_MS : 30000U))
            fail(m,PATH_TIMEOUT);
        return;
    }
    if (m->inventory.uncertain || m->point>=9)
    {
        fail(m,PATH_ERROR);
        return;
    }
    if(!m->warehouse_plan_ready) plan_warehouse(m);
    uint8_t code=PathWarehouse_Code(m);
    if (m->prep_pending && (m->phase==WAREHOUSE_SELECT_BALL || m->phase==WAREHOUSE_UNLOAD ||
        (m->phase==WAREHOUSE_MOVE && m->point==0 && in->warehouse_vision))) return;
    switch(m->phase)
    {
    case WAREHOUSE_ENTRY_BACK:
        if (!m->stable) {
            unsigned middle=in->gray&6U;
            if (middle==2U || middle==4U) {
                if (emit(m,(PathCommand){.kind=PC_HOLD})) m->stable=true;
            } else (void)emit(m,(PathCommand){.kind=PC_BODY,.x=-20,.timeout_ms=5000});
        } else if (in->settled) {
            {
                m->warehouse_mode=WAREHOUSE_DEFAULT_ORDER;
                m->point=0;m->warehouse_plan_ready=false;
                fixed_move(m,now,m->blue ? PATH_WAREHOUSE_COLUMN_SPACING_MM : PATH_RED_WAREHOUSE_FIRST_MM);
                break;
            }
            m->warehouse_pass=0;
            m->warehouse_edge_armed=(in->gray&4U)!=0;
            start_scan(m,now);
        }
        break;
    case WAREHOUSE_WAIT_CAMERA:
        if (in->warehouse_ready || in->warehouse_digit_reply!=PATH_WAIT)
            m->phase=WAREHOUSE_FIRST_DIGIT;
        break;
    case WAREHOUSE_FIRST_DIGIT: {
        float scanned=(in->x_mm-m->warehouse_first_x)*cosf(m->warehouse_first_axis)+
                      (in->y_mm-m->warehouse_first_y)*sinf(m->warehouse_first_axis);
        if (m->warehouse_first_known && m->point==3 &&
            scanned>=PATH_WAREHOUSE_MISSING_SCAN_MM &&
            !(valid_digit(in) && !(m->warehouse_used&(1U<<in->warehouse_digit)))) {
            if (!emit(m,(PathCommand){.kind=PC_HOLD}) ||
                !emit(m,(PathCommand){.kind=PC_WAREHOUSE_STOP})) break;
            m->warehouse_fallback=true;m->phase=WAREHOUSE_SCAN_STOP;m->warehouse_query=false;
            break;
        }
        unsigned sensor=m->warehouse_pass ? 2U : 4U;
        if (in->gray&sensor) m->warehouse_edge_armed=true;
        /* Only an ON-to-OFF transition ends a pass, not an OFF entry reading. */
        if (m->warehouse_edge_armed && !(in->gray&sensor)) {
            if (!emit(m,(PathCommand){.kind=PC_HOLD})) break;
            if (!emit(m,(PathCommand){.kind=PC_WAREHOUSE_STOP})) break;
            m->warehouse_fallback=m->warehouse_first_known && m->point==3;
            m->phase=WAREHOUSE_SCAN_STOP;m->warehouse_query=false;
            break;
        }
        if (valid_digit(in) && !(m->warehouse_used&(1U<<in->warehouse_digit))) {
            if (!isfinite(in->x_mm) || !isfinite(in->y_mm) || !isfinite(in->yaw_deg)) {
                fail(m,PATH_ERROR);break;
            }
            m->warehouse_detect_x=in->x_mm;m->warehouse_detect_y=in->y_mm;
            m->warehouse_detect_axis=in->yaw_deg*0.01745329252f;
            if (!m->warehouse_first_known && m->point==0 && !m->warehouse_pass) {
                m->warehouse_first_known=true;
                m->warehouse_first_x=in->x_mm; m->warehouse_first_y=in->y_mm;
                m->warehouse_first_axis=m->warehouse_detect_axis;
            }
            remember_digit(m,in->warehouse_digit);
            m->warehouse_query=false;
            if (emit(m,(PathCommand){.kind=PC_HOLD})) m->phase=WAREHOUSE_BRAKE;
        } else if (in->warehouse_digit_reply!=PATH_WAIT) {
            if (emit(m,(PathCommand){.kind=PC_HOLD})) m->phase=WAREHOUSE_SCAN_RETRY;
        } else {
            (void)emit(m,(PathCommand){.kind=PC_BODY,
                .x=(m->warehouse_pass ? -1 : 1)*PATH_WAREHOUSE_CREEP_SPEED_RPM,
                .timeout_ms=PATH_WAREHOUSE_CREEP_TIMEOUT_MS});
        }
        break;
    }
    case WAREHOUSE_SCAN_RETRY:
        if (in->settled) start_scan(m,now);
        break;
    case WAREHOUSE_SCAN_STOP:
        if (!in->settled || in->warehouse_digit_reply==PATH_WAIT) break;
        if (m->warehouse_fallback) {
            float dx=m->warehouse_first_x-in->x_mm+PATH_WAREHOUSE_DIGIT_ADVANCE_MM*cosf(m->warehouse_first_axis);
            float dy=m->warehouse_first_y-in->y_mm+PATH_WAREHOUSE_DIGIT_ADVANCE_MM*sinf(m->warehouse_first_axis);
            float yaw=in->yaw_deg*0.01745329252f;
            if (!isfinite(dx) || !isfinite(dy) || !isfinite(yaw)) {fail(m,PATH_ERROR);break;}
            if (emit(m,(PathCommand){.kind=PC_MOVE,.x=dx*cosf(yaw)+dy*sinf(yaw),
                .y=-dx*sinf(yaw)+dy*cosf(yaw),.speed=60,.timeout_ms=30000})) {
                m->warehouse_mode=WAREHOUSE_DEFAULT_ORDER;m->point=0;
                m->warehouse_plan_ready=false;m->phase=WAREHOUSE_INFERRED_MOVE;
            }
        } else if (!m->warehouse_pass) {
            m->warehouse_pass=1;
            m->warehouse_edge_armed=(in->gray&2U)!=0;
            start_scan(m,now);
        } else {
            /* Rear-off is the entrance reference. Inventory retains already unloaded balls. */
            m->warehouse_mode=WAREHOUSE_DEFAULT_ORDER;
            m->point=0;m->warehouse_plan_ready=false;
            fixed_move(m,now,m->blue ? PATH_WAREHOUSE_COLUMN_SPACING_MM : PATH_RED_WAREHOUSE_FIRST_MM);
        }
        break;
    case WAREHOUSE_BRAKE:
        if (in->settled) {
            float travelled=(in->x_mm-m->warehouse_detect_x)*cosf(m->warehouse_detect_axis)+
                (in->y_mm-m->warehouse_detect_y)*sinf(m->warehouse_detect_axis);
            float remaining=PATH_WAREHOUSE_DIGIT_ADVANCE_MM-travelled;
            if (!isfinite(remaining)) {fail(m,PATH_ERROR);break;}
            if (fabsf(remaining)<0.5f) m->phase=WAREHOUSE_SELECT_BALL;
            else fixed_move(m,now,remaining);
        }
        break;
    case WAREHOUSE_INFERRED_MOVE:
        if (in->settled) {m->phase=WAREHOUSE_SELECT_BALL;m->entered=now;}
        break;
    case WAREHOUSE_MOVE:
        if (!in->settled || !PathHeading_Ready(m,now,in)) break;
        if (m->point==0 && m->warehouse_mode==WAREHOUSE_UNDECIDED) {
            m->stable=false;m->phase=WAREHOUSE_ENTRY_BACK;m->entered=now;
        } else if (m->warehouse_mode==WAREHOUSE_DIGIT_ORDER) start_scan(m,now);
        else fixed_move(m,now,PATH_WAREHOUSE_COLUMN_SPACING_MM);
        break;
    case WAREHOUSE_SELECT_BALL:
    {
        if (PATH_BLUE_WAREHOUSE_TEST || PATH_SKIP_MATERIAL(m)) {
            if (!in->settled) { fail(m,PATH_ERROR); break; }
            m->point=(uint8_t)((m->point/3)*3+2);
            advance(m,now);
            break;
        }
        /* Column alignment is complete; do not rotate between balls. */
        if (!in->settled) { fail(m,PATH_ERROR); break; }
        int slot=BallInventory_Find(&m->inventory,code);
        if (slot<0) advance(m,now); /* Missing ball: no arm action, still visit all columns. */
        else if (slot==m->inventory.current)
        {
            m->phase=WAREHOUSE_UNLOAD;
            m->waiting=false;
            m->entered=now;
        }
        else {
            bool reverse=BallInventory_ReverseTo(m->inventory.current,(uint8_t)slot);
            if (emit(m,(PathCommand){.kind=PC_TURN,
                     .x=reverse
                         ? (m->inventory.current+BALL_SLOT_COUNT-slot)%BALL_SLOT_COUNT
                         : (slot+BALL_SLOT_COUNT-m->inventory.current)%BALL_SLOT_COUNT,
                     .argument=reverse,
                     .timeout_ms=3000}))
            {
                m->phase=WAREHOUSE_TURN;
                m->entered=now;
            }
        }
        break;
    }
    case WAREHOUSE_TURN: /* Adapter updates the current slot only on successful turn completion. */
        if (!in->settled || in->turn_reply==PATH_FAILED) fail(m,PATH_ERROR);
        else if (!PATH_WAREHOUSE_UNTIMED(m) && (uint32_t)(now-m->entered)>=3000) fail(m,PATH_TIMEOUT);
        else if (in->turn_reply==PATH_OK) m->phase=WAREHOUSE_SELECT_BALL;
        break;
    case WAREHOUSE_UNLOAD:
        if (!in->settled) { fail(m,PATH_ERROR); break; }
        if (!m->waiting)
        {
            m->entered=now;
            /* Rule rows run bottom to top: 1 -> G111, 2 -> G110, 3 -> G109. */
            m->waiting=emit(m,(PathCommand){.kind=PC_GROUP,.argument=112-(code>>4),.timeout_ms=30000});
        }
        else if (in->reply==PATH_FAILED) fail(m,PATH_ERROR);
        else if (!PATH_WAREHOUSE_UNTIMED(m) && (uint32_t)(now-m->entered)>=30000) fail(m,PATH_TIMEOUT);
        else if (in->reply==PATH_OK)
        {
            if (!BallInventory_Unload(&m->inventory,code)) fail(m,PATH_ERROR);
            else {
                m->waiting=false; m->phase=WAREHOUSE_CHECK_HEADING;
            }
        }
        break;
    case WAREHOUSE_CHECK_HEADING: /* Next column/return phases own heading correction. */
        if (in->settled) advance(m,now);
        break;
    default:
        fail(m,PATH_ERROR);
        break;
    }
}
