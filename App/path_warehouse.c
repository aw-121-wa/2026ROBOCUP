#include "path_warehouse.h"
#include "path_chassis.h"
#include "path_config.h"
#include <math.h>
/* Compare all 6^3 row orders once, including the transitions between columns.
 * Cache pocket lookups before enumeration; replan when a new digit changes the mapping. */
static unsigned column_code(const PathMission *m,unsigned col)
{
    return m->warehouse_columns[col] ? m->warehouse_columns[col] : col+1;
}
static void plan_warehouse(PathMission *m)
{
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
    if (m->send(m->context,&command)) return true;
    fail(m,PATH_ERROR);
    return false;
}
/* Recognition and first-column fallback must stop motion before unloading. */
static void brake_for_unload(PathMission *m, uint32_t now)
{
    if (!emit(m, (PathCommand){.kind=PC_HOLD})) return;
    m->waiting=false;
    m->phase=WAREHOUSE_BRAKE;
    m->entered=now;
}
static void begin_lateral_alignment(PathMission *m, uint32_t now)
{
    m->waiting=false;
    m->phase=WAREHOUSE_ALIGN_LINE;
    m->line_active=m->stable=m->line_stopping=false;
    m->entered=now;
}
static bool valid_digit(const PathInput *in)
{
    return in->warehouse_digit_reply==PATH_OK &&
           in->warehouse_digit>=1 && in->warehouse_digit<=3;
}
static void remember_digit(PathMission *m, uint8_t digit)
{
    m->warehouse_columns[m->point/3]=digit;
    m->warehouse_used|=(uint8_t)(1U<<digit);
    m->warehouse_plan_ready=false;
}
static void advance(PathMission *m, uint32_t now)
{
    ++m->point;
    m->waiting=false;
    m->entered=now;
    m->phase=m->point%3 == 0 ? WAREHOUSE_MOVE : WAREHOUSE_SELECT_BALL;
    if (m->point==9)
    {
        if (m->inventory.occupied) fail(m,PATH_ERROR);
        else if (emit(m,(PathCommand){.kind=PC_HOLD})) m->phase=WAREHOUSE_ALIGN_HOME;
    }
}
static float home_map_y(const PathInput *in)
{
    float axis=(in->yaw_deg-in->map_yaw_deg)*0.01745329252f;
    return -sinf(axis)*in->x_mm+cosf(axis)*in->y_mm;
}
void PathWarehouse_Tick(PathMission *m, uint32_t now, const PathInput *in)
{
    if (m->point==9 && m->phase==WAREHOUSE_ALIGN_HOME && !m->inventory.occupied && !m->inventory.uncertain) {
        /* Brake after unloading, then correct heading during return travel. */
        if (in->settled) {
            m->waiting=false; m->stable=false;
            m->phase=WAREHOUSE_RETURN_HOME; m->entered=now;
        }
        return;
    }
    if (m->point==9 && m->phase==WAREHOUSE_HOME_ADVANCE) {
        if (in->settled) m->result=PATH_DONE;
        else if (!PATH_WAREHOUSE_UNTIMED(m) && (uint32_t)(now-m->entered)>=5000U) fail(m,PATH_TIMEOUT);
        return;
    }
    if (m->point==9 && m->phase==WAREHOUSE_HOME_BRAKE) {
        if (in->settled) {
            float current=home_map_y(in);
            float remaining=20.0f-(m->home_line_y-current);
            if (m->blue || remaining<=0.5f) m->result=PATH_DONE;
            else if (!isfinite(remaining)) fail(m,PATH_ERROR);
            else if (emit(m,(PathCommand){.kind=PC_MAP_LATERAL,.y=-remaining,.timeout_ms=5000})) {
                m->phase=WAREHOUSE_HOME_ADVANCE; m->entered=now;
            }
        }
        else if (!PATH_WAREHOUSE_UNTIMED(m) && (uint32_t)(now-m->entered)>=3000U) fail(m,PATH_TIMEOUT);
        return;
    }
    if (m->point==9 && (m->phase==WAREHOUSE_RETURN_HOME || m->phase==WAREHOUSE_HOME_RIGHT ||
                       m->phase==WAREHOUSE_HOME_SEARCH) && !m->inventory.occupied && !m->inventory.uncertain) {
        unsigned leg=m->phase==WAREHOUSE_RETURN_HOME ? 0 : m->phase==WAREHOUSE_HOME_RIGHT ? 1 : 2;
        unsigned gray=in->gray&15U;
        bool arrived=gray && (gray&(gray-1U));
        if (!arrived) m->stable=true; /* Do not mistake the warehouse line for home. */
        if (leg && m->stable && arrived) {
            if (emit(m,(PathCommand){.kind=PC_HOLD})) {
                m->home_line_y=home_map_y(in);
                m->phase=WAREHOUSE_HOME_BRAKE; m->entered=now;
            }
        } else if (!m->waiting) {
            if (!in->settled) return;
            m->entered=now;
            m->waiting=emit(m,(PathCommand){.kind=PC_RETURN_HOME,.argument=leg,.timeout_ms=30000});
        } else if (!PATH_WAREHOUSE_UNTIMED(m) && (uint32_t)(now-m->entered)>=30000) fail(m,PATH_TIMEOUT);
        else if (in->settled) {
            if (leg==2) fail(m,PATH_TIMEOUT); /* Distance alone is not proof of arriving home. */
            else {
                m->phase=leg==0 ? WAREHOUSE_HOME_RIGHT : WAREHOUSE_HOME_SEARCH;
                m->waiting=false; m->entered=now;
            }
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
    if (m->prep_pending && (m->phase==WAREHOUSE_SELECT_BALL || m->phase==WAREHOUSE_UNLOAD ||
        (m->phase==WAREHOUSE_MOVE && m->point==0 && in->warehouse_vision))) return;
    switch(m->phase)
    {
    case WAREHOUSE_ENTRY_BACK:
        if (!PATH_WAREHOUSE_UNTIMED(m) && (uint32_t)(now-m->entered)>=5000U) { fail(m,PATH_TIMEOUT); break; }
        if (!in->settled) break;
        if (emit(m,(PathCommand){.kind=PC_MAP_LATERAL,
                .y=-PATH_WAREHOUSE_FIRST_RIGHT_MM,.timeout_ms=10000})) {
            m->phase=WAREHOUSE_FIRST_OFFSET;
            m->entered=now;
        }
        break;
    case WAREHOUSE_FIRST_OFFSET:
        if (!PATH_WAREHOUSE_UNTIMED(m) && (uint32_t)(now-m->entered)>=10000U) { fail(m,PATH_TIMEOUT); break; }
        if (!in->settled || !PathHeading_Ready(m,now,in)) break;
        m->waiting=false;
        if (in->warehouse_vision && m->warehouse_mode!=WAREHOUSE_DEFAULT_ORDER) {
            m->entered=now;
            m->warehouse_query=emit(m,(PathCommand){.kind=PC_WAREHOUSE_DIGIT,
                .argument=m->warehouse_used,.timeout_ms=PATH_WAREHOUSE_DIGIT_TIMEOUT_MS});
            if (m->warehouse_query) m->phase=WAREHOUSE_WAIT_CAMERA;
        } else {
            m->warehouse_mode=WAREHOUSE_DEFAULT_ORDER;
            begin_lateral_alignment(m,now);
        }
        break;
    case WAREHOUSE_CHECK_OFFSET_LINE:
        if (!PATH_WAREHOUSE_UNTIMED(m) && (uint32_t)(now-m->entered)>=10000U) { fail(m,PATH_TIMEOUT); break; }
        if (PathLine_AlignFour(m,now,in)) {
            m->phase=WAREHOUSE_FIRST_OFFSET;
            m->entered=now;
        }
        break;
    case WAREHOUSE_WAIT_CAMERA:
        /* Recognition is stationary after the first map-right offset. */
        if (in->warehouse_digit_reply!=PATH_WAIT || in->warehouse_ready ||
            (!PATH_WAREHOUSE_UNTIMED(m) && (uint32_t)(now-m->entered)>=PATH_WAREHOUSE_DIGIT_GUARD_MS))
            m->phase=WAREHOUSE_FIRST_DIGIT;
        if (in->warehouse_ready && in->warehouse_digit_reply==PATH_WAIT &&
            m->phase==WAREHOUSE_FIRST_DIGIT)
            emit(m,(PathCommand){.kind=PC_MOVE,.x=PATH_WAREHOUSE_CREEP_LIMIT_MM,
                .speed=PATH_WAREHOUSE_CREEP_SPEED_RPM,.acceleration=PATH_WAREHOUSE_CREEP_ACCEL,
                .deceleration=650,.timeout_ms=PATH_WAREHOUSE_CREEP_TIMEOUT_MS});
        break;
    case WAREHOUSE_FIRST_DIGIT: /* First-column result selects the mode once for the whole warehouse. */
        if (PATH_WAREHOUSE_UNTIMED(m)) {
            if (in->warehouse_digit_reply==PATH_WAIT) break;
            if (m->point && (!valid_digit(in) || (m->warehouse_used & (1U<<in->warehouse_digit)))) {
                /* Retry in place: do not restart creep beyond its distance limit. */
                m->warehouse_query=emit(m,(PathCommand){.kind=PC_WAREHOUSE_DIGIT,
                    .argument=m->warehouse_used,.timeout_ms=PATH_WAREHOUSE_DIGIT_TIMEOUT_MS});
                break;
            }
        }
        if (m->point && in->warehouse_digit_reply!=PATH_WAIT &&
            (!valid_digit(in) || (m->warehouse_used & (1U<<in->warehouse_digit)))) {
            fail(m,PATH_TIMEOUT); break;
        }
        if (in->warehouse_digit_reply==PATH_WAIT) {
            if ((uint32_t)(now-m->entered)<PATH_WAREHOUSE_DIGIT_GUARD_MS) break;
            if (m->point) { fail(m,PATH_TIMEOUT); break; }
            m->warehouse_mode=WAREHOUSE_DEFAULT_ORDER;
        } else if (valid_digit(in)) {
            m->warehouse_mode=WAREHOUSE_DIGIT_ORDER;
            remember_digit(m,in->warehouse_digit);
        } else m->warehouse_mode=WAREHOUSE_DEFAULT_ORDER;
        m->warehouse_query=false;
        if (m->warehouse_mode==WAREHOUSE_DIGIT_ORDER) {
            if (emit(m,(PathCommand){.kind=PC_FINISH_FORWARD,.x=PATH_WAREHOUSE_DIGIT_ADVANCE_MM,
                    .speed=PATH_WAREHOUSE_CREEP_SPEED_RPM,.acceleration=PATH_WAREHOUSE_CREEP_ACCEL,
                    .deceleration=650,.timeout_ms=5000})) {
                m->waiting=true; m->phase=WAREHOUSE_BRAKE; m->entered=now;
            }
        } else brake_for_unload(m,now);
        break;
    case WAREHOUSE_BRAKE:
        if (!PATH_WAREHOUSE_UNTIMED(m) && (uint32_t)(now-m->entered)>=5000) { fail(m,PATH_TIMEOUT); break; }
        if (in->settled) {
            if (m->warehouse_mode==WAREHOUSE_DIGIT_ORDER) {
                /* A confirmed digit locates this column; retain gyro heading hold. */
                m->waiting=false;
                m->phase=WAREHOUSE_SELECT_BALL;
                m->entered=now;
            } else begin_lateral_alignment(m,now);
        }
        break;
    case WAREHOUSE_MOVE: /* Start at the detected line; only subsequent columns require a move. */
        if (m->point==0 && !m->waiting) {
            if (!in->settled) break;
            if (!PathHeading_Ready(m,now,in)) break;
            m->entered=now;
            /* Retreat along map -X, independently of residual body yaw. */
            float yaw=in->map_yaw_deg*0.01745329252f;
            if (!isfinite(yaw)) { fail(m,PATH_ERROR); break; }
            if (emit(m,(PathCommand){.kind=PC_MOVE,
                    .x=-PATH_WAREHOUSE_ENTRY_BACK_MM*cosf(yaw),
                    .y=PATH_WAREHOUSE_ENTRY_BACK_MM*sinf(yaw),
                    .speed=PATH_WAREHOUSE_CREEP_SPEED_RPM,.acceleration=300,.deceleration=300,.timeout_ms=5000}))
                m->phase=WAREHOUSE_ENTRY_BACK;
            break;
        }
        if (!m->waiting) {
            if (!in->settled || !PathHeading_Ready(m,now,in)) break;
            m->entered=now;
            /* Later columns are reached by forward digit scanning, without a right shift.
             * Default order retains the fixed forward column spacing. */
            if (m->warehouse_mode==WAREHOUSE_DEFAULT_ORDER &&
                !emit(m,(PathCommand){.kind=PC_MOVE,.x=200,.speed=120,
                    .acceleration=850,.deceleration=850,.timeout_ms=30000})) break;
            m->phase=WAREHOUSE_FIRST_OFFSET;
        }
        break;
    case WAREHOUSE_RETURN_LINE:
        if (!PATH_WAREHOUSE_UNTIMED(m) && (uint32_t)(now-m->entered)>=30000U) { fail(m,PATH_TIMEOUT); break; }
        if (PathLine_Aligned(m,in->gray)) {
            if (!m->line_stopping) {
                if (emit(m,(PathCommand){.kind=PC_HOLD})) m->line_stopping=true;
            } else if (in->settled) {
                m->line_stopping=false;
                m->phase=WAREHOUSE_ALIGN_LINE;
                m->line_active=m->stable=false;
                m->entered=now;
            }
        } else if (!m->line_stopping)
            emit(m,(PathCommand){.kind=PC_BODY,.x=-10,.timeout_ms=30000});
        break;
    case WAREHOUSE_ALIGN_LINE: /* Default-order fallback retains lateral line alignment. */
        if (PathLine_AlignFour(m,now,in)) {
            m->phase=WAREHOUSE_SELECT_BALL;
            m->entered=now;
        } else if (m->result!=PATH_RUNNING && m->inventory.occupied) {
            m->inventory.uncertain=true;
        }
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
