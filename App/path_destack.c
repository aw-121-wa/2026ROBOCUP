#include "path_destack.h"
#include "path_warehouse.h"
#include "path_config.h"
#include <math.h>
#define RAD 0.01745329252f
#define ARM_TIMEOUT_MS 30000U
static void fail(PathMission *m)
{
    PathCommand c={.kind=PC_HOLD};m->send(m->context,&c);
    c.kind=PC_CANCEL;m->send(m->context,&c);m->result=PATH_ERROR;
}
static void enter(PathMission *m, unsigned phase, uint32_t now)
{
    m->phase=phase;m->waiting=false;m->entered=now;
}
static bool send(PathMission *m, PathCommand c)
{
    if (m->send(m->context,&c)) return true;
    fail(m);return false;
}
static float position(const PathMission *m,const PathInput *in)
{
    return in->x_mm*cosf(m->destack.axis)+in->y_mm*sinf(m->destack.axis);
}
/* The arm group is only issued at rest and must acknowledge completion before motion. */
static bool arm(PathMission *m,uint32_t now,const PathInput *in,unsigned group)
{
    if (!in->settled) {fail(m);return false;}
    if (!m->waiting) {
        m->entered=now;
        m->waiting=send(m,(PathCommand){.kind=PC_GROUP,.argument=group,.timeout_ms=ARM_TIMEOUT_MS});
    } else if (in->reply==PATH_FAILED || (uint32_t)(now-m->entered)>=ARM_TIMEOUT_MS) fail(m);
    else return in->reply==PATH_OK;
    return false;
}
static bool move(PathMission *m,uint32_t now,const PathInput *in,float target)
{
    if (m->waiting) {
        if ((uint32_t)(now-m->entered)>=ARM_TIMEOUT_MS) {fail(m);return false;}
        return in->settled;
    }
    if (!in->settled) return false;
    float delta=target-position(m,in), angle=m->destack.axis-in->yaw_deg*RAD;
    if (!isfinite(delta) || !isfinite(angle) || fabsf(delta)>1000) {fail(m);return false;}
    if (fabsf(delta)<0.5f) return true;
    m->entered=now;
    m->waiting=send(m,(PathCommand){.kind=PC_MOVE,.x=delta*cosf(angle),.y=delta*sinf(angle),
        .speed=80,.acceleration=700,.deceleration=700,.timeout_ms=ARM_TIMEOUT_MS});
    return false;
}
static void nearest_column(PathMission *m,uint32_t now,float current)
{
    unsigned nearest=3;float best=INFINITY;
    for(unsigned col=0;col<3;++col) {
        float distance=fabsf(m->destack.position[col]-current);
        if (!(m->destack.unloaded&(1U<<col)) && distance<best) {best=distance;nearest=col;}
    }
    if(nearest==3) {m->point=9;enter(m,DESTACK_HOME,now);return;}
    m->destack.column=(uint8_t)nearest;m->point=nearest*3;
    if (m->destack.cleared) m->warehouse_plan_ready=false;
    enter(m,DESTACK_NEXT,now);
}
static void next_row(PathMission *m,uint32_t now,const PathInput *in)
{
    if (--m->destack.row) enter(m,DESTACK_POSE,now);
    else {
        /* Finish all columns before admitting any ball-unload action. */
        if (m->destack.column) --m->destack.column;
        else {
            m->destack.cleared=true;
            nearest_column(m,now,position(m,in));
            return;
        }
        m->point=m->destack.column*3;
        enter(m,DESTACK_NEXT,now);
    }
}
bool PathDestack_Advance(PathMission *m,uint32_t now)
{
    if (!m->destack.enabled || !m->destack.scanned || !m->destack.cleared || m->point%3!=2) return false;
    m->destack.unloaded|=(uint8_t)(1U<<m->destack.column);
    nearest_column(m,now,m->destack.position[m->destack.column]);
    return true;
}
bool PathDestack_Tick(PathMission *m,uint32_t now,const PathInput *in)
{
    if (!m->destack.enabled && in->destack_enabled && in->warehouse_vision &&
        m->phase==WAREHOUSE_SELECT_BALL && m->point==0) {
        m->destack.enabled=true;
        m->destack.axis=(in->yaw_deg-in->map_yaw_deg+PATH_WAREHOUSE_TARGET_DEG(m->blue))*RAD;
    }
    if (!m->destack.enabled) return false;
    if (m->phase==WAREHOUSE_SELECT_BALL && !m->destack.scanned) {
        if (!in->settled && m->point==6) return true; /* Third column begins stationary arm work. */
        unsigned col=m->point/3;
        float p=position(m,in);
        if (col>2 || !isfinite(p) || (col && (p-m->destack.position[col-1]<50 ||
            p-m->destack.position[col-1]>500))) {fail(m);return true;}
        m->destack.position[col]=p;
        if (col<2) {m->point+=3;enter(m,WAREHOUSE_MOVE,now);}
        else {
            if (!m->blue) {
                /* Anchor all unloading positions to the correctly aligned third column. */
                float spacing=PATH_WAREHOUSE_SPACING_MM(m->blue);
                m->destack.position[1]=p-spacing;
                m->destack.position[0]=p-2*spacing;
            }
            m->destack.scanned=true;m->destack.column=2;m->destack.row=3;
            enter(m,DESTACK_POSE,now);
        }
        return true;
    }
    if (m->phase<DESTACK_POSE) return false;
    switch(m->phase) {
    case DESTACK_POSE:
        if (arm(m,now,in,121-3*m->destack.row)) enter(m,DESTACK_CHECK,now);
        break;
    case DESTACK_CHECK:
        if (!in->settled) {fail(m);break;}
        if (!m->waiting) {
            m->entered=now;
            m->waiting=send(m,(PathCommand){.kind=PC_BLOCK_CHECK,.argument=m->destack.row,.timeout_ms=10000});
        } else if (in->warehouse_digit_reply==PATH_FAILED || (uint32_t)(now-m->entered)>=11000) fail(m);
        else if (in->warehouse_digit_reply==PATH_OK) {
            unsigned digit=in->warehouse_digit;
            if (digit==4) next_row(m,now,in); /* Explicit EMPTY, never a missing numeric result. */
            else if (digit>=1 && digit<=3) {
                if (m->destack.occupied&(1U<<digit)) {fail(m);break;}
                m->destack.target=(uint8_t)digit;enter(m,DESTACK_PICK,now);
            } else if (digit==0) next_row(m,now,in); /* Recognition budget exhausted: skip this cell. */
            else fail(m);
        }
        break;
    case DESTACK_PICK:
        if (arm(m,now,in,122-3*m->destack.row)) {
            m->destack.carrying=true;enter(m,DESTACK_TO_FOURTH,now);
        }
        break;
    case DESTACK_TO_FOURTH:
        if (move(m,now,in,m->destack.position[2]+PATH_WAREHOUSE_SPACING_MM(m->blue))) enter(m,DESTACK_PLACE,now);
        break;
    case DESTACK_PLACE:
        if (arm(m,now,in,123-3*m->destack.target)) {
            m->destack.carrying=false;m->destack.occupied|=(uint8_t)(1U<<m->destack.target);
            if (!m->destack.column && (m->blue || m->destack.row==1)) {
                if (m->blue) m->destack.row=1;
                next_row(m,now,in); /* Final source cell complete: stay near column 4. */
            } else enter(m,DESTACK_RETURN,now);
        }
        break;
    case DESTACK_RETURN:
        if (move(m,now,in,m->destack.position[m->destack.column]-(m->blue ? 0 : 20.0f))) {
            /* Random placement can leave another block in this same column. */
            if (m->blue) m->destack.row=1; /* Preserve the existing blue workflow. */
            next_row(m,now,in);
        }
        break;
    case DESTACK_NEXT:
        if (move(m,now,in,m->destack.position[m->destack.column])) {
            m->destack.row=3;
            enter(m,m->destack.cleared ?
                WAREHOUSE_SELECT_BALL :
                DESTACK_POSE,now);
        }
        break;
    case DESTACK_HOME:
        if (in->settled) {
            if (BallInventory_HasKnown(&m->inventory) || m->inventory.uncertain) {fail(m);break;}
            enter(m,WAREHOUSE_ALIGN_HOME,now);
        }
        break;
    default:fail(m);break;
    }
    return true;
}
