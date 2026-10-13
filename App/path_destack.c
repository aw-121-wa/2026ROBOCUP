#include "path_destack.h"
#include "path_warehouse.h"
#include "path_chassis.h"
#include "path_config.h"
#include <math.h>
#include <stdlib.h>
#define RAD 0.01745329252f
#define CENTER_PX 8

static __attribute__((noinline)) bool send(PathMission *m,const PathCommand *c)
{
    if(m->send(m->context,c)) return true;
    (void)m->send(m->context,&(PathCommand){.kind=PC_HOLD});
    (void)m->send(m->context,&(PathCommand){.kind=PC_CANCEL});
    m->result=PATH_ERROR;return false;
}
static void hold(PathMission *m) { m->destack.braking=true;(void)send(m,&(PathCommand){.kind=PC_HOLD}); }
static void fail(PathMission *m)
{
    hold(m);(void)send(m,&(PathCommand){.kind=PC_CANCEL});m->result=PATH_ERROR;
}
static void enter(PathMission *m,unsigned state,uint32_t now)
{
    m->destack.state=(uint8_t)state;m->destack.since=now;
    m->destack.arm_wait=false;m->destack.stopping=false;
    m->phase=state;
}
static float position(const PathMission *m,const PathInput *in)
{ return in->x_mm*cosf(m->destack.axis)+in->y_mm*sinf(m->destack.axis); }
static __attribute__((noinline)) void velocity(PathMission *m,const PathInput *in,float rpm)
{
    if(m->destack.braking && !in->settled) return;
    m->destack.braking=false;
    float a=m->destack.axis-in->yaw_deg*RAD;
    (void)send(m,&(PathCommand){.kind=PC_BODY,.x=rpm*cosf(a),.y=rpm*sinf(a),.timeout_ms=30000});
}
static bool move(PathMission *m,const PathInput *in,float goal,float rpm)
{
    float error=goal-position(m,in);
    if(fabsf(error)<=ROUTE_WAREHOUSE_MOVE_TOL_MM) {hold(m);return in->settled;}
    velocity(m,in,copysignf(fminf(rpm,fmaxf(ROUTE_WAREHOUSE_APPROACH_MIN_RPM_COMMAND,fabsf(error))),error));
    return false;
}
/* A confirmed digit may cross the center between two camera updates. */
static bool centered(const PathDestack *d,const PathInput *in)
{
    return abs(in->scan_offset)<=CENTER_PX ||
        (d->sample_valid && d->sample_digit==in->scan_digit &&
         fabsf(d->sample_offset)<=80 && abs(in->scan_offset)<=80 &&
         (int)d->sample_offset*in->scan_offset<0);
}
static bool stream(PathMission *m,const PathInput *in,unsigned row)
{
    if(!m->destack.stream) {
        if(in->scan_active) return false;
        if(send(m,&(PathCommand){.kind=PC_WAREHOUSE_STREAM,.argument=row})) {
            m->destack.stream=true;m->destack.frame=0;
        }
        return false;
    }
    return in->scan_active && in->scan_fresh;
}
static bool stop_stream(PathMission *m,const PathInput *in)
{
    if(m->destack.stream) {
        (void)send(m,&(PathCommand){.kind=PC_WAREHOUSE_STREAM,.argument=4});
        m->destack.stream=false;return false;
    }
    return !in->scan_active;
}
static bool arm(PathMission *m,uint32_t now,const PathInput *in,unsigned group)
{
    if(!in->settled || in->scan_active) {hold(m);return false;}
    if(!m->destack.arm_wait) {
        m->destack.since=now;
        m->destack.arm_wait=send(m,&(PathCommand){.kind=PC_GROUP,.argument=group,.timeout_ms=30000});
    } else if(in->reply==PATH_FAILED || (uint32_t)(now-m->destack.since)>=30000) fail(m);
    else if(in->reply==PATH_OK) return true;
    return false;
}
/* Line recovery never changes arm state or scan progress. A new frame is required
 * after lateral recovery; frames seen during correction cannot trigger pickup. */
static bool line_ready(PathMission *m,uint32_t now,const PathInput *in)
{
    PathDestack *d=&m->destack;
    /* Rear-off is the entry datum; depart longitudinally before enforcing 0110. */
    if((d->state==DS_FIRST || in->gray==6 || (d->state==DS_BALL_MOVE && !d->mapped && in->gray==4)) && !d->correcting) return true;
    if(!d->correcting) {hold(m);d->correcting=true;return false;}
    if(!m->line_active && !in->settled) return false;
    if(PathLine_AlignFour(m,now,in)) {
        d->correcting=false;d->frame=in->scan_frame;d->fresh_since=now;
    }
    return false;
}
/* Choose by current position, not numeric column order. */
static void next_ball(PathMission *m,uint32_t now)
{
    PathDestack *d=&m->destack;
    d->sample_valid=false;
    unsigned nearest=3;float best=INFINITY;
    for(unsigned c=0;c<3;++c) if(!(d->mapped&(1U<<c))) {
        float distance=fabsf(d->current_position-d->columns[c]);
        if(distance<best) {best=distance;nearest=c;}
    }
    if(nearest<3) {
        d->column=nearest;d->direction=d->columns[nearest]>d->current_position ? 1 : -1;
        d->goal=d->current_position+ROUTE_WAREHOUSE_LAST_COLUMN_MM; /* Last column follows the first two. */
        m->point=nearest*3;enter(m,DS_BALL_MOVE,now);
    } else {
        m->point=9;m->phase=WAREHOUSE_ALIGN_HOME;m->stable=false;
        m->waiting=false;m->entered=now;
    }
}
bool PathDestack_Advance(PathMission *m,uint32_t now)
{
    PathDestack *d=&m->destack;
    if(!d->enabled || !d->cleared) return false;
    if(m->point%3<2) {++m->point;m->phase=WAREHOUSE_SELECT_BALL;m->waiting=false;return true;}
    d->mapped|=1U<<d->column;
    next_ball(m,now);
    return true;
}
bool PathDestack_Tick(PathMission *m,uint32_t now,const PathInput *in)
{
    PathDestack *d=&m->destack;
    if(!d->enabled) {
        if(!in->destack_enabled || !in->warehouse_vision || m->phase!=WAREHOUSE_MOVE || m->point) return false;
        if(m->prep_pending || !in->settled) return true;
        *d=(PathDestack){.enabled=true,.axis=(in->yaw_deg-in->map_yaw_deg+PATH_WAREHOUSE_TARGET_DEG(m->blue))*RAD};
        enter(m,DS_ENTRY,now);
        (void)send(m,&(PathCommand){.kind=PC_MAP_HEADING,.x=PATH_WAREHOUSE_TARGET_DEG(m->blue)});
    }
    if(m->point==9) return false;
    if(!isfinite(in->yaw_deg+in->x_mm+in->y_mm)) {fail(m);return true;}
    /* Entry alone permits a middle probe off: establish the longitudinal datum. */
    if(d->state==DS_ENTRY) {
        if(!stop_stream(m,in)) {hold(m);return true;}
        if((in->gray&6U)==4U) {
            hold(m);
            if(in->settled) {
                d->origin=position(m,in);
                for(unsigned c=0;c<3;++c) d->columns[c]=d->origin+(c+1)*PATH_WAREHOUSE_COLUMN_SPACING_MM;
                if(d->cleared) {d->current_position=d->origin;next_ball(m,now);}
                else enter(m,DS_FIRST,now);
            }
        } else velocity(m,in,d->cleared ? -ROUTE_WAREHOUSE_REBASE_BACK_RPM_COMMAND : ROUTE_WAREHOUSE_ENTRY_BACK_RPM_COMMAND);
        return true;
    }
    d->current_position=position(m,in);
    /* Do not move the chassis while the arm or turntable is operating. */
    bool arm_busy=d->arm_wait || (d->state==DS_BALL &&
        (m->phase==WAREHOUSE_TURN || (m->phase==WAREHOUSE_UNLOAD && m->waiting)));
    if(!arm_busy && !line_ready(m,now,in)) return true;
    if(d->state==DS_BALL) return false;
    float p=position(m,in);
    bool new_frame=in->scan_fresh && in->scan_frame!=d->frame;
    switch(d->state) {
    case DS_FIRST:
        if(!stop_stream(m,in)) {hold(m);break;}
        if(move(m,in,d->columns[0],ROUTE_WAREHOUSE_INITIAL_RPM_COMMAND)) {d->row=3;enter(m,DS_POSE,now);}
        break;
    case DS_POSE:
        if(arm(m,now,in,121-3*d->row)) {
            d->direction=p<d->columns[1] ? 1 : -1;
            d->scan_end=d->columns[d->direction>0 ? 2 : 0]+d->direction*ROUTE_BLOCK_SCAN_MARGIN_MM;
            d->pixel_slope=0;d->sample_valid=false;
            d->fresh_since=now;enter(m,DS_SCAN,now);
        } break;
    case DS_SCAN:
        if(!stream(m,in,d->row)) {hold(m);d->fresh_since=now;break;}
        if(new_frame) {
            d->frame=in->scan_frame;
            if(in->scan_digit) {
                unsigned col=0;float best=INFINITY;
                for(unsigned c=0;c<3;++c) if(fabsf(p-d->columns[c])<best) {best=fabsf(p-d->columns[c]);col=c;}
                if(!(d->handled&(1U<<((d->row-1)*3+col)))) {
                    bool at_center=centered(d,in) || abs(in->scan_offset)<=48;
                    float dp=p-d->sample_position;
                    if(d->sample_valid && d->sample_digit==in->scan_digit && fabsf(dp)>1 && fabsf(in->scan_offset-d->sample_offset)>1)
                        d->pixel_slope=(in->scan_offset-d->sample_offset)/dp;
                    if(!d->sample_valid || fabsf(dp)>1) {
                        d->sample_position=p;d->sample_offset=in->scan_offset;
                        d->sample_valid=true;d->sample_digit=in->scan_digit;
                    }
                    if(at_center) {
                        if(fabsf(d->pixel_slope)<=.05f) d->pixel_slope=-2.0f;
                        d->target=in->scan_digit;d->column=col;d->resume=p;
                        hold(m);enter(m,DS_CENTER,now);break;
                    }
                }
            }
        }
        if((uint32_t)(now-d->fresh_since)<350) {hold(m);break;} /* Inspect the initial endpoint. */
        if((p-d->scan_end)*d->direction>=0) {hold(m);enter(m,DS_STOP_ROW,now);break;}
        float scan_rpm=ROUTE_BLOCK_SCAN_RPM_COMMAND;
        if(in->scan_digit && abs(in->scan_offset)<160) scan_rpm=fminf(scan_rpm,ROUTE_VISION_CENTER_RPM_COMMAND);
        velocity(m,in,d->direction*scan_rpm);break;
    case DS_CENTER:
        if(!in->scan_fresh) {hold(m);break;}
        if(d->stopping) {
            if(move(m,in,d->goal,ROUTE_BLOCK_NUDGE_RPM_COMMAND)) {d->stopping=false;d->frame=in->scan_frame;}
            break;
        }
        hold(m);
        if(!in->settled) {d->frame=in->scan_frame;break;}
        if(!new_frame) break;
        d->frame=in->scan_frame;
        if(in->scan_digit!=d->target) break;
        if(abs(in->scan_offset)<=CENTER_PX) {
            if(d->occupied&(1U<<d->target)) {fail(m);break;}
            enter(m,DS_STOP_PICK,now);
        } else if(fabsf(d->pixel_slope)>.05f && fabsf(p-d->resume)<ROUTE_BLOCK_NUDGE_LIMIT_MM) {
            float delta=-in->scan_offset/d->pixel_slope;
            d->goal=p+copysignf(fminf(fabsf(delta),ROUTE_BLOCK_NUDGE_MM),delta);d->stopping=true;
        }
        break;
    case DS_STOP_PICK:
        hold(m);
        if(stop_stream(m,in) && in->settled) enter(m,DS_PICK,now);
        break;
    case DS_PICK:
        if(arm(m,now,in,122-3*d->row)) enter(m,DS_FOURTH,now);
        break;
    case DS_FOURTH:
        if(move(m,in,d->columns[2]+PATH_WAREHOUSE_COLUMN_SPACING_MM,ROUTE_WAREHOUSE_TO_FOURTH_RPM_COMMAND)) enter(m,DS_PLACE,now);
        break;
    case DS_PLACE:
        if(arm(m,now,in,123-3*d->target)) {
            d->occupied|=1U<<d->target;d->handled|=7U<<((d->row-1)*3);
            enter(m,DS_RETURN,now);
        } break;
    case DS_RETURN:
        /* Fourth column is nearest the third-column scan endpoint for every row. */
        if(d->row==1) {enter(m,DS_STOP_ROW,now);break;}
        if(move(m,in,d->columns[2]+ROUTE_BLOCK_SCAN_MARGIN_MM,ROUTE_WAREHOUSE_AFTER_BLOCK_RPM_COMMAND))
            enter(m,DS_STOP_ROW,now);
        break;
    case DS_STOP_ROW:
        hold(m);
        if(stop_stream(m,in) && in->settled) {
            if(--d->row) enter(m,DS_POSE,now);
            else {d->cleared=true;enter(m,DS_ENTRY,now);}
        } break;
    case DS_BALL_MOVE:
        /* Continue past estimated centers until a fresh, unserved bottom number is seen. */
        if(!d->stopping) {
            /* Off-center recognition must not become a column datum. Inferred last column uses distance below. */
            unsigned digit=centered(d,in) ? in->scan_digit : 0;
            if(d->column==2 && m->warehouse_columns[0] && m->warehouse_columns[1]) {
                if(!move(m,in,d->goal,ROUTE_WAREHOUSE_TRANSFER_RPM_COMMAND)) break;
                digit=6-m->warehouse_columns[0]-m->warehouse_columns[1]; /* 6 minus the two distinct IDs. */
                new_frame=true;
            } else if(!stream(m,in,0)) {hold(m);break;}
            if(new_frame) {
                d->frame=in->scan_frame;
                d->sample_valid=in->scan_digit!=0;d->sample_digit=in->scan_digit;d->sample_offset=in->scan_offset;
                if(digit>=1 && digit<=3 && !(m->warehouse_used&(1U<<digit))) {
                    /* A fresh centered identity is the stop trigger. */
                    unsigned col=d->column;
                    m->warehouse_columns[col]=digit;m->warehouse_used|=1U<<digit;
                    d->stopping=true;
                }
            }
            if(!d->stopping) {
                /* Search both ways within the three-column shelf if a number is missed. */
                float low=d->origin+PATH_WAREHOUSE_COLUMN_SPACING_MM*.5f;
                float high=d->origin+PATH_WAREHOUSE_COLUMN_SPACING_MM*3.5f;
                if((d->direction<0 && p<=low) || (d->direction>0 && p>=high)) {
                    hold(m);d->direction=-d->direction;
                } else velocity(m,in,d->direction*((in->scan_digit && !(m->warehouse_used&(1U<<in->scan_digit)) && abs(in->scan_offset)<160) ? ROUTE_VISION_CENTER_RPM_COMMAND : ROUTE_WAREHOUSE_TRANSFER_RPM_COMMAND));
                break;
            }
        }
        hold(m);
        if(stop_stream(m,in) && in->settled) {
            d->columns[d->column]=p; /* Record the actual brake-complete location. */
            d->state=DS_BALL;m->phase=WAREHOUSE_SELECT_BALL;m->waiting=false;
            m->warehouse_mode=WAREHOUSE_DIGIT_ORDER;m->warehouse_plan_ready=false;
        } break;
    default:fail(m);break;
    }
    return true;
}

