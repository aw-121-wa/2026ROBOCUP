#include "path_destack.h"
#include "path_warehouse.h"
#include "path_chassis.h"
#include "path_config.h"
#include <math.h>
#include <stdlib.h>
#define RAD 0.01745329252f
#define CENTER_PX 8
#define SCAN_RPM 10.0f
static bool send(PathMission *m,PathCommand c)
{
    if(m->send(m->context,&c)) return true;
    (void)m->send(m->context,&(PathCommand){.kind=PC_HOLD});
    (void)m->send(m->context,&(PathCommand){.kind=PC_CANCEL});
    m->result=PATH_ERROR;return false;
}
static void hold(PathMission *m) { m->destack.braking=true;(void)send(m,(PathCommand){.kind=PC_HOLD}); }
static void fail(PathMission *m)
{
    hold(m);(void)send(m,(PathCommand){.kind=PC_CANCEL});m->result=PATH_ERROR;
}
static void enter(PathMission *m,unsigned state,uint32_t now)
{
    m->destack.state=(uint8_t)state;m->destack.since=now;
    m->destack.arm_wait=false;m->destack.stopping=false;
    m->phase=state;
}
static float position(const PathMission *m,const PathInput *in)
{ return in->x_mm*cosf(m->destack.axis)+in->y_mm*sinf(m->destack.axis); }
static void velocity(PathMission *m,const PathInput *in,float rpm)
{
    if(m->destack.braking && !in->settled) return;
    m->destack.braking=false;
    float a=m->destack.axis-in->yaw_deg*RAD;
    (void)send(m,(PathCommand){.kind=PC_BODY,.x=rpm*cosf(a),.y=rpm*sinf(a),.timeout_ms=30000});
}
static bool move(PathMission *m,const PathInput *in,float goal,float rpm)
{
    float error=goal-position(m,in);
    if(fabsf(error)<=1) {hold(m);return in->settled;}
    velocity(m,in,copysignf(fminf(rpm,fmaxf(3,fabsf(error))),error));
    return false;
}
static bool stream(PathMission *m,const PathInput *in,unsigned row)
{
    if(!m->destack.stream) {
        if(in->scan_active) return false;
        if(send(m,(PathCommand){.kind=PC_WAREHOUSE_STREAM,.argument=row})) {
            m->destack.stream=true;m->destack.frame=0;
        }
        return false;
    }
    return in->scan_active && in->scan_fresh;
}
static bool stop_stream(PathMission *m,const PathInput *in)
{
    if(m->destack.stream) {
        (void)send(m,(PathCommand){.kind=PC_WAREHOUSE_STREAM,.argument=4});
        m->destack.stream=false;return false;
    }
    return !in->scan_active;
}
static bool arm(PathMission *m,uint32_t now,const PathInput *in,unsigned group)
{
    if(!in->settled || in->scan_active) {hold(m);return false;}
    if(!m->destack.arm_wait) {
        m->destack.since=now;
        m->destack.arm_wait=send(m,(PathCommand){.kind=PC_GROUP,.argument=group,.timeout_ms=30000});
    } else if(in->reply==PATH_FAILED || (uint32_t)(now-m->destack.since)>=30000) fail(m);
    else if(in->reply==PATH_OK) return true;
    return false;
}
/* Line recovery never changes arm state or scan progress. A new frame is required
 * after lateral recovery; frames seen during correction cannot trigger pickup. */
static bool line_ready(PathMission *m,uint32_t now,const PathInput *in)
{
    PathDestack *d=&m->destack;
    if(in->gray==6 && !d->correcting) return true;
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
    unsigned nearest=3;float best=INFINITY;
    for(unsigned c=0;c<3;++c) if(!(d->mapped&(1U<<c))) {
        float distance=fabsf(d->current_position-d->columns[c]);
        if(distance<best) {best=distance;nearest=c;}
    }
    if(nearest<3) {
        d->column=nearest;m->point=nearest*3;enter(m,DS_BALL_MOVE,now);
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
        (void)send(m,(PathCommand){.kind=PC_MAP_HEADING,.x=PATH_WAREHOUSE_TARGET_DEG(m->blue)});
    }
    if(m->point==9) return false;
    if(!isfinite(in->yaw_deg)||!isfinite(in->x_mm)||!isfinite(in->y_mm)) {fail(m);return true;}
    /* Entry alone permits a middle probe off: establish the longitudinal datum. */
    if(d->state==DS_ENTRY) {
        if(!(in->gray&2U)) {
            hold(m);
            if(in->settled) {
                d->origin=position(m,in);
                for(unsigned c=0;c<3;++c) d->columns[c]=d->origin+(c+1)*PATH_WAREHOUSE_COLUMN_SPACING_MM;
                enter(m,DS_THIRD,now);
            }
        } else velocity(m,in,-12);
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
    case DS_THIRD:
        if(!stop_stream(m,in)) {hold(m);break;}
        if(move(m,in,d->columns[2]+15,40)) {d->row=3;enter(m,DS_POSE,now);}
        break;
    case DS_POSE:
        if(arm(m,now,in,121-3*d->row)) {
            d->direction=d->row==2 ? 1 : -1;
            d->scan_end=d->columns[d->direction>0 ? 2 : 0]+d->direction*15;
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
                    float dp=p-d->sample_position;
                    if(d->sample_valid && d->sample_digit==in->scan_digit && fabsf(dp)>1 && fabsf(in->scan_offset-d->sample_offset)>1)
                        d->pixel_slope=(in->scan_offset-d->sample_offset)/dp;
                    if(!d->sample_valid || fabsf(dp)>1) {
                        d->sample_position=p;d->sample_offset=in->scan_offset;
                        d->sample_valid=true;d->sample_digit=in->scan_digit;
                    }
                    if(abs(in->scan_offset)<=CENTER_PX) {
                        d->target=in->scan_digit;d->column=col;d->resume=p;
                        hold(m);enter(m,DS_CENTER,now);break;
                    }
                }
            }
        }
        if((uint32_t)(now-d->fresh_since)<350) {hold(m);break;} /* Inspect the initial endpoint. */
        if((p-d->scan_end)*d->direction>=0) {hold(m);enter(m,DS_STOP_ROW,now);break;}
        velocity(m,in,d->direction*SCAN_RPM);break;
    case DS_CENTER:
        if(!in->scan_fresh) {hold(m);break;}
        if(d->stopping) {
            if(move(m,in,d->goal,1)) {d->stopping=false;d->frame=in->scan_frame;}
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
        } else if(fabsf(d->pixel_slope)>.05f && fabsf(p-d->resume)<20) {
            float delta=-in->scan_offset/d->pixel_slope;
            d->goal=p+copysignf(fminf(fabsf(delta),3),delta);d->stopping=true;
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
        if(move(m,in,d->columns[2]+PATH_WAREHOUSE_COLUMN_SPACING_MM,70)) enter(m,DS_PLACE,now);
        break;
    case DS_PLACE:
        if(arm(m,now,in,123-3*d->target)) {
            d->occupied|=1U<<d->target;d->handled|=7U<<((d->row-1)*3);
            enter(m,DS_RETURN,now);
        } break;
    case DS_RETURN:
        /* One source block per row: move to the next row's scan endpoint. */
        if(d->row==1) {d->cleared=true;next_ball(m,now);break;}
        if(move(m,in,d->columns[d->row==2 ? 2 : 0]+(d->row==2 ? 15 : -15),70))
            enter(m,DS_STOP_ROW,now);
        break;
    case DS_STOP_ROW:
        hold(m);
        if(stop_stream(m,in) && in->settled) {
            if(--d->row) enter(m,DS_POSE,now);
            else {d->cleared=true;next_ball(m,now);}
        } break;
    case DS_BALL_MOVE:
        /* Observe the bottom digit while travelling to the nearest unserved column. */
        if(!d->stopping) {
            if(!stream(m,in,0)) {hold(m);break;}
            unsigned digit=in->scan_digit;
            if(new_frame) {
                d->frame=in->scan_frame;
                if(!m->warehouse_columns[d->column] && digit>=1 && digit<=3 && abs(in->scan_offset)<=CENTER_PX &&
                   fabsf(p-d->columns[d->column])<PATH_WAREHOUSE_COLUMN_SPACING_MM*.5f &&
                   !(m->warehouse_used&(1U<<digit))) {
                    m->warehouse_columns[d->column]=digit;m->warehouse_used|=1U<<digit;
                    for(unsigned c=0;c<3;++c) {
                        unsigned a=m->warehouse_columns[(c+1)%3], b=m->warehouse_columns[(c+2)%3];
                        if(!m->warehouse_columns[c] && a && b) m->warehouse_columns[c]=6-a-b;
                    }
                }
            }
            if(!move(m,in,d->columns[d->column],70)) break;
            if(!m->warehouse_columns[d->column]) break;
            d->stopping=true;
        }
        hold(m);
        if(stop_stream(m,in) && in->settled) {
            d->state=DS_BALL;m->phase=WAREHOUSE_SELECT_BALL;m->waiting=false;
            m->warehouse_mode=WAREHOUSE_DIGIT_ORDER;m->warehouse_plan_ready=false;
        } break;
    default:fail(m);break;
    }
    return true;
}

