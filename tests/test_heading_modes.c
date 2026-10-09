#include <assert.h>
#include <math.h>
#include "heading.h"
#include "heading_tuning.h"
int main(void) {
    HeadingControlConfig c={.kp=4,.ki=0,.damping=.15f,.limit=1.9f};
    HeadingRequest r={.mode=HEADING_TRAVEL,.error=.1f,.gyro=.2f};
    float integral=0;
    assert(fabsf(HeadingControl_Update(&r,&c,.005f,&integral)-.53f)<1e-6f);
    r.mode=HEADING_PRECISION; r.error=.001f; r.gyro=0;
    assert(fabsf(HeadingControl_Update(&r,&c,.005f,&integral)-.002f)<1e-6f);
    r.error=-.01f; assert(HeadingControl_Update(&r,&c,.005f,&integral)<0);
    r.mode=HEADING_DYNAMIC; r.error=0; r.gyro=.5f; r.feedforward=.5f;
    assert(fabsf(HeadingControl_Update(&r,&c,.005f,&integral)-.5f)<1e-6f);
    /* Saturation preserves the existing dynamic feedforward + feedback contract. */
    r.error=1; r.gyro=0;
    assert(fabsf(HeadingControl_Update(&r,&c,.005f,&integral)-1.9f)<1e-6f);
    /* An ahead-of-target chassis must reverse even when feedforward exceeds the limit. */
    r.error=-2; r.gyro=4; r.feedforward=4;
    assert(HeadingControl_Update(&r,&c,.005f,&integral)==-1.9f);
    r.error=2; r.gyro=-4; r.feedforward=-4;
    assert(HeadingControl_Update(&r,&c,.005f,&integral)==1.9f);
    r.mode=HEADING_ROTATE; r.limit=.2f; r.error=-.3f;
    assert(fabsf(HeadingControl_Update(&r,&c,.005f,&integral)+.2f)<1e-6f);
    r.mode=HEADING_OFF; integral=.3f;
    assert(HeadingControl_Update(&r,&c,.005f,&integral)==0 && integral==0);
    r.mode=HEADING_MANUAL; r.feedforward=-.7f;
    assert(HeadingControl_Update(&r,&c,.005f,&integral)==-.7f);
    r.mode=HEADING_HOLD; r.error=.5f; r.gyro=0;
    assert(fabsf(HeadingControl_Update(&r,&c,.005f,&integral)-.01745329252f)<1e-6f);
    r.mode=HEADING_TRAVEL; r.error=NAN;
    assert(HeadingControl_Update(&r,&c,.005f,&integral)==0);
    return 0;
}
