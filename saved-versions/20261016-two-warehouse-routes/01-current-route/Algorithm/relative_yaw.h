#ifndef RELATIVE_YAW_H
#define RELATIVE_YAW_H
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
typedef struct
{
    float yaw_rad, previous_deg;
    uint32_t frame;
    bool ready;
} RelativeYaw;
static inline void RelativeYaw_Update(RelativeYaw *p, float degrees, uint32_t frame,
                                      bool usable, bool allow_reference)
{
    if (!usable || !isfinite(degrees))
    {
        *p = (RelativeYaw){0};
        return;
    }
    if (!p->ready)
    {
        if (allow_reference)
            *p = (RelativeYaw){.previous_deg = degrees, .frame = frame, .ready = true};
        return;
    }
    if (p->frame == frame)
        return;
    float delta = remainderf(degrees - p->previous_deg, 360.0f);
    p->yaw_rad = remainderf(p->yaw_rad + delta * 0.017453292519943295f,
                           6.283185307179586f);
    p->previous_deg = degrees;
    p->frame = frame;
}
#endif
