#ifndef FORWARD_COMP_H
#define FORWARD_COMP_H

#include <stdbool.h>
#include <math.h>

typedef struct
{
    float vy_original;
    float forward_comp_vy;
    float vy_final;
} ForwardCompResult;

static inline bool ForwardComp_ConfigValid(float gain)
{
    return isfinite(gain);
}

static inline ForwardCompResult ForwardComp_Apply(float vx, float vy_requested,
                                                   float lateral_direction, float left_gain,
                                                   float right_gain, float gain)
{
    ForwardCompResult result;
    result.vy_original = vy_requested * (vy_requested >= 0.0f ? left_gain : right_gain);
    result.forward_comp_vy =
        (vx > 0.0f && lateral_direction == 0.0f) ? gain * vx : 0.0f;
    result.vy_final = result.forward_comp_vy == 0.0f
                          ? result.vy_original
                          : result.vy_original + result.forward_comp_vy;
    return result;
}

#endif
