#ifndef STAIR_HEADING_H
#define STAIR_HEADING_H
#include <math.h>
#include "heading_tuning.h"
#define STAIR_MAP_TARGET_DEG 180.0f
/* Both field sides use the same map heading, without a side offset. */
#define STAIR_TARGET_DEG(blue) STAIR_MAP_TARGET_DEG
#define STAIR_HEADING_KP 4.0f
#define STAIR_HEADING_TOLERANCE_DEG HEADING_STATIC_TOLERANCE_DEG
#define STAIR_HEADING_STOP_TOLERANCE_DEG HEADING_STATIC_TOLERANCE_DEG
#define STAIR_HEADING_STABLE_MS 100U
#define STAIR_HEADING_TIMEOUT_MS 1000U
/* Sensor angle zero; wrap the measured error to choose the shortest correction. */
static inline float StairHeading_Error(float yaw_deg)
{
    return -remainderf(yaw_deg, 360.0f);
}
#endif
