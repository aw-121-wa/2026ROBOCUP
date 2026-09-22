#ifndef STAIR_HEADING_H
#define STAIR_HEADING_H
#include <math.h>
#define STAIR_HEADING_TOLERANCE_DEG 0.2f
#define STAIR_HEADING_STOP_TOLERANCE_DEG 0.15f
#define STAIR_HEADING_STABLE_MS 100U
#define STAIR_HEADING_TIMEOUT_MS 300U
/* Sensor angle zero; wrap the measured error to choose the shortest correction. */
static inline float StairHeading_Error(float yaw_deg)
{
    return -remainderf(yaw_deg, 360.0f);
}
#endif
