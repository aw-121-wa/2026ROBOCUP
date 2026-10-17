#ifndef HEADING_H
#define HEADING_H
#include <stdbool.h>
#include <stdint.h>
typedef struct
{
    float yaw_rad;
    float last_gyro_rad_s;
    uint32_t angle_frame;
    bool ready;
} HeadingEstimator;
void HeadingEstimator_Reset(HeadingEstimator *estimator);
bool HeadingEstimator_Update(HeadingEstimator *estimator, float angle_rad, uint32_t angle_frame,
                             float gyro_rad_s, float dt, bool usable);
float Heading_Update(float error_rad, float gyro_rad_s, float dt, float kp, float ki, float kg,
                     float limit, float *integral);
/* One request selects exactly one yaw-output owner per cycle. */
typedef enum {
    HEADING_OFF, HEADING_FIXED, HEADING_TRAVEL, HEADING_DYNAMIC,
    HEADING_ROTATE, HEADING_PRECISION, HEADING_HOLD, HEADING_MANUAL
} HeadingMode;
typedef struct { float kp, ki, damping, limit; } HeadingControlConfig;
typedef struct {
    HeadingMode mode;
    float error, gyro, feedforward;
    float limit; /* Rotation-specific limit; other modes use the profile. */
} HeadingRequest;
float HeadingControl_Update(const HeadingRequest *request, const HeadingControlConfig *config,
                            float dt, float *integral);
#endif
