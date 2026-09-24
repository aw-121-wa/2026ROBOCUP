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
#endif
