#ifndef IMU_HEALTH_H
#define IMU_HEALTH_H
#include <stdbool.h>
#include <stdint.h>
typedef struct
{
    float last_yaw;
    uint32_t rejected, checksum_errors;
    uint8_t recovery_frames, consecutive_errors;
    bool initialized;
} ImuHealth;
void ImuHealth_ChecksumError(ImuHealth *h);
bool ImuHealth_Angle(ImuHealth *h, float yaw_deg, float gyro_dps, float interval_sec);
unsigned ImuHealth_Confidence(const ImuHealth *h);
#endif
