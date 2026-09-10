#include "imu_health.h"
#include <math.h>
void ImuHealth_ChecksumError(ImuHealth *h)
{
    h->checksum_errors++;
    if (h->consecutive_errors < 255)
        h->consecutive_errors++;
    h->recovery_frames = 0;
}
bool ImuHealth_Angle(ImuHealth *h, float yaw, float gyro, float dt)
{
    bool valid = isfinite(yaw) && isfinite(gyro) && fabsf(gyro) <= 720;
    if (h->initialized)
    {
        float delta = remainderf(yaw - h->last_yaw, 360);
        valid = valid && dt >= 0.015f && dt <= 0.18f &&
                fabsf(delta - gyro * dt) <= 8.0f + 0.25f * fabsf(gyro * dt);
    }
    if (isfinite(yaw))
    {
        h->last_yaw = yaw;
        h->initialized = true;
    }
    if (!valid)
    {
        h->rejected++;
        h->recovery_frames = 0;
        h->consecutive_errors = 3;
        return false;
    }
    h->consecutive_errors = 0;
    if (h->recovery_frames < 5)
        h->recovery_frames++;
    return true;
}
unsigned ImuHealth_Confidence(const ImuHealth *h)
{
    if (h->consecutive_errors >= 3 || !h->initialized)
        return 0;
    return h->recovery_frames >= 5 ? 2 : 1;
}
