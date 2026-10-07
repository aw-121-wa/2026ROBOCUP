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
    bool valid = isfinite(yaw) && isfinite(gyro) && fabsf(gyro) <= 720 &&
                 isfinite(dt) && dt >= 0 && dt <= IMU_LOST_TIMEOUT_MS * .001f;
    /* Decode intervals may be near zero for frames received together by DMA.
     * Use configured cadence for plausibility only; freshness uses real time. */
    float sample_dt = fmaxf(dt, IMU_SAMPLE_PERIOD_SEC);
    if (h->initialized)
    {
        float delta = remainderf(yaw - h->last_yaw, 360);
        valid = valid &&
                fabsf(delta - gyro * sample_dt) <= 8.0f + 0.25f * fabsf(gyro * sample_dt);
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
