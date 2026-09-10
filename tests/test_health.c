#include "imu_health.h"
#include <assert.h>
int main(void)
{
    ImuHealth h = {0};
    for (int i = 0; i < 5; ++i)
        assert(ImuHealth_Angle(&h, 179, 0, 0.05f));
    assert(ImuHealth_Confidence(&h) == 2);
    assert(ImuHealth_Angle(&h, -179, 40, 0.05f));
    assert(!ImuHealth_Angle(&h, 0, 0, 0.05f));
    assert(ImuHealth_Confidence(&h) == 0);
    for (int i = 0; i < 5; ++i)
        assert(ImuHealth_Angle(&h, 0, 0, 0.05f));
    assert(ImuHealth_Confidence(&h) == 2);
    ImuHealth_ChecksumError(&h);
    ImuHealth_ChecksumError(&h);
    ImuHealth_ChecksumError(&h);
    assert(ImuHealth_Confidence(&h) == 0);
    assert(!ImuHealth_Angle(&h, 0, 0, 1));
    return 0;
}
