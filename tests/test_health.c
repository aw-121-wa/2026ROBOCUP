#include "imu_health.h"
#include <assert.h>
#include <math.h>
int main(void)
{
    ImuHealth fast = {0};
    for (int i=0;i<10;i++) assert(ImuHealth_Angle(&fast, i*.5f,100,.005f));
    assert(ImuHealth_Confidence(&fast)==2);
    /* DMA batching gives adjacent decoded frames almost identical timestamps. */
    assert(ImuHealth_Angle(&fast,5,100,0));
    assert(ImuHealth_Angle(&fast,5.5f,100,.00001f));
    assert(!ImuHealth_Angle(&fast,60,0,.005f));
    assert(!ImuHealth_Angle(&fast,60,0,NAN));
    assert(!ImuHealth_Angle(&fast,60,0,-.001f));
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
