#include "zdt_x42s.h"
#include <assert.h>
#include <string.h>
int main(void)
{
    uint8_t p[40];
    memset(p, 0xCC, sizeof(p));
    const uint8_t ids[4] = {2, 1, 3, 4};
    const int16_t rpm[4] = {-300, 100, 0, 3000};
    const uint8_t expected[37] = {
        0, 0xAA, 0, 0x25,
        2, 0xF6, 1, 0x0B, 0xB8, 0, 0, 0x6B,
        1, 0xF6, 0, 0x03, 0xE8, 0, 0, 0x6B,
        3, 0xF6, 0, 0, 0, 0, 0, 0x6B,
        4, 0xF6, 0, 0x75, 0x30, 0, 0, 0x6B,
        0x6B};
    assert(ZDT_BuildMultiSpeed(p, sizeof(p), ids, rpm, 0) == 37);
    assert(memcmp(p, expected, 37) == 0 && p[37] == 0xCC);
    assert(ZDT_BuildMultiSpeed(p, 36, ids, rpm, 0) == 0);
    const uint8_t duplicate[4] = {1, 1, 3, 4};
    assert(ZDT_BuildMultiSpeed(p, sizeof(p), duplicate, rpm, 0) == 0);
    uint8_t legacy[8];
    ZDT_BuildLegacySpeed(legacy, 2, -300, 0);
    assert(legacy[6] == 1 && legacy[7] == 0x6B);
    assert(legacy[2] == 1 && legacy[3] == 0x0B && legacy[4] == 0xB8);
    ZDT_BuildLegacySpeed(legacy, 1, 100, 0);
    assert(legacy[2] == 0 && legacy[3] == 3 && legacy[4] == 0xE8);
    ZDT_BuildLegacySpeed(legacy, 1, INT16_MIN, 0);
    assert(legacy[2] == 1 && legacy[3] == 0x75 && legacy[4] == 0x30);
    const int16_t invalid[4] = {3001, 0, 0, 0};
    assert(ZDT_BuildMultiSpeed(p, sizeof(p), ids, invalid, 0) == 0);
    const int16_t fine[4] = {1, -3, 0, 1234};
    assert(ZDT_BuildMultiSpeedDeci(p, sizeof(p), ids, fine, 0) == 37);
    assert(p[6]==0 && p[7]==0 && p[8]==1); /* +0.1 RPM */
    assert(p[14]==1 && p[15]==0 && p[16]==3); /* -0.3 RPM */
    assert(p[31]==4 && p[32]==0xD2); /* 123.4 RPM */
    ZDT_BuildLegacySpeedDeci(legacy, 2, -1, 0);
    assert(legacy[2]==1 && legacy[3]==0 && legacy[4]==1 && legacy[6]==1);
    const int16_t fine_bad[4] = {30001,0,0,0};
    assert(ZDT_BuildMultiSpeedDeci(p,sizeof(p),ids,fine_bad,0)==0);
    return 0;
}
