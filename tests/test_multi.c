#include "zdt_x42s.h"
#include <assert.h>
#include <string.h>
int main(void)
{
    uint8_t p[40];
    memset(p, 0xCC, sizeof(p));
    const uint8_t ids[4] = {2, 1, 3, 4};
    const int16_t rpm[4] = {-300, 100, 0, 3000};
    const uint8_t expected[37] = {0,    0xAA, 0, 0x25, 2, 0xF6, 1,    1, 0x2C, 0,    0,   0x6B, 1,
                                  0xF6, 0,    0, 100,  0, 0,    0x6B, 3, 0xF6, 0,    0,   0,    0,
                                  0,    0x6B, 4, 0xF6, 0, 0x0B, 0xB8, 0, 0,    0x6B, 0x6B};
    assert(ZDT_BuildMultiSpeed(p, sizeof(p), ids, rpm, 0) == 37);
    assert(memcmp(p, expected, 37) == 0 && p[37] == 0xCC);
    assert(ZDT_BuildMultiSpeed(p, 36, ids, rpm, 0) == 0);
    const uint8_t duplicate[4] = {1, 1, 3, 4};
    assert(ZDT_BuildMultiSpeed(p, sizeof(p), duplicate, rpm, 0) == 0);
    uint8_t legacy[8];
    ZDT_BuildLegacySpeed(legacy, 2, -300, 0);
    assert(legacy[6] == 1 && legacy[7] == 0x6B);
    return 0;
}
