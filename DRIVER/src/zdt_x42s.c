#include "zdt_x42s.h"
void ZDT_BuildSpeed(uint8_t p[8], uint8_t id, int16_t rpm, uint8_t acceleration)
{
    uint16_t magnitude = (uint16_t)(rpm < 0 ? -(int32_t)rpm : rpm);
    p[0] = id;
    p[1] = 0xF6;
    p[2] = rpm < 0;
    p[3] = (uint8_t)(magnitude >> 8);
    p[4] = (uint8_t)magnitude;
    p[5] = acceleration;
    p[6] = 1;
    p[7] = 0x6B;
}
void ZDT_BuildSync(uint8_t p[4])
{
    p[0] = 0;
    p[1] = 0xFF;
    p[2] = 0x66;
    p[3] = 0x6B;
}
