#include "zdt_x42s.h"
void ZDT_BuildLegacySpeed(uint8_t p[8], uint8_t id, int16_t rpm, uint8_t acceleration)
{
    uint16_t magnitude = (uint16_t)(rpm < 0 ? -(int32_t)rpm : rpm);
    /* Public input is physical RPM; configured driver uses 0.1 RPM per unit.
     * Clamp the void legacy API before scaling to avoid uint16_t overflow. */
    if (magnitude > 3000U)
        magnitude = 3000U;
    magnitude = (uint16_t)(magnitude * 10U);
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

void ZDT_BuildSpeed(uint8_t p[8], uint8_t id, int16_t rpm, uint8_t acceleration)
{
    ZDT_BuildLegacySpeed(p, id, rpm, acceleration);
}

/* Manual V1.0.4 pp49-50: BE16 total length includes envelope and all checksums. */
size_t ZDT_BuildMultiSpeed(uint8_t *p, size_t capacity, const uint8_t ids[4], const int16_t rpm[4],
                           uint8_t acceleration)
{
    if (!p || !ids || !rpm || capacity < ZDT_MULTI_SPEED_SIZE)
        return 0;
    for (int i = 0; i < 4; ++i)
    {
        if (ids[i] == 0 || ids[i] > 247 || rpm[i] < -3000 || rpm[i] > 3000)
            return 0;
        for (int j = 0; j < i; ++j)
            if (ids[i] == ids[j])
                return 0;
    }
    p[0] = 0;
    p[1] = 0xAA;
    p[2] = 0;
    p[3] = ZDT_MULTI_SPEED_SIZE;
    for (int i = 0; i < 4; ++i)
    {
        ZDT_BuildLegacySpeed(&p[4 + i * 8], ids[i], rpm[i], acceleration);
        p[4 + i * 8 + 6] = 0; /* Immediate within the aggregate command. */
    }
    p[36] = 0x6B;
    return ZDT_MULTI_SPEED_SIZE;
}
