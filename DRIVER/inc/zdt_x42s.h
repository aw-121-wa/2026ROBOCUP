#ifndef ZDT_X42S_H
#define ZDT_X42S_H
#include <stdint.h>
#include <stddef.h>
#define ZDT_MULTI_SPEED_SIZE 37U
typedef enum
{
    ZDT_MULTI_COMMAND = 0,
    ZDT_LEGACY_SYNC = 1
} ZDT_CommandMode;
/* All APIs accept physical RPM; F6 wire units are 0.1 RPM (encoded x10).
 * Legacy clamps magnitude to 3000 RPM; multi rejects out-of-range input. */
void ZDT_BuildLegacySpeed(uint8_t frame[8], uint8_t id, int16_t rpm, uint8_t acceleration);
size_t ZDT_BuildMultiSpeed(uint8_t *frame, size_t capacity, const uint8_t ids[4],
                           const int16_t rpm[4], uint8_t acceleration);
/* Emm fixed-checksum F6, signed physical RPM, synchronous execution. */
void ZDT_BuildSpeed(uint8_t frame[8], uint8_t id, int16_t rpm, uint8_t acceleration);
void ZDT_BuildSync(uint8_t frame[4]);
#endif
