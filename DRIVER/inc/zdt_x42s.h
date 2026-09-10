#ifndef ZDT_X42S_H
#define ZDT_X42S_H
#include <stdint.h>
/* Emm fixed-checksum F6, signed physical RPM, synchronous execution. */
void ZDT_BuildSpeed(uint8_t frame[8], uint8_t id, int16_t rpm, uint8_t acceleration);
void ZDT_BuildSync(uint8_t frame[4]);
#endif
