#ifndef CHASSIS_TEST_DWT_H
#define CHASSIS_TEST_DWT_H
#include <stdint.h>
#include <stdbool.h>
extern uint32_t fake_cycle;
static inline bool DWT_Time_Init(void) { return true; }
static inline uint32_t DWT_GetCycle(void) { return fake_cycle; }
static inline float DWT_DeltaSec(uint32_t a,uint32_t b) { return (a-b)*0.000001f; }
#endif
