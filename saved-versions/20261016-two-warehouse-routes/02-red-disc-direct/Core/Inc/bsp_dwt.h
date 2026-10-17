#ifndef BSP_DWT_H
#define BSP_DWT_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/*
 * 初始化 Cortex-M7 DWT CYCCNT
 */
bool DWT_Time_Init(void);
#define BSP_DWT_Init DWT_Time_Init
#define BSP_DWT_GetCycle DWT_GetCycle
#define BSP_DWT_DeltaSec DWT_DeltaSec
#define BSP_DWT_MsToCycles DWT_MsToCycles
#define BSP_DWT_ElapsedCycles DWT_ElapsedCycles
#define BSP_DWT_CyclesToMs DWT_CyclesToMs

/*
 * 当前 CPU cycle
 */
static inline uint32_t DWT_GetCycle(void)
{
    return DWT->CYCCNT;
}

/*
 * 两个 cycle 时间戳之间经过多少 cycle
 *
 * uint32_t 无符号减法天然支持 CYCCNT 回绕。
 */
static inline uint32_t DWT_ElapsedCycles(uint32_t now, uint32_t last)
{
    return now - last;
}

/*
 * cycle -> 秒
 */
static inline float DWT_CyclesToSec(uint32_t cycles)
{
    return (float)cycles / (float)SystemCoreClock;
}

/*
 * 两个时间戳的秒数差
 */
static inline float DWT_DeltaSec(uint32_t now, uint32_t last)
{
    return DWT_CyclesToSec(now - last);
}

/*
 * us -> cycle
 */
static inline uint32_t DWT_UsToCycles(uint32_t us)
{
    return (uint32_t)(((uint64_t)SystemCoreClock * (uint64_t)us) / 1000000ULL);
}

/*
 * ms -> cycle
 */
static inline uint32_t DWT_MsToCycles(uint32_t ms)
{
    return (uint32_t)(((uint64_t)SystemCoreClock * (uint64_t)ms) / 1000ULL);
}

/*
 * elapsed cycle -> us
 */
static inline uint32_t DWT_CyclesToUs(uint32_t cycles)
{
    return (uint32_t)(((uint64_t)cycles * 1000000ULL) / (uint64_t)SystemCoreClock);
}

/*
 * elapsed cycle -> ms
 */
static inline uint32_t DWT_CyclesToMs(uint32_t cycles)
{
    return (uint32_t)(((uint64_t)cycles * 1000ULL) / (uint64_t)SystemCoreClock);
}

#endif
