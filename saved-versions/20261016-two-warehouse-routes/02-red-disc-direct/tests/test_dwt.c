#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

/* Replace only the hardware boundary; execute the production initializer. */
#define BSP_DWT_H
static struct { volatile uint32_t DEMCR; } debug_regs;
static struct { volatile uint32_t CTRL, CYCCNT, LAR; } dwt_regs;
#define CoreDebug (&debug_regs)
#define DWT (&dwt_regs)
#define CoreDebug_DEMCR_TRCENA_Msk (1UL << 24)
#define DWT_CTRL_CYCCNTENA_Msk 1UL
static bool counter_stuck;
static void hardware_step(void)
{
    if (!counter_stuck && DWT->LAR == 0xC5ACCE55U &&
        (CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) &&
        (DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk))
        DWT->CYCCNT++;
}
#define __NOP() hardware_step()
#define __DSB() ((void)0)
#define __ISB() ((void)0)
#include "../Core/Src/bsp_dwt.c"

int main(void)
{
    if (!DWT_Time_Init())
    {
        puts("FAIL: initializer must start a locked DWT counter");
        return 1;
    }
    counter_stuck = true;
    if (DWT_Time_Init())
    {
        puts("FAIL: initializer must reject a counter that does not advance");
        return 1;
    }
    puts("DWT initialization tests passed");
    return 0;
}
