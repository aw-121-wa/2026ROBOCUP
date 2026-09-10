#include "bsp_dwt.h"

bool DWT_Time_Init(void)
{
    /*
     * 允许访问 DWT
     */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    /*
     * 清零 cycle counter
     */
    DWT->CYCCNT = 0U;

    /*
     * 开启 CYCCNT
     */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    /*
     * 保证寄存器操作完成
     */
    __DSB();
    __ISB();

    /*
     * 简单验证计数器是否运行
     */
    uint32_t start = DWT->CYCCNT;

    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();

    uint32_t end = DWT->CYCCNT;

    return (end != start);
}