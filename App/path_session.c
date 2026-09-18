#include "path_session.h"
#include "main.h"

uint32_t PathSession_Create(void)
{
    static uint32_t session;
    static uint8_t attempted;
    if (attempted)
        return session;
    attempted = 1;

    /* Current board: HSE 8 MHz / PLLM 4 = 2 MHz. Configure only the unused
     * auxiliary PLLSAI: 2 MHz * 192 / 8 = 48 MHz for RNG. SYSCLK/APB and
     * the main PLL (hence motor/IMU baud rates and control timing) stay intact.
     * Refuse an unexpected clock tree rather than silently reconfiguring it.
     */
    uint32_t divider = RCC->PLLCFGR & RCC_PLLCFGR_PLLM;
    if (!(RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC_HSE) || divider == 0 ||
        HSE_VALUE / divider != 2000000U || (RCC->CR & RCC_CR_PLLSAION))
        return 0;
    RCC_PeriphCLKInitTypeDef clock = {0};
    clock.PeriphClockSelection = RCC_PERIPHCLK_CLK48;
    clock.Clk48ClockSelection = RCC_CLK48SOURCE_PLLSAIP;
    clock.PLLSAI.PLLSAIN = 192;
    clock.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV8;
    if (HAL_RCCEx_PeriphCLKConfig(&clock) != HAL_OK)
        return 0;

    __HAL_RCC_RNG_CLK_ENABLE();
    RNG->CR = RNG_CR_RNGEN;
    uint32_t start = HAL_GetTick();
    while ((uint32_t)(HAL_GetTick() - start) < 10U)
    {
        uint32_t status = RNG->SR;
        if (status & (RNG_SR_CECS | RNG_SR_SECS | RNG_SR_CEIS | RNG_SR_SEIS))
            break;
        if (status & RNG_SR_DRDY)
        {
            session = RNG->DR;
            if (session != 0)
                break;
        }
    }
    RNG->CR = 0;
    __HAL_RCC_RNG_CLK_DISABLE();
    return session;
}
