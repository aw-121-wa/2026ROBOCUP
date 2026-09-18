#include "main.h"
#include "path_session.h"
#include <stdio.h>
#include <string.h>
TestRcc test_rcc;
TestRng test_rng;
int test_rng_clock;
static unsigned ticks, configured;
static int clock_fail;
uint32_t HAL_GetTick(void)
{
    return ticks++;
}
HAL_StatusTypeDef HAL_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef *c)
{
    if (c->PeriphClockSelection != RCC_PERIPHCLK_CLK48 ||
        c->Clk48ClockSelection != RCC_CLK48SOURCE_PLLSAIP || c->PLLSAI.PLLSAIN != 192 ||
        c->PLLSAI.PLLSAIP != RCC_PLLSAIP_DIV8)
        return HAL_ERROR;
    configured++;
    return clock_fail ? HAL_ERROR : HAL_OK;
}
#define CHECK(x)                                                                                   \
    do                                                                                             \
    {                                                                                              \
        if (!(x))                                                                                  \
        {                                                                                          \
            printf("FAIL %d: %s\n", __LINE__, #x);                                                 \
            return 1;                                                                              \
        }                                                                                          \
    } while (0)
int main(int argc, char **argv)
{
    CHECK(argc == 2);
    test_rcc.PLLCFGR = RCC_PLLCFGR_PLLSRC_HSE | 4;
    test_rng.SR = RNG_SR_DRDY;
    test_rng.DR = 0x12345678;
    if (!strcmp(argv[1], "clock"))
        clock_fail = 1;
    if (!strcmp(argv[1], "input"))
        test_rcc.PLLCFGR = RCC_PLLCFGR_PLLSRC_HSE | 8;
    if (!strcmp(argv[1], "used"))
        test_rcc.CR = RCC_CR_PLLSAION;
    if (!strcmp(argv[1], "seed"))
        test_rng.SR |= RNG_SR_SECS;
    if (!strcmp(argv[1], "timeout"))
        test_rng.SR = 0;
    if (!strcmp(argv[1], "zero"))
        test_rng.DR = 0;
    uint32_t nonce = PathSession_Create();
    if (!strcmp(argv[1], "good"))
    {
        CHECK(nonce == 0x12345678);
        test_rng.DR = 0x87654321;
        CHECK(PathSession_Create() == nonce && configured == 1);
    }
    else
        CHECK(nonce == 0);
    CHECK(!test_rng_clock && test_rng.CR == 0);
    CHECK(ticks < 100);
    puts("session nonce test passed");
    return 0;
}
