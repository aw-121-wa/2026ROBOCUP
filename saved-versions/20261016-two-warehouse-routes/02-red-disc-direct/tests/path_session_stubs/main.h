#ifndef PATH_SESSION_TEST_MAIN_H
#define PATH_SESSION_TEST_MAIN_H
#include <stdint.h>
#define HSE_VALUE 8000000U
#define RCC_PLLCFGR_PLLM 63U
#define RCC_PLLCFGR_PLLSRC_HSE (1U << 22)
#define RCC_CR_PLLSAION (1U << 28)
#define RCC_PERIPHCLK_CLK48 0x200000U
#define RCC_CLK48SOURCE_PLLSAIP 1U
#define RCC_PLLSAIP_DIV8 3U
#define RNG_CR_RNGEN 4U
#define RNG_SR_DRDY 1U
#define RNG_SR_CECS 2U
#define RNG_SR_SECS 4U
#define RNG_SR_CEIS 32U
#define RNG_SR_SEIS 64U
typedef struct { uint32_t PLLCFGR, CR; } TestRcc;
typedef struct { uint32_t CR, SR, DR; } TestRng;
extern TestRcc test_rcc;
extern TestRng test_rng;
extern int test_rng_clock;
#define RCC (&test_rcc)
#define RNG (&test_rng)
#define __HAL_RCC_RNG_CLK_ENABLE() (test_rng_clock = 1)
#define __HAL_RCC_RNG_CLK_DISABLE() (test_rng_clock = 0)
typedef struct {
    uint32_t PeriphClockSelection, Clk48ClockSelection;
    struct { uint32_t PLLSAIN, PLLSAIP; } PLLSAI;
} RCC_PeriphCLKInitTypeDef;
typedef enum { HAL_OK, HAL_ERROR } HAL_StatusTypeDef;
HAL_StatusTypeDef HAL_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef *config);
uint32_t HAL_GetTick(void);
#endif
