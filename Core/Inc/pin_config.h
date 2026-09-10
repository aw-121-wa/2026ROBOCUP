#ifndef PIN_CONFIG_H
#define PIN_CONFIG_H

#include "main.h"
#include "usart.h"
#include <stdint.h>

/*
 * ============================================================
 * Logical peripheral mapping
 * ============================================================
 *
 * 应用层禁止直接使用 huart1 / huart3。
 * 所有模块必须使用下面的 PINCFG_xxx。
 *
 * 更换外设实例时，只修改这里。
 */

/* -------------------- ZDT X42S -------------------- */

#define PINCFG_ZDT_UART              (&huart3)
#define PINCFG_ZDT_BAUDRATE          921600U


/* -------------------- WIT JY60 -------------------- */

#define PINCFG_JY60_UART             (&huart2)
#define PINCFG_JY60_BAUDRATE         9600U


/*
 * ============================================================
 * Optional GPIO mapping
 * ============================================================
 *
 * 后面如果增加IO口的使用，可以在这里定义。
 *
 * 都统一定义在这里。
 *
 * 示例：
 *
 * #define PINCFG_LED_GPIO_PORT      LED_GPIO_Port
 * #define PINCFG_LED_GPIO_PIN       LED_Pin
 */


/*
 * ============================================================
 * Configuration validation
 * ============================================================
 */

typedef enum
{
    PINCFG_OK                     = 0x00000000U,

    PINCFG_ERR_UART_CONFLICT      = 0x00000001U,
    PINCFG_ERR_ZDT_BAUDRATE       = 0x00000002U,

    PINCFG_ERR_JY60_BAUDRATE      = 0x00000004U,
    PINCFG_ERR_JY60_DMA_MISSING   = 0x00000008U,
    PINCFG_ERR_JY60_DMA_NOT_CIRC  = 0x00000010U,

} PinConfigError_t;


uint32_t PinConfig_Validate(void);


#endif