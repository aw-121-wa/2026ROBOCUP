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

#define PINCFG_ZDT_UART (&huart3)
#define PINCFG_ZDT_BAUDRATE 921600U

/* -------------------- Host telemetry (UART5: PC12 TX / PD2 RX) -------------------- */
#define PINCFG_VOFA_UART (&huart5)
#define PINCFG_VOFA_BAUDRATE 115200U

/* -------------------- WIT JY60 -------------------- */

#define PINCFG_JY60_UART (&huart2)
#define PINCFG_JY60_BAUDRATE 9600U

/* Mission peripherals: old turntable/RFID wiring, separate RDK link. */
#define PINCFG_RDK_BAUDRATE 115200U
#define PINCFG_RDK_UART (&huart4) /* PC10 TX / PC11 RX, 115200 */
#define PINCFG_TURNTABLE_UART (&huart6) /* PC6 TX / PC7 RX, 115200 */
#define PINCFG_RFID_UART (&huart7) /* PE7 RX / PE8 TX, RFID factory baud 9600 */

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
    PINCFG_OK = 0x00000000U,

    PINCFG_ERR_UART_CONFLICT = 0x00000001U,
    PINCFG_ERR_ZDT_BAUDRATE = 0x00000002U,

    PINCFG_ERR_JY60_BAUDRATE = 0x00000004U,
    PINCFG_ERR_JY60_DMA_MISSING = 0x00000008U,
    PINCFG_ERR_JY60_DMA_NOT_CIRC = 0x00000010U,
    PINCFG_ERR_ZDT_DMA = 0x00000020U,
    PINCFG_ERR_DMA_CACHE = 0x00000040U,
    PINCFG_ERR_RDK_BAUDRATE = 0x00000080U,

} PinConfigError_t;

uint32_t PinConfig_Validate(void);

#endif
