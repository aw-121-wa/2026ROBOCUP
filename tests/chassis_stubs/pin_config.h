#ifndef CHASSIS_TEST_PIN_H
#define CHASSIS_TEST_PIN_H
#include "usart.h"
extern UART_HandleTypeDef huart3;
#define PINCFG_ZDT_UART (&huart3)
#define PINCFG_VOFA_UART (&huart4)
#define PINCFG_ZDT_BAUDRATE 115200
#define PINCFG_OK 0
static inline int PinConfig_Validate(void) { return 0; }
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *, uint8_t *, uint16_t);
#endif
