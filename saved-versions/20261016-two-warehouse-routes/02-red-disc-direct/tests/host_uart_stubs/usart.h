#ifndef TEST_USART_H
#define TEST_USART_H
#include <stdint.h>
typedef struct
{
    int instance;
} UART_HandleTypeDef;
typedef enum
{
    HAL_OK,
    HAL_ERROR
} HAL_StatusTypeDef;
extern UART_HandleTypeDef test_uart, other_uart;
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *);
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t);
#endif
