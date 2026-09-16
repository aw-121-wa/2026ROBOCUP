#ifndef PATH_PORTS_TEST_USART_H
#define PATH_PORTS_TEST_USART_H
#include <stdint.h>
typedef struct { unsigned gState; unsigned id; } UART_HandleTypeDef;
extern UART_HandleTypeDef huart4, huart6, huart7;
typedef enum { HAL_OK, HAL_ERROR } HAL_StatusTypeDef;
#define HAL_UART_STATE_READY 0U
#define GPIO_PIN_RESET 0U
#define GPIO_PIN_SET 1U
#define GPIO_PIN_0 1U
#define GPIO_PIN_1 2U
#define GPIO_PIN_3 8U
#define GPIO_PIN_8 256U
#define GPIO_PIN_10 1024U
#define GPIOB ((void *)2)
#define GPIOD ((void *)1)
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *);
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *, uint8_t *, uint16_t);
uint32_t HAL_GetTick(void);
unsigned HAL_GPIO_ReadPin(void *, uint16_t);
static inline uint32_t __get_PRIMASK(void) { return 0; }
static inline void __disable_irq(void) { }
static inline void __set_PRIMASK(uint32_t mask) { (void)mask; }
#endif
