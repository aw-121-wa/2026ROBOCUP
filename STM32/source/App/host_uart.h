#ifndef HOST_UART_H
#define HOST_UART_H
#include "usart.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#define HOST_UART_CAPACITY 256U
bool HostUart_Init(void);
/* Task only. -1 means RX loss: discard command state and stop/disarm. */
int HostUart_Read(uint8_t *data, size_t capacity);
void HostUart_Flush(void);
void HostUart_RxComplete(UART_HandleTypeDef *uart);
void HostUart_Error(UART_HandleTypeDef *uart);
#endif
