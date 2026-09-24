#include "host_uart.h"
#include "pin_config.h"

/* ISR owns head, task owns tail; task snapshots are protected from IRQ writes. */
static volatile uint8_t ring[HOST_UART_CAPACITY];
static volatile uint16_t head, tail;
static volatile bool rx_error;
static uint8_t rx_byte;

bool HostUart_Init(void)
{
    head = tail = 0;
    rx_error = false;
    return HAL_UART_Receive_IT(PINCFG_VOFA_UART, &rx_byte, 1) == HAL_OK;
}
void HostUart_RxComplete(UART_HandleTypeDef *uart)
{
    if (uart != PINCFG_VOFA_UART)
        return;
    uint16_t next = (uint16_t)((head + 1U) % HOST_UART_CAPACITY);
    if (next == tail)
        rx_error = true;
    else if (!rx_error)
    {
        ring[head] = rx_byte;
        head = next;
    }
    if (HAL_UART_Receive_IT(uart, &rx_byte, 1) != HAL_OK)
        rx_error = true;
}
void HostUart_Error(UART_HandleTypeDef *uart)
{
    if (uart == PINCFG_VOFA_UART)
        rx_error = true;
}
int HostUart_Read(uint8_t *data, size_t capacity)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    if (rx_error)
    {
        /* RX-only abort: leave the ongoing JustFloat transmission untouched. */
        HAL_StatusTypeDef status = HAL_UART_AbortReceive(PINCFG_VOFA_UART);
        tail = head;
        rx_error = false;
        if (status != HAL_OK || HAL_UART_Receive_IT(PINCFG_VOFA_UART, &rx_byte, 1) != HAL_OK)
            rx_error = true;
        __set_PRIMASK(mask);
        return -1;
    }
    size_t count = 0;
    while (tail != head && count < capacity)
    {
        data[count++] = ring[tail];
        tail = (uint16_t)((tail + 1U) % HOST_UART_CAPACITY);
    }
    __set_PRIMASK(mask);
    return (int)count;
}
void HostUart_Flush(void)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    tail = head;
    __set_PRIMASK(mask);
}
