#include "host_uart.h"
#include <stdio.h>
#include <string.h>
#define CHECK(x)                                                                                   \
    do                                                                                             \
    {                                                                                              \
        if (!(x))                                                                                  \
        {                                                                                          \
            printf("FAIL line %d: %s\n", __LINE__, #x);                                            \
            return 1;                                                                              \
        }                                                                                          \
    } while (0)
UART_HandleTypeDef test_uart, other_uart;
static uint8_t *receive_buffer;
static bool fail_receive;
static uint32_t mask;
static unsigned aborts;
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *uart, uint8_t *data, uint16_t size)
{
    if (uart != &test_uart || size != 1 || fail_receive)
        return HAL_ERROR;
    receive_buffer = data;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *uart)
{
    if (uart != &test_uart)
        return HAL_ERROR;
    ++aborts;
    return HAL_OK;
}
uint32_t __get_PRIMASK(void)
{
    return mask;
}
void __disable_irq(void)
{
    mask = 1;
}
void __set_PRIMASK(uint32_t value)
{
    mask = value;
}
static void inject(char c)
{
    *receive_buffer = (uint8_t)c;
    HostUart_RxComplete(&test_uart);
}
int main(void)
{
    uint8_t bytes[HOST_UART_CAPACITY];
    CHECK(HostUart_Init());
    inject('A');
    inject('R');
    inject('M');
    inject('\n');
    CHECK(HostUart_Read(bytes, sizeof(bytes)) == 4);
    CHECK(!memcmp(bytes, "ARM\n", 4) && mask == 0);
    CHECK(HostUart_Read(bytes, sizeof(bytes)) == 0);
    for (int i = 0; i < 600; ++i)
    {
        inject('x');
        CHECK(HostUart_Read(bytes, 1) == 1 && bytes[0] == 'x');
    }
    inject('a');
    HostUart_Flush();
    CHECK(HostUart_Read(bytes, sizeof(bytes)) == 0);
    HostUart_Error(&other_uart);
    CHECK(HostUart_Read(bytes, sizeof(bytes)) == 0);
    for (unsigned i = 0; i < HOST_UART_CAPACITY; ++i)
        inject('x');
    CHECK(HostUart_Read(bytes, sizeof(bytes)) == -1);
    CHECK(aborts == 1 && mask == 0);
    CHECK(HostUart_Read(bytes, sizeof(bytes)) == 0);
    inject('S');
    CHECK(HostUart_Read(bytes, sizeof(bytes)) == 1 && bytes[0] == 'S');
    inject('A');
    HostUart_Error(&test_uart);
    CHECK(HostUart_Read(bytes, sizeof(bytes)) == -1);
    CHECK(HostUart_Read(bytes, sizeof(bytes)) == 0);
    fail_receive = true;
    inject('x');
    CHECK(HostUart_Read(bytes, sizeof(bytes)) == -1);
    CHECK(HostUart_Read(bytes, sizeof(bytes)) == -1);
    fail_receive = false;
    CHECK(HostUart_Read(bytes, sizeof(bytes)) == -1);
    inject('B');
    mask = 1;
    CHECK(HostUart_Read(bytes, sizeof(bytes)) == 1 && bytes[0] == 'B' && mask == 1);
    puts("Host UART transport tests passed");
    return 0;
}
