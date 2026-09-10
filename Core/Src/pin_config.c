#include "pin_config.h"
uint32_t PinConfig_Validate(void)
{
    uint32_t e = 0;
    if (PINCFG_JY60_UART == PINCFG_ZDT_UART)
        e |= PINCFG_ERR_UART_CONFLICT;
    if (PINCFG_VOFA_UART == PINCFG_JY60_UART || PINCFG_VOFA_UART == PINCFG_ZDT_UART)
        e |= PINCFG_ERR_UART_CONFLICT;
    if (!PINCFG_ZDT_UART->hdmatx || PINCFG_ZDT_UART->hdmatx->Init.Mode != DMA_NORMAL)
        e |= PINCFG_ERR_ZDT_DMA;
    if ((SCB->CCR & SCB_CCR_DC_Msk) != 0)
        e |= PINCFG_ERR_DMA_CACHE;
    if (PINCFG_JY60_UART->Init.BaudRate != PINCFG_JY60_BAUDRATE)
        e |= PINCFG_ERR_JY60_BAUDRATE;
    if (PINCFG_ZDT_UART->Init.BaudRate != PINCFG_ZDT_BAUDRATE)
        e |= PINCFG_ERR_ZDT_BAUDRATE;
    if (!PINCFG_JY60_UART->hdmarx)
        e |= PINCFG_ERR_JY60_DMA_MISSING;
    else if (PINCFG_JY60_UART->hdmarx->Init.Mode != DMA_CIRCULAR)
        e |= PINCFG_ERR_JY60_DMA_NOT_CIRC;
    return e;
}
