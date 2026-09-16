#ifndef PATH_PORTS_H
#define PATH_PORTS_H
#include "usart.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
typedef struct
{
    uint32_t result, step, phase, ids, accepted_ids, link_reply, turn_reply, fault, session,
        sequence;
    uint32_t link_stage, link_error, gray;
    uint32_t rfid_fault; /* 1=RX init, 2=abort, 8=rearm, 16=UART, 64=RX overflow, 128=UID list full; nonfatal. */
    uint32_t rfid_count; /* Separate from ZHY accepted_ids/PING flag. */
} PathDiagnostics;
extern volatile PathDiagnostics path_diagnostics;
/* RAM first-seen list of up to 64 distinct full four-byte UIDs.
 * capacity and return value are UID counts, not byte counts. Retained after completion/error/reset
 * of the link; cleared on next accepted PATH/DISC or MCU reset. */
size_t PathPorts_CopyIds(uint32_t *out, size_t capacity);
void PathPorts_Init(void);
bool PathPorts_Start(void);
bool PathPorts_Ping(void);
bool PathPorts_Reset(void);
bool PathPorts_Disc(void);
bool PathPorts_Busy(void);
void PathPorts_Cancel(void);
void PathPorts_Tick(void);
void PathPorts_RxComplete(UART_HandleTypeDef *uart);
void PathPorts_Error(UART_HandleTypeDef *uart);
#endif
