#ifndef PATH_PORTS_H
#define PATH_PORTS_H
#include "usart.h"
#include <stdbool.h>
#include <stdint.h>
typedef struct
{
    uint32_t result, step, phase, ids, accepted_ids, link_reply, turn_reply, fault, session,
        sequence;
} PathDiagnostics;
extern volatile PathDiagnostics path_diagnostics;
void PathPorts_Init(void);
bool PathPorts_Start(void);
bool PathPorts_Busy(void);
void PathPorts_Cancel(void);
void PathPorts_Tick(void);
void PathPorts_RxComplete(UART_HandleTypeDef *uart);
void PathPorts_Error(UART_HandleTypeDef *uart);
#endif
