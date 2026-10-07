#ifndef PATH_PORTS_H
#define PATH_PORTS_H
#include "usart.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "ball_inventory.h"
enum {
    INVENTORY_NO_BLOCK = 1, INVENTORY_BAD_BLOCK = 2, INVENTORY_CONFLICT = 4,
    INVENTORY_FULL = 8, INVENTORY_TURN_ERROR = 16
};
typedef struct
{
    uint32_t result, step, phase, ids, accepted_ids, link_reply, turn_reply, fault, session,
        sequence;
    uint32_t link_stage, link_error, gray;
    uint32_t rfid_fault; /* 1=RX init, 2=abort, 8=rearm, 16=UART, 64=RX overflow, 128=UID list full; nonfatal. */
    uint32_t ir_raw, settled; /* PD10 electrical level and chassis stop readiness. */
    uint32_t rfid_count; /* Separate from ZHY accepted_ids/PING flag. */
    uint32_t disc_action_done_index;
    uint32_t disc_rfid_confirmed_index;
    uint32_t disc_waiting_rfid;
    uint32_t disc_action_allowed;
    uint32_t point, phase_elapsed_ms; /* Read-only diagnostics. */
    uint32_t inventory_fault, inventory_occupied, inventory_slot, inventory_uncertain;
    uint32_t warehouse_code, warehouse_placed;
    uint32_t blue;
} PathDiagnostics;
extern volatile PathDiagnostics path_diagnostics;
/* RAM first-seen list of up to 64 distinct full four-byte UIDs.
 * capacity and return value are UID counts, not byte counts. Retained after completion/error/reset
 * of the link; cleared on next accepted PATH/DISC or MCU reset. */
size_t PathPorts_CopyIds(uint32_t *out, size_t capacity);
void PathPorts_CopyInventory(BallInventory *out);
void PathPorts_Init(void);
bool PathPorts_Start(void);
bool PathPorts_Ping(void);
bool PathPorts_Reset(void);
bool PathPorts_SelectSide(bool blue);
bool PathPorts_Disc(void);
bool PathPorts_Busy(void);
void PathPorts_Cancel(void);
void PathPorts_Tick(void);
void PathPorts_RxComplete(UART_HandleTypeDef *uart);
void PathPorts_Error(UART_HandleTypeDef *uart);
#endif
