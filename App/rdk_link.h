#ifndef RDK_LINK_H
#define RDK_LINK_H
#include "path_mission.h"
#include <stddef.h>
typedef bool (*RdkTransmit)(void *context, const char *data, size_t size);
typedef struct
{
    uint32_t session, sequence, started, last_tx, timeout;
    PathReply reply, interrupted_reply;
    uint32_t interrupted_sequence;
    bool active, sent, overflow, locked, carriage;
    bool no_timeout; /* Scoped by STM32 warehouse task; faults still propagate. */
    unsigned stage, error;
    char request[80], line[80];
    size_t length;
    char aux_request[24];
    size_t aux_length;
    uint8_t disc_action_done_index, disc_action_event_index, disc_rfid_sent_index;
    bool aux_pending, cancel_after_aux, disc_action_event_pending;
    uint32_t group;
    uint32_t warehouse_token, warehouse_started, warehouse_timeout;
    uint8_t warehouse_digit;
    PathReply warehouse_reply;
    bool warehouse_active, warehouse_sent, warehouse_ready;
    char warehouse_request[64];
    bool pillar_ready, pillar_ending;
    bool disc_camera_ready;
    bool camera_wait_event; /* Startup heartbeat, consumed by the port adapter. */
    bool stair_scan;
    uint8_t ball_index, stopped_index, resume_index;
    RdkTransmit transmit;
    void *context;
} RdkLink;
void Rdk_Init(RdkLink *r, uint32_t session, RdkTransmit transmit, void *context);
bool Rdk_Begin(RdkLink *r, const char *verb, uint32_t argument, uint32_t now, uint32_t timeout);
bool Rdk_WarehouseBegin(RdkLink *r, uint8_t excluded, uint32_t now, uint32_t timeout);
void Rdk_Feed(RdkLink *r, uint8_t byte);
void Rdk_Tick(RdkLink *r, uint32_t now);
bool Rdk_TakeDiscActionDone(RdkLink *r, uint8_t *index);
bool Rdk_SendDiscRfidOk(RdkLink *r, uint8_t index);
bool Rdk_PillarStopped(RdkLink *r, uint8_t index);
bool Rdk_PillarEnd(RdkLink *r);
#endif
