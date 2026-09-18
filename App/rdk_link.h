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
    unsigned stage, error;
    char request[80], line[80];
    size_t length;
    char aux_request[24];
    size_t aux_length;
    uint8_t disc_action_done_index, disc_action_event_index, disc_rfid_sent_index;
    bool aux_pending, cancel_after_aux, disc_action_event_pending;
    RdkTransmit transmit;
    void *context;
} RdkLink;
void Rdk_Init(RdkLink *r, uint32_t session, RdkTransmit transmit, void *context);
bool Rdk_Begin(RdkLink *r, const char *verb, uint32_t argument, uint32_t now, uint32_t timeout);
void Rdk_Feed(RdkLink *r, uint8_t byte);
void Rdk_Tick(RdkLink *r, uint32_t now);
bool Rdk_TakeDiscActionDone(RdkLink *r, uint8_t *index);
bool Rdk_SendDiscRfidOk(RdkLink *r, uint8_t index);
#endif
