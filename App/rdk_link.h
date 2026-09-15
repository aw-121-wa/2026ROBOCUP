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
    char request[80], line[80];
    size_t length;
    RdkTransmit transmit;
    void *context;
} RdkLink;
void Rdk_Init(RdkLink *r, uint32_t session, RdkTransmit transmit, void *context);
bool Rdk_Begin(RdkLink *r, const char *verb, uint32_t argument, uint32_t now, uint32_t timeout);
void Rdk_Feed(RdkLink *r, uint8_t byte);
void Rdk_Tick(RdkLink *r, uint32_t now);
#endif
