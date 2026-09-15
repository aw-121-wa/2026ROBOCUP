#ifndef TURNTABLE_LINK_H
#define TURNTABLE_LINK_H
#include "path_mission.h"
#include <stddef.h>
typedef bool (*TurnTransmit)(void *context, const uint8_t *data, size_t size);
typedef struct
{
    PathReply reply;
    uint8_t frame, direction;
    bool pending, stopping, awaiting, settling;
    uint32_t started, at;
    TurnTransmit transmit;
    void *context;
} TurntableLink;
void Turn_Init(TurntableLink *t, TurnTransmit transmit, void *context);
bool Turn_Start(TurntableLink *t, bool reverse, uint32_t now);
void Turn_Stop(TurntableLink *t, uint32_t now);
void Turn_Tick(TurntableLink *t, uint32_t now, bool tx_idle);
#endif
