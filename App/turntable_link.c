#include "turntable_link.h"
void Turn_Init(TurntableLink *t, TurnTransmit tx, void *ctx)
{
    *t = (TurntableLink){.transmit = tx, .context = ctx};
}
bool Turn_Start(TurntableLink *t, bool r, uint32_t n)
{
    if (t->pending || t->reply == PATH_FAILED)
        return false;
    t->direction = r ? 1 : 0;
    t->started = n;
    t->frame = 0;
    t->pending = true;
    t->stopping = false;
    t->awaiting = false;
    t->settling = false;
    t->reply = PATH_WAIT;
    return true;
}
void Turn_Stop(TurntableLink *t, uint32_t n)
{
    t->started = n;
    t->frame = 0;
    t->pending = true;
    t->stopping = true;
    t->awaiting = false;
    t->settling = false;
    t->reply = PATH_WAIT;
}
void Turn_Tick(TurntableLink *t, uint32_t n, bool idle)
{
    if (!t->pending)
        return;
    if ((uint32_t)(n - t->started) >= 2000)
    {
        t->reply = PATH_FAILED;
        t->pending = false;
        return;
    }
    if (!idle)
        return;
    if (t->awaiting)
    {
        t->awaiting = false;
        t->at = n;
        t->frame++;
        if (t->frame == (t->stopping ? 2 : 4))
            t->settling = true;
        return;
    }
    if (t->settling)
    {
        /* Legacy completion is estimated travel (240 ms) + 600 ms settle.
         * UART transfer success does not constitute position feedback. */
        if ((uint32_t)(n - t->at) >= (t->stopping ? 2U : 840U))
        {
            t->pending = false;
            t->reply = PATH_OK;
        }
        return;
    }
    if (t->frame && (uint32_t)(n - t->at) < 2)
        return;
    const uint8_t enable[] = {5, 0xf3, 0xab, 1, 1, 0x6b};
    const uint8_t stop[] = {5, 0xfe, 0x98, 1, 0x6b};
    const uint8_t sync[] = {0, 0xff, 0x66, 0x6b};
    const uint8_t position[] = {5, 0xfd, t->direction, 3, 0xe8, 0, 0, 0, 5, 0, 0, 1, 0x6b};
    const uint8_t *data;
    size_t size;
    if (t->frame & 1)
    {
        data = sync;
        size = sizeof(sync);
    }
    else if (t->stopping)
    {
        data = stop;
        size = sizeof(stop);
    }
    else if (t->frame == 0)
    {
        data = enable;
        size = sizeof(enable);
    }
    else
    {
        data = position;
        size = sizeof(position);
    }
    if (t->transmit(t->context, data, size))
        t->awaiting = true;
}
