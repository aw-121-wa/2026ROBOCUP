#include "turntable_link.h"
void Turn_Init(TurntableLink *t, TurnTransmit tx, void *ctx)
{
    *t = (TurntableLink){.transmit = tx, .context = ctx};
}
bool Turn_Start(TurntableLink *t, bool r, uint32_t n)
{
    if (!Turn_StartSteps(t, r, 1, n)) return false;
    t->settle_ms = 600U; /* Collection: retain the existing insertion margin. */
    return true;
}
bool Turn_StartSteps(TurntableLink *t, bool r, uint8_t steps, uint32_t n)
{
    if (!steps || steps > BALL_SLOT_COUNT/2 || t->pending || t->reply == PATH_FAILED)
        return false;
    t->steps = steps;
    t->settle_ms = 0U; /* Warehouse positioning. */
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
    if (!t->no_timeout && (uint32_t)(n - t->started) >= (t->stopping || t->steps == 1 ? 2000U : 3000U))
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
        /* Completion is estimated travel (240 ms per slot) + the configured settle margin.
         * UART transfer success does not constitute position feedback. */
        if ((uint32_t)(n - t->at) >= (t->stopping ? 2U : 240U*t->steps+t->settle_ms))
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
    const uint8_t position[] = {5, 0xfd, t->direction, 3, 0xe8, 0, 0, 0, (uint8_t)(5U*t->steps), 0, 0, 1, 0x6b};
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
