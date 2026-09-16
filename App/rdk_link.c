#include "rdk_link.h"
#include <string.h>
static void fail(RdkLink *r, unsigned e)
{
    r->active = false;
    r->locked = true;
    r->stage = 6;
    r->error = e;
    r->reply = PATH_FAILED;
}
void Rdk_Init(RdkLink *r, uint32_t s, RdkTransmit tx, void *ctx)
{
    *r = (RdkLink){.session = s, .transmit = tx, .context = ctx};
}
bool Rdk_Begin(RdkLink *r, const char *v, uint32_t a, uint32_t n, uint32_t t)
{
    (void)a;
    if (!strcmp(v, "STOP"))
    {
        if (!r->locked)
            fail(r, 5);
        return true;
    } /* Local lock only: RDK cannot abort. */
    if (r->active || r->locked || !t)
        return false;
    if (!strcmp(v, "HELLO"))
    {
        strcpy(r->request, "PING\r\n");
        r->stage = 1;
    }
    else if (!strcmp(v, "DISC") && r->stage == 2)
    {
        strcpy(r->request, "DISC_START\r\n");
        r->stage = 3;
    }
    else
        return false;
    r->sequence++;
    r->active = true;
    r->sent = false;
    r->reply = PATH_WAIT;
    r->started = n;
    r->timeout = t;
    r->error = 0;
    return true;
}
void Rdk_Feed(RdkLink *r, uint8_t b)
{
    if (b == '\r')
    {
        r->carriage = true;
        return;
    }
    if (b != '\n')
    {
        if (r->carriage || b < 32 || b > 126 || r->length >= sizeof(r->line) - 1)
            r->overflow = true;
        if (!r->overflow)
            r->line[r->length++] = (char)b;
        return;
    }
    r->line[r->length] = 0;
    r->length = 0;
    r->carriage = false;
    if (r->overflow)
    {
        r->overflow = false;
        fail(r, 3);
        return;
    }
    if (!r->line[0] || r->locked)
        return;
    if (!r->active || !r->sent)
    {
        fail(r, 3);
        return;
    }
    if (!strcmp(r->line, "PONG") && r->stage == 1)
    {
        r->stage = 2;
        r->active = false;
        r->reply = PATH_OK;
    }
    else if (!strcmp(r->line, "DISC_ACK") && r->stage == 3)
        r->stage = 4;
    else if (!strcmp(r->line, "DISC_DONE") && r->stage == 4)
    {
        r->stage = 5;
        r->active = false;
        r->reply = PATH_OK;
    }
    else if (!strcmp(r->line, "DISC_ERROR") && (r->stage == 3 || r->stage == 4))
        fail(r, 2);
    else
        fail(r, 3);
}
void Rdk_Tick(RdkLink *r, uint32_t n)
{
    if (!r->active)
        return;
    uint32_t elapsed = n - r->started;
    if (elapsed >= r->timeout || ((r->stage == 1 || r->stage == 3) && elapsed >= 2000))
    {
        fail(r, 1);
        return;
    }
    if (!r->sent && r->transmit(r->context, r->request, strlen(r->request)))
    {
        r->sent = true;
        r->last_tx = n;
    }
}
