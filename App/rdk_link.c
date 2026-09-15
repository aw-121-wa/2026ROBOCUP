#include "rdk_link.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <limits.h>
void Rdk_Init(RdkLink *r, uint32_t s, RdkTransmit tx, void *ctx)
{
    *r = (RdkLink){.session = s, .transmit = tx, .context = ctx};
}
bool Rdk_Begin(RdkLink *r, const char *v, uint32_t a, uint32_t n, uint32_t t)
{
    bool stop = !strcmp(v, "STOP");
    if ((r->active && !stop) || (r->locked && !stop) || r->sequence == UINT32_MAX || !t)
        return false;
    if (strcmp(v, "GROUP") && strcmp(v, "VISION") && strcmp(v, "DISC") && strcmp(v, "HELLO") &&
        !stop)
        return false;
    r->interrupted_sequence = stop ? r->sequence : 0;
    r->interrupted_reply = stop ? r->reply : PATH_WAIT;
    r->sequence++;
    int count =
        snprintf(r->request, sizeof(r->request), "Q %lu %lu %s %lu\n", (unsigned long)r->session,
                 (unsigned long)r->sequence, v, (unsigned long)a);
    if (count < 0 || (size_t)count >= sizeof(r->request))
        return false;
    r->active = true;
    r->sent = false;
    r->reply = PATH_WAIT;
    r->started = n;
    r->timeout = t;
    return true;
}
static bool number(char **s, uint32_t *n)
{
    if (**s < '0' || **s > '9')
        return false;
    uint32_t value = 0;
    do
    {
        unsigned d = (unsigned)(**s - '0');
        if (value > (UINT32_MAX - d) / 10U)
            return false;
        value = value * 10U + d;
        (*s)++;
    } while (**s >= '0' && **s <= '9');
    if (**s != ' ')
        return false;
    (*s)++;
    *n = value;
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
        if (r->carriage)
            r->overflow = true;
        if (b < 32 || b > 126 || r->length >= sizeof(r->line) - 1)
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
        return;
    }
    if (!r->active || strncmp(r->line, "R ", 2))
        return;
    char *s = r->line + 2;
    uint32_t session, sequence;
    if (!number(&s, &session) || !number(&s, &sequence) || session != r->session ||
        (sequence != r->sequence && sequence != r->interrupted_sequence))
        return;
    if (!strcmp(s, "ACK"))
        return;
    PathReply result;
    if (!strcmp(s, "DONE"))
        result = PATH_OK;
    else if (!strcmp(s, "NONE"))
        result = PATH_NONE;
    else if (!strcmp(s, "ERROR"))
    {
        result = PATH_FAILED;
        r->locked = true;
    }
    else
        return;
    if (sequence == r->interrupted_sequence)
    {
        r->interrupted_reply = result;
        return;
    }
    r->reply = result;
    r->active = false;
}
void Rdk_Tick(RdkLink *r, uint32_t n)
{
    if (!r->active)
        return;
    if ((uint32_t)(n - r->started) >= r->timeout)
    {
        r->reply = PATH_FAILED;
        r->active = false;
        r->locked = true;
        return;
    }
    /* Identical retransmissions also renew the RDK's two-second detector lease. */
    if (!r->sent || (uint32_t)(n - r->last_tx) >= 500)
    {
        if (r->transmit(r->context, r->request, strlen(r->request)))
        {
            r->last_tx = n;
            r->sent = true;
        }
    }
}
