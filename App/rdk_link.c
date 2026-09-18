#include "rdk_link.h"
#include <string.h>
static void fail(RdkLink *r, unsigned e)
{
    r->active = false;
    r->locked = true;
    r->stage = 6;
    r->error = e;
    r->reply = PATH_FAILED;
    r->aux_pending = false;
    r->cancel_after_aux = false;
    r->disc_action_event_pending = false;
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
        {
            if (r->active && r->stage == 4)
            {
                static const char cancel[] = "DISC_CANCEL\r\n";
                memcpy(r->aux_request, cancel, sizeof(cancel) - 1U);
                r->aux_length = sizeof(cancel) - 1U;
                r->aux_pending = true;
                r->cancel_after_aux = true;
            }
            else
                fail(r, 5);
        }
        return true;
    }
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
        r->disc_action_done_index = 0;
        r->disc_action_event_index = 0;
        r->disc_rfid_sent_index = 0;
        r->disc_action_event_pending = false;
        r->aux_pending = false;
        r->cancel_after_aux = false;
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
    else if (!strncmp(r->line, "DISC_ACTION_DONE ", 17) &&
             strlen(r->line) == 18 && r->stage == 4)
    {
        uint8_t index = (uint8_t)(r->line[17] - '0');
        if (index < 1 || index > 5 ||
            index != (uint8_t)(r->disc_action_done_index + 1U) ||
            r->disc_action_event_pending ||
            r->disc_rfid_sent_index != r->disc_action_done_index)
        {
            fail(r, 3);
            return;
        }
        r->disc_action_done_index = index;
        r->disc_action_event_index = index;
        r->disc_action_event_pending = true;
    }
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
    if (r->aux_pending)
    {
        if (r->transmit(r->context, r->aux_request, r->aux_length))
        {
            r->aux_pending = false;
            if (r->cancel_after_aux)
            {
                r->cancel_after_aux = false;
                fail(r, 5);
            }
        }
        return;
    }
    if (!r->sent && r->transmit(r->context, r->request, strlen(r->request)))
    {
        r->sent = true;
        r->last_tx = n;
    }
}

bool Rdk_TakeDiscActionDone(RdkLink *r, uint8_t *index)
{
    if (!index || !r->disc_action_event_pending)
        return false;
    *index = r->disc_action_event_index;
    r->disc_action_event_pending = false;
    return true;
}

bool Rdk_SendDiscRfidOk(RdkLink *r, uint8_t index)
{
    static const char prefix[] = "DISC_RFID_OK ";
    if (!r->active || r->stage != 4 || r->locked || r->aux_pending ||
        index < 1 || index > 5 || index != r->disc_action_done_index ||
        index != (uint8_t)(r->disc_rfid_sent_index + 1U))
        return false;
    memcpy(r->aux_request, prefix, sizeof(prefix) - 1U);
    r->aux_request[sizeof(prefix) - 1U] = (char)('0' + index);
    r->aux_request[sizeof(prefix)] = '\r';
    r->aux_request[sizeof(prefix) + 1U] = '\n';
    r->aux_length = sizeof(prefix) + 2U;
    r->aux_pending = true;
    r->cancel_after_aux = false;
    r->disc_rfid_sent_index = index;
    return true;
}
