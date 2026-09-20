#include "rdk_link.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
static unsigned indexed(const char *line, const char *prefix)
{
    size_t n = strlen(prefix);
    if (strncmp(line, prefix, n) || line[n] < '0' || line[n] > '9') return 256;
    char *end;
    unsigned long value = strtoul(line + n, &end, 10);
    return *end || value > 255 ? 256 : (unsigned)value;
}
static void fail(RdkLink *r, unsigned e)
{
    bool cancel = r->active && r->stage >= 3 && r->stage != 6;
    r->active = false;
    r->locked = true;
    r->stage = 6;
    r->error = e;
    r->reply = PATH_FAILED;
    r->aux_pending = false;
    r->cancel_after_aux = false;
    r->disc_action_event_pending = false;
    if (cancel)
    {
        strcpy(r->aux_request, "DISC_CANCEL\r\n"); /* Cancels the single RDK worker of any type. */
        r->aux_length = strlen(r->aux_request);
        r->aux_pending = r->cancel_after_aux = true;
    }
}
void Rdk_Init(RdkLink *r, uint32_t s, RdkTransmit tx, void *ctx)
{
    *r = (RdkLink){.session = s, .transmit = tx, .context = ctx};
}
bool Rdk_Begin(RdkLink *r, const char *v, uint32_t a, uint32_t n, uint32_t t)
{
    if (!strcmp(v, "STOP"))
    {
        if (!r->locked)
        {
            if (r->active && r->stage != 1)
            {
                const char *cancel = r->stage >= 9 ? "PILLAR_CANCEL\r\n" : "DISC_CANCEL\r\n";
                strcpy(r->aux_request, cancel);
                r->aux_length = strlen(cancel);
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
    else if (!strcmp(v, "GROUP") && (r->stage == 2 || r->stage == 5) &&
             (a == 0 || a == 1 || a == 2 || a == 100 || a == 105))
    {
        snprintf(r->request, sizeof(r->request), "GROUP %lu\r\n", (unsigned long)a);
        r->group = a;
        r->stage = 7;
    }
    else if (!strcmp(v, "STAIR") && (r->stage == 2 || r->stage == 5) && a >= 1 && a <= 8)
    {
        snprintf(r->request, sizeof(r->request), "STAIR_CHECK %lu\r\n", (unsigned long)a);
        r->group = a; /* Point token: stale replies cannot complete another point. */
        r->stage = 11;
        r->disc_action_done_index = r->disc_rfid_sent_index = 0;
        r->disc_action_event_pending = r->aux_pending = false;
        r->cancel_after_aux = false;
    }
    else if (!strcmp(v, "PILLAR") && (r->stage == 2 || r->stage == 5))
    {
        strcpy(r->request, "PILLAR_START\r\n");
        r->stage = 9;
        r->pillar_ready = r->pillar_ending = false;
        r->ball_index = r->stopped_index = r->resume_index = 0;
        r->disc_action_done_index = r->disc_rfid_sent_index = 0;
        r->disc_action_event_pending = r->aux_pending = false;
    }
    else if (!strcmp(v, "DISC") && (r->stage == 2 || r->stage == 5))
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
    if (!r->active && r->stage == 0 && !strcmp(r->line, "PONG")) return;
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
    else if (r->stage == 7 && indexed(r->line, "GROUP_ACK ") == r->group)
        r->stage = 8;
    else if (r->stage == 8 && indexed(r->line, "GROUP_DONE ") == r->group)
    {
        r->stage = 2;
        r->active = false;
        r->reply = PATH_OK;
    }
    else if (r->stage == 11 && indexed(r->line, "STAIR_ACK ") == r->group)
        r->stage = 12;
    else if (r->stage == 12 && !r->disc_action_done_index &&
             indexed(r->line, "STAIR_ACTION_DONE ") == r->group)
    {
        r->disc_action_done_index = r->disc_action_event_index = 1;
        r->disc_action_event_pending = true;
    }
    else if (r->stage == 12 && !r->disc_action_done_index &&
             indexed(r->line, "STAIR_NONE ") == r->group)
    {
        r->stage = 2;
        r->active = false;
        r->reply = PATH_NONE;
    }
    else if (r->stage == 12 && r->disc_rfid_sent_index == 1 && !r->aux_pending &&
             indexed(r->line, "STAIR_DONE ") == r->group)
    {
        r->stage = 2;
        r->active = false;
        r->reply = PATH_OK;
    }
    else if (r->stage == 9 && !strcmp(r->line, "PILLAR_ACK"))
        r->stage = 10;
    else if (r->stage == 10 && !r->pillar_ready && !strcmp(r->line, "PILLAR_READY"))
        r->pillar_ready = true;
    else if (r->stage == 10 && r->pillar_ready && !r->pillar_ending &&
             indexed(r->line, "PILLAR_BALL ") == (unsigned)r->ball_index + 1U &&
             r->ball_index == r->resume_index && r->ball_index < 59)
        ++r->ball_index;
    else if (r->stage == 10 && !r->pillar_ending &&
             indexed(r->line, "PILLAR_ACTION_DONE ") == (unsigned)r->disc_action_done_index + 1U &&
             r->stopped_index == r->ball_index && r->ball_index == r->disc_action_done_index + 1U)
    {
        r->disc_action_done_index = r->ball_index;
        r->disc_action_event_index = r->ball_index;
        r->disc_action_event_pending = true;
    }
    else if (r->stage == 10 && !r->pillar_ending &&
             indexed(r->line, "PILLAR_RESUME ") == (unsigned)r->resume_index + 1U &&
             r->disc_rfid_sent_index == r->resume_index + 1U && !r->aux_pending)
        ++r->resume_index;
    else if (r->stage == 10 && r->pillar_ending && !strcmp(r->line, "PILLAR_DONE"))
    {
        r->stage = 2;
        r->active = false;
        r->reply = PATH_OK;
    }
    else if (r->stage == 10 && r->pillar_ending && indexed(r->line, "PILLAR_BALL ") == r->ball_index + 1U)
        return; /* END and an in-flight detection crossed; never grant STOPPED. */
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
    if (r->aux_pending && r->cancel_after_aux)
    {
        if (r->transmit(r->context, r->aux_request, r->aux_length))
        {
            r->active = false;
            unsigned error = r->error ? r->error : 5;
            fail(r, error);
        }
        return;
    }
    if (!r->active)
        return;
    uint32_t elapsed = n - r->started;
    if (elapsed >= r->timeout || ((r->stage == 1 || r->stage == 3 || r->stage == 7 || r->stage == 9 || r->stage == 11) && elapsed >= 2000))
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
    if (!r->active || (r->stage != 4 && r->stage != 10 && r->stage != 12) || r->locked || r->aux_pending ||
        index < 1 || index > (r->stage == 4 ? 5 : r->stage == 12 ? 1 : 59) || index != r->disc_action_done_index ||
        index != (uint8_t)(r->disc_rfid_sent_index + 1U))
        return false;
    r->aux_length = (size_t)snprintf(r->aux_request, sizeof(r->aux_request), "%s_RFID_OK %u\r\n",
                                   r->stage == 4 ? "DISC" : r->stage == 12 ? "STAIR" : "PILLAR",
                                   r->stage == 12 ? (unsigned)r->group : index);
    r->aux_pending = true;
    r->cancel_after_aux = false;
    r->disc_rfid_sent_index = index;
    return true;
}

bool Rdk_PillarStopped(RdkLink *r, uint8_t index)
{
    if (!r->active || r->locked || r->stage != 10 || r->aux_pending ||
        !index || index != r->ball_index || index != r->stopped_index + 1U) return false;
    r->aux_length = (size_t)snprintf(r->aux_request, sizeof(r->aux_request), "PILLAR_STOPPED %u\r\n", index);
    r->aux_pending = true;
    r->stopped_index = index;
    return true;
}
bool Rdk_PillarEnd(RdkLink *r)
{
    if (!r->active || r->locked || r->stage != 10 || r->aux_pending ||
        r->ball_index != r->resume_index) return false;
    strcpy(r->aux_request, "PILLAR_END\r\n");
    r->aux_length = strlen(r->aux_request);
    r->aux_pending = r->pillar_ending = true;
    return true;
}
