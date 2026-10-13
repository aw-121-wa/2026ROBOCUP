#include "rdk_link.h"
#include "path_config.h"
#include <string.h>
#include <stdlib.h>
/* Protocol fields are unsigned decimal, never general printf formats.
 * Largest line: WAREHOUSE_CHECK + two uint32 fields + CRLF + NUL = 40 bytes.
 * Generic/scan/warehouse buffers fit that bound; aux verbs take only uint8
 * values (maximum 20 bytes including NUL, within aux_request[24]). */
static char *decimal(char *out,uint32_t value)
{
    char digits[10];unsigned n=0;
    do { digits[n++]=(char)('0'+value%10);value/=10; } while(value);
    while(n) *out++=digits[--n];
    return out;
}
static size_t request(char *out,const char *verb,uint32_t a,bool second,uint32_t b)
{
    char *p=out;
    while(*verb) *p++=*verb++;
    *p++=' ';p=decimal(p,a);
    if(second) { *p++=' ';p=decimal(p,b); }
    *p++='\r';*p++='\n';*p=0;
    return (size_t)(p-out);
}
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
        if (r->scan_active) (void)Rdk_Scan(r,4);
        bool number_active=r->warehouse_active;
        r->warehouse_active=false; r->warehouse_reply=PATH_NONE;
        if (number_active && !r->locked) {
            strcpy(r->aux_request,"DISC_CANCEL\r\n");
            r->aux_length=strlen(r->aux_request);
            r->aux_pending=r->cancel_after_aux=true;
            return true;
        }
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
    if (r->active || r->locked || r->warehouse_active || r->scan_active || !t)
        return false;
    if (!strcmp(v, "HELLO"))
    {
        strcpy(r->request, "PING\r\n");
        r->stage = 1;
    }
    else if (!strcmp(v, "COLOR") && a <= 1)
    {
        strcpy(r->request, a ? "COLOR BLUE\r\n" : "COLOR RED\r\n");
        r->group = a;
        r->stage = 13;
    }
    else if (!strcmp(v, "GROUP") && (r->stage == 2 || r->stage == 5) &&
             (a == 0 || a == 1 || a == 2 || a == 3 || a == 4 || a == 100 || a == 105 ||
              (a >= 109 && a <= 120)))
    {
        request(r->request,"GROUP",a,false,0);
        r->group = a;
        r->stage = 7;
    }
    else if (!strcmp(v, "STAIR") && (r->stage == 2 || r->stage == 5) && a >= 1 && a <= 8)
    {
        request(r->request,"STAIR_CHECK",a,false,0);
        r->group = a; /* Point token: stale replies cannot complete another point. */
        r->stage = 11;
        r->disc_action_done_index = r->disc_rfid_sent_index = 0;
        r->disc_action_event_pending = r->aux_pending = false;
        r->cancel_after_aux = false;
    }
    else if ((!strcmp(v, "PILLAR") || (!strcmp(v, "STAIR_SCAN") && a>=1 && a<=3)) &&
             (r->stage == 2 || r->stage == 5))
    {
        r->stair_scan = !strcmp(v, "STAIR_SCAN");
        if (r->stair_scan)
            request(r->request,"STAIR_SCAN",a,false,0);
        else strcpy(r->request, "PILLAR_START\r\n");
        r->stage = 9;
        r->pillar_ready = r->pillar_ending = r->camera_wait_event = false;
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
    r->disc_camera_ready=false;
    r->started = n;
    r->timeout = t;
    r->error = 0;
    return true;
}
bool Rdk_WarehouseBegin(RdkLink *r, uint8_t excluded, uint32_t now, uint32_t timeout)
{
    if (r->active || r->locked || r->warehouse_active || r->scan_active || !timeout || (excluded & ~14U)) return false;
    if (++r->sequence==0) ++r->sequence;
    r->warehouse_token = r->sequence;
    request(r->warehouse_request,"WAREHOUSE_CHECK",r->warehouse_token,true,excluded);
    r->warehouse_started=now; r->warehouse_timeout=timeout;
    r->warehouse_active=true; r->warehouse_sent=false; r->warehouse_ready=false;
    r->warehouse_digit=0; r->warehouse_reply=PATH_WAIT;
    return true;
}
bool Rdk_Scan(RdkLink *r, unsigned row)
{
    if (row==4) {
        if (!r->scan_active || r->scan_stopping) return true;
        request(r->scan_request,"WH_STOP",r->scan_token,false,0);
        r->scan_stopping=true;r->scan_pending=true;return true;
    }
    if (row>3 || r->locked || r->active || r->warehouse_active || r->scan_active) return false;
    if (++r->sequence==0) ++r->sequence;
    r->scan_token=r->sequence;r->scan_active=r->scan_pending=true;
    r->scan_stopping=r->scan_seen=false;r->scan_digit=0;r->scan_frame=0;
    request(r->scan_request,PATH_WAREHOUSE_CAPTURE_START ? "WH_CAPTURE" : "WH_SCAN",r->scan_token,true,row);
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
    if (!strncmp(r->line,"WH_",3)) {
        char *end;
        const char *start=r->line+(!strncmp(r->line,"WH_FRAME ",9) ? 9 : 8);
        if (strncmp(r->line,"WH_FRAME ",9) && strncmp(r->line,"WH_DONE ",8)) return;
        unsigned long token=strtoul(start,&end,10);
        if(end==start || token!=r->scan_token || !r->scan_active) return;
        if(!strncmp(r->line,"WH_DONE ",8)) {
            if(!*end && r->scan_stopping) r->scan_active=r->scan_pending=r->scan_seen=false;
        } else if(*end==' ' && !r->scan_stopping && !r->scan_pending) {
            start=end+1;unsigned long frame=strtoul(start,&end,10);
            if(end==start || *end!=' ' || frame<=r->scan_frame) return;
            start=end+1;unsigned long digit=strtoul(start,&end,10);
            if(end==start || *end!=' ' || digit>3) return;
            start=end+1;long offset=strtol(start,&end,10);
            if(end==start || *end || offset < -640 || offset>640) return;
            r->scan_frame=(uint32_t)frame;r->scan_digit=(uint8_t)digit;r->scan_offset=(int16_t)offset;
            r->scan_received=r->now;r->scan_seen=true;
        }
        return; /* Never let stale scan packets complete an arm command. */
    }
#if !PATH_CONTINUOUS_WAREHOUSE
    if (!strncmp(r->line,"WAREHOUSE_READY ",16)) {
        char *end;
        unsigned long token=strtoul(r->line+16,&end,10);
        if (*end=='\0' && end!=r->line+16 &&
            r->warehouse_active && r->warehouse_sent && !r->locked && token==r->warehouse_token)
            r->warehouse_ready=true;
        return;
    }
    if (!strncmp(r->line,"WAREHOUSE_DIGIT ",16)) {
        if (r->line[16]>='0' && r->line[16]<='9') {
            char *end;
            unsigned long token=strtoul(r->line+16,&end,10);
            if (*end==' ' && end[1]>='0' && end[1]<='3' && !end[2] &&
                r->warehouse_active && r->warehouse_sent && !r->locked && token==r->warehouse_token) {
                r->warehouse_digit=(uint8_t)(end[1]-'0');
                r->warehouse_reply=r->warehouse_digit ? PATH_OK : PATH_NONE;
                r->warehouse_active=false;
            }
        }
        return; /* Optional/stale digit replies never enter the ball/action protocol. */
    }
#endif
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
    else if (r->stage == 13 && !strcmp(r->line, r->group ? "COLOR_OK BLUE" : "COLOR_OK RED"))
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
    else if (r->stage == 10 && !r->pillar_ready && !r->pillar_ending &&
             !strcmp(r->line, "PILLAR_CAMERA_WAIT"))
        r->camera_wait_event = true;
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
    else if (r->stage==4 && !r->disc_camera_ready && !strcmp(r->line,"DISC_CAMERA_WAIT"))
        r->camera_wait_event=true;
    else if (r->stage==4 && !r->disc_camera_ready && !strcmp(r->line,"DISC_CAMERA_READY")) {
        r->disc_camera_ready=true; r->camera_wait_event=true;
    }
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
    r->now=n;
    if (r->scan_pending && r->transmit(r->context,r->scan_request,strlen(r->scan_request)))
        r->scan_pending=false;
#if !PATH_CONTINUOUS_WAREHOUSE
    if (r->warehouse_active) {
        if (r->locked || (!r->no_timeout && (uint32_t)(n-r->warehouse_started)>=r->warehouse_timeout)) {
            r->warehouse_active=false; r->warehouse_reply=PATH_NONE; r->warehouse_digit=0;
        } else if (!r->warehouse_sent && r->transmit(r->context,r->warehouse_request,strlen(r->warehouse_request)))
            r->warehouse_sent=true;
    }
#endif
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
    if (!r->no_timeout && (elapsed >= r->timeout || ((r->stage == 1 || r->stage == 3 || r->stage == 7 || r->stage == 9 || r->stage == 11) && elapsed >= 2000)))
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
    r->aux_length = request(r->aux_request, r->stage == 4 ? "DISC_RFID_OK" : r->stage == 12 ? "STAIR_RFID_OK" : "PILLAR_RFID_OK",
                                   r->stage == 12 ? (unsigned)r->group : index,false,0);
    r->aux_pending = true;
    r->cancel_after_aux = false;
    r->disc_rfid_sent_index = index;
    return true;
}

bool Rdk_PillarStopped(RdkLink *r, uint8_t index)
{
    if (!r->active || r->locked || r->stage != 10 || r->aux_pending ||
        !index || index != r->ball_index || index != r->stopped_index + 1U) return false;
    r->aux_length = request(r->aux_request,"PILLAR_STOPPED",index,false,0);
    r->aux_pending = true;
    r->stopped_index = index;
    return true;
}
bool Rdk_PillarEnd(RdkLink *r)
{
    if (!r->active || r->locked || r->stage != 10 || r->aux_pending ||
        (r->ball_index != r->resume_index &&
         !(r->stair_scan && r->stopped_index == r->resume_index))) return false;
    strcpy(r->aux_request, "PILLAR_END\r\n");
    r->aux_length = strlen(r->aux_request);
    r->aux_pending = r->pillar_ending = true;
    return true;
}
