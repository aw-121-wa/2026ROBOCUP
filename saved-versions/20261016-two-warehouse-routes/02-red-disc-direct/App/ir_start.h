#ifndef IR_START_H
#define IR_START_H
#include <stdbool.h>
#include <stdint.h>
#define IR_START_DEBOUNCE_MS 100U
#define IR_START_PENDING_MS 10000U
#define IR_START_BLUE_HOLD_MS 5000U
/* Observe during startup; consume a short-lived request only when ready.
 * The caller commits fired only after both ARM and PATH have succeeded. */
typedef struct {
    uint32_t since, requested_at;
    bool sampled, blocked, covered, pending, fired, blue, long_seen;
} IrStart;
static inline void IrStart_Clear(IrStart *s)
{
    bool fired=s->fired;
    *s=(IrStart){.fired=fired};
}
static inline bool IrStart_Update(IrStart *s, uint32_t now, bool blocked, bool ready)
{
    if (s->fired) return false;
    if (s->pending && (uint32_t)(now-s->requested_at)>=IR_START_PENDING_MS)
        s->pending=false;
    /* Latch the long gesture before handling release, including sparse sampling. */
    if (s->sampled && s->blocked && !s->long_seen &&
        (uint32_t)(now-s->since)>IR_START_BLUE_HOLD_MS) {
        s->long_seen=true; s->covered=false; s->blue=true;
        s->pending=true; s->requested_at=now;
    }
    if (!s->sampled || s->blocked!=blocked) {
        s->sampled=true; s->blocked=blocked; s->since=now;
        if (blocked) { s->pending=false; s->long_seen=false; s->covered=false; }
        return false;
    }
    if ((uint32_t)(now-s->since)<IR_START_DEBOUNCE_MS) return false;
    if (blocked && !s->long_seen) s->covered=true;
    else if (!blocked) {
        if (s->covered && !s->long_seen) {
            s->covered=false; s->pending=true; s->blue=false; s->requested_at=now;
        }
    }
    if (s->pending && ready) { s->pending=false; return true; }
    return false;
}
#endif
