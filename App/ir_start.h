#ifndef IR_START_H
#define IR_START_H
#include <stdbool.h>
#include <stdint.h>

/* One start per MCU reset. Running/fault/not-ready inputs discard partial gestures. */
typedef struct {
    uint32_t since;
    bool sampled, blocked, covered, fired;
} IrStart;
static inline bool IrStart_Update(IrStart *s, uint32_t now, bool blocked, bool ready)
{
    if (s->fired) return false;
    if (!ready) { s->sampled=s->covered=false; return false; }
    if (!s->sampled || s->blocked!=blocked) {
        s->sampled=true; s->blocked=blocked; s->since=now;
        return false;
    }
    if ((uint32_t)(now-s->since)<100U) return false;
    if (blocked) s->covered=true;
    else if (s->covered) { s->fired=true; return true; }
    return false;
}
#endif
