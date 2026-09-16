#ifndef PATH_YAW_H
#define PATH_YAW_H
#include <stdbool.h>
#include <math.h>
typedef struct
{
    float continuous, previous;
    bool ready;
} PathYaw;
static inline void PathYaw_Update(PathYaw *p, float wrapped, bool ready)
{
    if (!ready || !isfinite(wrapped))
    {
        *p = (PathYaw){0};
        return;
    }
    if (!p->ready)
    {
        *p = (PathYaw){.continuous = wrapped, .previous = wrapped, .ready = true};
        return;
    }
    p->continuous += remainderf(wrapped - p->previous, 6.283185307179586f);
    p->previous = wrapped;
}
#endif
