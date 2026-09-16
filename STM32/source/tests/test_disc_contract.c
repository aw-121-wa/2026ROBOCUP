#include "rdk_link.h"
#include <stdio.h>
#include <string.h>
#define CHECK(x)                                                                                   \
    do                                                                                             \
    {                                                                                              \
        if (!(x))                                                                                  \
        {                                                                                          \
            printf("FAIL %d: %s\n", __LINE__, #x);                                                 \
            return 1;                                                                              \
        }                                                                                          \
    } while (0)
static char wire[80];
static unsigned sends;
static bool tx(void *ctx, const char *s, size_t n)
{
    (void)ctx;
    memcpy(wire, s, n);
    wire[n] = 0;
    sends++;
    return true;
}
static void feed(RdkLink *r, const char *s)
{
    while (*s)
        Rdk_Feed(r, (uint8_t)*s++);
}
int main(void)
{
    RdkLink r;
    Rdk_Init(&r, 1, tx, 0);
    CHECK(Rdk_Begin(&r, "HELLO", 0, 0, 2000));
    Rdk_Tick(&r, 0);
    CHECK(!strcmp(wire, "PING\r\n"));
    feed(&r, "PONG\r\n");
    CHECK(!r.active && r.reply == PATH_OK);
    CHECK(Rdk_Begin(&r, "DISC", 0, 10, 180000));
    Rdk_Tick(&r, 10);
    CHECK(!strcmp(wire, "DISC_START\r\n"));
    Rdk_Tick(&r, 600);
    CHECK(sends == 2); /* Never retransmit a physical task. */
    feed(&r, "DISC_ACK\r\n");
    CHECK(r.active && r.reply == PATH_WAIT);
    feed(&r, "DISC_DONE\r\n");
    CHECK(!r.active && r.reply == PATH_OK);
    Rdk_Init(&r, 1, tx, 0);
    CHECK(Rdk_Begin(&r, "HELLO", 0, 0, 2000));
    Rdk_Tick(&r, 0);
    Rdk_Tick(&r, 2000);
    CHECK(r.locked && r.reply == PATH_FAILED);
    CHECK(!Rdk_Begin(&r, "DISC", 0, 2001, 180000));
    Rdk_Init(&r, 1, tx, 0);
    CHECK(Rdk_Begin(&r, "HELLO", 0, 0, 2000));
    Rdk_Tick(&r, 0);
    feed(&r, "PONG\r\n");
    CHECK(Rdk_Begin(&r, "DISC", 0, 10, 180000));
    Rdk_Tick(&r, 10);
    feed(&r, "DISC_DONE\r\n"); /* DONE without ACK must not finish a task. */
    CHECK(r.locked && r.error == 3);
    CHECK(Rdk_Begin(&r, "STOP", 0, 20, 1));
    CHECK(r.error == 3); /* Preserve the original diagnosis. */
    return 0;
}
