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
typedef struct
{
    char wire[80];
    unsigned count;
} Port;
static bool tx(void *ctx, const char *s, size_t n)
{
    Port *p = ctx;
    memcpy(p->wire, s, n);
    p->wire[n] = 0;
    p->count++;
    return true;
}
static void feed(RdkLink *r, const char *s)
{
    while (*s)
        Rdk_Feed(r, (uint8_t)*s++);
}
int main(void)
{
    Port p = {0};
    RdkLink r;
    Rdk_Init(&r, 123, tx, &p);
    CHECK(Rdk_Begin(&r, "GROUP", 101, 0, 30000));
    Rdk_Tick(&r, 0);
    CHECK(!strcmp(p.wire, "Q 123 1 GROUP 101\n"));
    feed(&r, "R 122 1 DONE\nR 123 2 DONE\n");
    CHECK(r.reply == PATH_WAIT);
    feed(&r, "R 123 1 ACK\n");
    Rdk_Tick(&r, 500);
    CHECK(p.count == 2 && r.reply == PATH_WAIT);
    CHECK(!Rdk_Begin(&r, "GROUP", 102, 501, 30000));
    feed(&r, "R 123 1 DONE garbage\n");
    CHECK(r.reply == PATH_WAIT);
    feed(&r, "R 123 1 DO\rNE\n");
    CHECK(r.reply == PATH_WAIT);
    feed(&r, "R 123 1 DONE\n");
    CHECK(r.reply == PATH_OK && !r.active);
    CHECK(Rdk_Begin(&r, "DISC", 500, 600, 30500));
    feed(&r, "R 123 1 DONE\n");
    CHECK(r.reply == PATH_WAIT);
    CHECK(Rdk_Begin(&r, "STOP", 0, 700, 30000));
    feed(&r, "R 123 2 DONE\n");
    CHECK(r.reply == PATH_WAIT);
    CHECK(r.interrupted_reply == PATH_OK);
    feed(&r, "R 123 3 DONE\n");
    CHECK(r.reply == PATH_OK);
    CHECK(Rdk_Begin(&r, "VISION", 1000, 0xfffffff0U, 2000));
    Rdk_Tick(&r, 0x800U);
    CHECK(r.reply == PATH_FAILED && r.locked);
    CHECK(!Rdk_Begin(&r, "GROUP", 0, 0x805U, 30000));
    Rdk_Init(&r, 123, tx, &p);
    CHECK(Rdk_Begin(&r, "DISC", 1000, 0, 31000));
    feed(&r, "R 123 1 DONE\n");
    CHECK(Rdk_Begin(&r, "STOP", 0, 1000, 31000));
    CHECK(r.interrupted_reply == PATH_OK);
    feed(&r, "R 123 2 DONE\n");
    CHECK(r.reply == PATH_OK);
    puts("RDK link tests passed");
    return 0;
}
