#include "rdk_link.h"
#include <stdio.h>
#include <string.h>
#define CHECK(x) do { if (!(x)) { printf("FAIL %d: %s\n", __LINE__, #x); return 1; } } while (0)
static char wire[80];
static unsigned sends;
static bool tx(void *ctx, const char *s, size_t n) {
    (void)ctx; memcpy(wire, s, n); wire[n] = 0; sends++; return true;
}
static void feed(RdkLink *r, const char *s) { while (*s) Rdk_Feed(r, (uint8_t)*s++); }
static int connect(RdkLink *r) {
    Rdk_Init(r, 1, tx, 0);
    CHECK(Rdk_Begin(r, "HELLO", 0, 0, 2000));
    Rdk_Tick(r, 0);
    CHECK(!strcmp(wire, "PING\r\n"));
    feed(r, "PONG\r\n");
    CHECK(!r->active && r->reply == PATH_OK);
    return 0;
}
int main(void) {
    RdkLink r;
    CHECK(connect(&r) == 0);
    CHECK(!Rdk_Begin(&r, "GROUP", 0, 1, 1000));
    CHECK(!Rdk_Begin(&r, "VISION", 0, 1, 1000));
    CHECK(Rdk_Begin(&r, "DISC", 0, 10, 20000));
    Rdk_Tick(&r, 10);
    CHECK(!strcmp(wire, "DISC_START\r\n"));
    unsigned n = sends;
    Rdk_Tick(&r, 600);
    CHECK(sends == n);
    feed(&r, "DISC_ACK\r\n");
    CHECK(r.active && r.reply == PATH_WAIT);
    Rdk_Tick(&r, 20009);
    CHECK(r.active);
    feed(&r, "DISC_DONE\r\n");
    CHECK(!r.active && r.reply == PATH_OK);
    CHECK(connect(&r) == 0);
    CHECK(Rdk_Begin(&r, "DISC", 0, 10, 20000));
    Rdk_Tick(&r, 10); feed(&r, "DISC_ACK\r\n");
    Rdk_Tick(&r, 20010);
    CHECK(r.locked && r.reply == PATH_FAILED);
    feed(&r, "DISC_DONE\r\n");
    CHECK(r.locked && r.reply == PATH_FAILED);
    n = sends;
    CHECK(Rdk_Begin(&r, "STOP", 0, 20011, 1));
    Rdk_Tick(&r, 20011);
    CHECK(sends == n); /* ZHY has no remote STOP. */
    CHECK(connect(&r) == 0);
    CHECK(Rdk_Begin(&r, "DISC", 0, 10, 20000));
    Rdk_Tick(&r, 10); feed(&r, "DISC_DONE\r\n");
    CHECK(r.locked && r.reply == PATH_FAILED);
    CHECK(connect(&r) == 0);
    CHECK(Rdk_Begin(&r, "DISC", 0, 10, 20000));
    Rdk_Tick(&r, 10); feed(&r, "DISC_ERROR\r\n");
    CHECK(r.locked && r.reply == PATH_FAILED);
    CHECK(connect(&r) == 0);
    CHECK(Rdk_Begin(&r, "DISC", 0, 10, 20000));
    Rdk_Tick(&r, 10); Rdk_Tick(&r, 2010);
    CHECK(r.locked && r.reply == PATH_FAILED); /* missing ACK */
    puts("ZHY link tests passed"); return 0;
}
