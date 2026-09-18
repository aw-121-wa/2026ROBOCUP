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
static int begin_disc(RdkLink *r, uint32_t started, uint32_t timeout) {
    CHECK(connect(r) == 0);
    CHECK(Rdk_Begin(r, "DISC", 0, started, timeout));
    Rdk_Tick(r, started);
    CHECK(!strcmp(wire, "DISC_START\r\n"));
    feed(r, "DISC_ACK\r\n");
    CHECK(r->active && r->stage == 4 && r->reply == PATH_WAIT);
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

    CHECK(begin_disc(&r, 100, 5000) == 0);
    uint32_t started = r.started, timeout = r.timeout;
    for (uint8_t expected = 1; expected <= 5; ++expected) {
        char line[32];
        snprintf(line, sizeof(line), "DISC_ACTION_DONE %u\r\n", expected);
        feed(&r, line);
        CHECK(r.active && r.stage == 4 && r.reply == PATH_WAIT);
        CHECK(r.started == started && r.timeout == timeout);
        uint8_t taken = 0;
        CHECK(Rdk_TakeDiscActionDone(&r, &taken) && taken == expected);
        CHECK(!Rdk_TakeDiscActionDone(&r, &taken));
        CHECK(Rdk_SendDiscRfidOk(&r, expected));
        Rdk_Tick(&r, 100 + expected);
        snprintf(line, sizeof(line), "DISC_RFID_OK %u\r\n", expected);
        CHECK(!strcmp(wire, line));
        CHECK(r.active && r.stage == 4 && r.started == started && r.timeout == timeout);
    }
    Rdk_Tick(&r, 5099); CHECK(r.active);
    Rdk_Tick(&r, 5100); CHECK(r.locked && r.reply == PATH_FAILED);

    CHECK(begin_disc(&r, 10, 20000) == 0);
    feed(&r, "DISC_ACTION_DONE 1\r\n");
    feed(&r, "DISC_ACTION_DONE 1\r\n");
    CHECK(r.locked && r.reply == PATH_FAILED);
    CHECK(begin_disc(&r, 10, 20000) == 0);
    feed(&r, "DISC_ACTION_DONE 2\r\n");
    CHECK(r.locked && r.reply == PATH_FAILED);
    CHECK(begin_disc(&r, 10, 20000) == 0);
    CHECK(!Rdk_SendDiscRfidOk(&r, 1));
    feed(&r, "DISC_ACTION_DONE 1\r\n");
    CHECK(Rdk_SendDiscRfidOk(&r, 1));
    CHECK(!Rdk_SendDiscRfidOk(&r, 1));
    feed(&r, "DISC_DONE\r\n");
    CHECK(!r.active && r.stage == 5 && r.reply == PATH_OK);
    CHECK(begin_disc(&r, 1000, 5000) == 0);
    n = sends;
    CHECK(Rdk_Begin(&r, "STOP", 0, 1100, 1));
    CHECK(r.active && r.stage == 4 && r.started == 1000 && r.timeout == 5000);
    Rdk_Tick(&r, 1100);
    CHECK(sends == n + 1 && !strcmp(wire, "DISC_CANCEL\r\n"));
    CHECK(r.locked && !r.active && r.reply == PATH_FAILED);
    puts("ZHY link tests passed"); return 0;
}
