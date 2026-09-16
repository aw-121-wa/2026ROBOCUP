#include "host_command.h"
#include <stdio.h>
#include <string.h>
#define CHECK(x)                                                                                   \
    do                                                                                             \
    {                                                                                              \
        if (!(x))                                                                                  \
        {                                                                                          \
            printf("FAIL line %d: %s\n", __LINE__, #x);                                            \
            return 1;                                                                              \
        }                                                                                          \
    } while (0)
static int feed(HostParser *p, const char *s, HostCommand *c)
{
    int result = 0;
    while (*s)
    {
        int r = HostCommand_Feed(p, (uint8_t)*s++, c);
        if (r)
            result = r;
    }
    return result;
}
int main(void)
{
    HostParser p = {0};
    HostCommand c = {0};
    CHECK(feed(&p, "PATH\n", &c) == HOST_OK);
    CHECK(HostCommand_Check(&c, false, false) == HOST_NOT_READY);
    CHECK(HostCommand_Check(&c, true, true) == HOST_BUSY);
    CHECK(HostCommand_Check(&c, true, false) == HOST_OK);
    CHECK(feed(&p, "PATH 1\n", &c) == HOST_SYNTAX);
    CHECK(feed(&p, "ARM\r\n", &c) == HOST_OK && c.kind == HOST_ARM);
    CHECK(feed(&p, "\r\n  \n", &c) == HOST_IDLE);
    CHECK(feed(&p, "FOR", &c) == HOST_IDLE);
    CHECK(feed(&p, "WARD 100\n", &c) == HOST_OK && c.kind == HOST_FORWARD && c.distance_mm == 100);
    CHECK(feed(&p, "FORWARD -100\n", &c) == HOST_OK && c.distance_mm == -100);
    CHECK(feed(&p, " SHIFT +12.5 \r", &c) == HOST_OK && c.kind == HOST_SHIFT &&
          c.distance_mm == 12.5f);
    CHECK(feed(&p, "SHIFT -1\n", &c) == HOST_OK && c.distance_mm == -1);
    CHECK(feed(&p, "SHIFT 5000\n", &c) == HOST_OK);
    const char *bad[] = {"ARM 1\n",     "STOP NOW\n",   "FORWARD\n",   "FORWARD nan\n",
                         "SHIFT inf\n", "SHIFT 1abc\n", "SHIFT 1 2\n", "SHIFT 1e2\n",
                         "ARMjunk\n",   "SHIFT --1\n",  "SHIFT .\n"};
    for (size_t i = 0; i < sizeof(bad) / sizeof(bad[0]); ++i)
        CHECK(feed(&p, bad[i], &c) == HOST_SYNTAX);
    CHECK(feed(&p, "SHIFT 0\n", &c) == HOST_RANGE);
    CHECK(feed(&p, "SHIFT 0.5\n", &c) == HOST_RANGE);
    CHECK(feed(&p, "SHIFT -5001\n", &c) == HOST_RANGE);
    CHECK(feed(&p, "SHIFT 9999999999999999999999999999999999999999999\n", &c) == HOST_RANGE);
    for (int i = 0; i < 100; ++i)
        CHECK(HostCommand_Feed(&p, 'A', &c) == HOST_IDLE);
    CHECK(feed(&p, "STOP\n", &c) == HOST_SYNTAX); /* no suffix execution */
    CHECK(feed(&p, "STOP\n", &c) == HOST_OK && c.kind == HOST_STOP);
    CHECK(HostCommand_Check(&c, false, true) == HOST_OK);
    c.kind = HOST_ARM;
    CHECK(HostCommand_Check(&c, false, false) == HOST_OK);
    CHECK(HostCommand_Check(&c, true, true) == HOST_BUSY);
    c.kind = HOST_FORWARD;
    CHECK(HostCommand_Check(&c, false, false) == HOST_NOT_READY);
    CHECK(HostCommand_Check(&c, true, true) == HOST_BUSY);
    CHECK(HostCommand_Check(&c, true, false) == HOST_OK);
    c.kind = HOST_SHIFT;
    CHECK(HostCommand_Check(&c, false, false) == HOST_NOT_READY);
    CHECK(HostCommand_Check(&c, true, true) == HOST_BUSY);
    CHECK(HostCommand_Check(&c, true, false) == HOST_OK);
    CHECK(feed(&p, "PING\r\n", &c) == HOST_OK && c.kind == HOST_PING);
    CHECK(HostCommand_Check(&c, false, false) == HOST_OK);
    CHECK(HostCommand_Check(&c, true, true) == HOST_BUSY);
    CHECK(feed(&p, "RDK_RESET\r\n", &c) == HOST_OK && c.kind == HOST_RDK_RESET);
    CHECK(HostCommand_Check(&c, false, false) == HOST_OK);
    CHECK(feed(&p, "DISC\r\n", &c) == HOST_OK && c.kind == HOST_DISC);
    CHECK(HostCommand_Check(&c, false, false) == HOST_NOT_READY);
    CHECK(HostCommand_Check(&c, true, false) == HOST_OK);
    puts("Host command tests passed");
    return 0;
}
