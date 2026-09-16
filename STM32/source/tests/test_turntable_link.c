#include "turntable_link.h"
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
    uint8_t data[8][13];
    size_t len[8];
    unsigned count;
} Port;
static bool tx(void *ctx, const uint8_t *d, size_t n)
{
    Port *p = ctx;
    memcpy(p->data[p->count], d, n);
    p->len[p->count++] = n;
    return true;
}
int main(void)
{
    Port p = {0};
    TurntableLink t;
    Turn_Init(&t, tx, &p);
    CHECK(Turn_Start(&t, false, 0));
    Turn_Tick(&t, 0, true);
    CHECK(p.count == 1);
    Turn_Tick(&t, 1, false);
    CHECK(p.count == 1);
    for (unsigned n = 2; n < 100; n++)
        Turn_Tick(&t, n, true);
    CHECK(p.count == 4);
    uint8_t pos[] = {5, 0xfd, 0, 3, 0xe8, 0, 0, 0, 5, 0, 0, 1, 0x6b};
    CHECK(p.len[2] == 13 && !memcmp(p.data[2], pos, 13));
    CHECK(p.data[1][1] == 0xff && p.data[3][2] == 0x66);
    CHECK(t.reply == PATH_WAIT);
    Turn_Tick(&t, 851, true);
    CHECK(t.reply == PATH_OK);
    CHECK(Turn_Start(&t, true, 851));
    Turn_Tick(&t, 851, true);
    Turn_Stop(&t, 852);
    Turn_Tick(&t, 853, false);
    CHECK(p.count == 5);
    for (unsigned n = 854; n < 865; n++)
        Turn_Tick(&t, n, true);
    CHECK(p.count == 7 && p.data[5][1] == 0xfe);
    CHECK(!t.pending);
    puts("Turntable transport tests passed");
    return 0;
}
