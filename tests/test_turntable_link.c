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
    Turn_Tick(&t, t.at+839, true); CHECK(t.pending);
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
    p=(Port){0}; Turn_Init(&t,tx,&p);
    CHECK(!Turn_StartSteps(&t,false,0,0));
    CHECK(!Turn_StartSteps(&t,false,7,0));
    CHECK(Turn_StartSteps(&t,true,6,0));
    CHECK(!Turn_StartSteps(&t,false,1,0));
    for(unsigned n=0;n<100;n++) Turn_Tick(&t,n,true);
    CHECK(p.count==4 && p.data[2][2]==1);
    CHECK(p.data[2][6]==0 && p.data[2][7]==0 && p.data[2][8]==30 && p.data[2][9]==0);
    Turn_Tick(&t,t.at+1439,true); CHECK(t.pending);
    Turn_Tick(&t,t.at+1440,true); CHECK(!t.pending && t.reply==PATH_OK);
    p=(Port){0}; Turn_Init(&t,tx,&p);
    CHECK(Turn_StartSteps(&t,false,1,0));
    for(unsigned n=0;n<100;n++) Turn_Tick(&t,n,true);
    Turn_Tick(&t,t.at+239,true); CHECK(t.pending);
    Turn_Tick(&t,t.at+240,true); CHECK(!t.pending && t.reply==PATH_OK);
    p=(Port){0};Turn_Init(&t,tx,&p);t.no_timeout=true;
    CHECK(Turn_StartSteps(&t,false,1,0));Turn_Tick(&t,60000,false);
    CHECK(t.pending && t.reply==PATH_WAIT);
    t.no_timeout=false;Turn_Tick(&t,60001,false);CHECK(t.reply==PATH_FAILED);
    puts("Turntable transport tests passed");
    return 0;
}
