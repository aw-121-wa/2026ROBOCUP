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
    CHECK(connect(&r)==0);
    CHECK(Rdk_Begin(&r,"COLOR",1,100,2000)); Rdk_Tick(&r,100);
    CHECK(!strcmp(wire,"COLOR BLUE\r\n"));
    CHECK(!Rdk_Begin(&r,"DISC",0,101,30000));
    feed(&r,"COLOR_OK BLUE\r\n"); CHECK(!r.active && !r.locked && r.stage==2);
    CHECK(Rdk_Begin(&r,"COLOR",0,200,2000)); Rdk_Tick(&r,200);
    feed(&r,"COLOR_OK BLUE\r\n"); CHECK(r.locked);
    CHECK(connect(&r)==0);
    CHECK(Rdk_Begin(&r,"COLOR",1,100,2000)); Rdk_Tick(&r,2100);
    CHECK(r.locked && r.reply==PATH_FAILED);

    CHECK(connect(&r)==0);
    CHECK(Rdk_WarehouseBegin(&r,0,100,5000)); Rdk_Tick(&r,100);
    CHECK(strstr(wire,"WAREHOUSE_CHECK ")==wire);
    char number[80];
    snprintf(number,sizeof(number),"WAREHOUSE_READY %lu\r\n",(unsigned long)r.warehouse_token+1);
    feed(&r,number); CHECK(!r.warehouse_ready);
    snprintf(number,sizeof(number),"WAREHOUSE_READY %lu\r\n",(unsigned long)r.warehouse_token);
    feed(&r,number); CHECK(r.warehouse_ready && r.warehouse_active);
    snprintf(number,sizeof(number),"WAREHOUSE_DIGIT %lu 3\r\n",(unsigned long)r.warehouse_token+1);
    feed(&r,number); CHECK(r.warehouse_active && !r.locked);
    CHECK(!Rdk_Begin(&r,"GROUP",109,105,30000));
    snprintf(number,sizeof(number),"WAREHOUSE_DIGIT %lu 3\r\n",(unsigned long)r.warehouse_token);
    feed(&r,number); CHECK(!r.warehouse_active && r.warehouse_reply==PATH_OK && r.warehouse_digit==3 && !r.locked);
    CHECK(Rdk_WarehouseBegin(&r,8,200,5000)); CHECK(!r.warehouse_ready); Rdk_Tick(&r,200);
    Rdk_Tick(&r,5200); CHECK(!r.locked && !r.warehouse_active && r.warehouse_reply==PATH_NONE);
    feed(&r,number); CHECK(!r.locked && r.warehouse_reply==PATH_NONE);
    CHECK(Rdk_WarehouseBegin(&r,0,5300,5000)); Rdk_Tick(&r,5300);
    CHECK(Rdk_Begin(&r,"STOP",0,5305,1)); Rdk_Tick(&r,5305);
    CHECK(!strcmp(wire,"DISC_CANCEL\r\n") && !r.warehouse_active && r.locked);


    CHECK(connect(&r)==0);
    for (unsigned g=109;g<=111;g++) {
        char line[48];
        CHECK(Rdk_Begin(&r,"GROUP",g,100,30000)); Rdk_Tick(&r,100);
        snprintf(line,sizeof(line),"GROUP_ACK %u\r\nGROUP_DONE %u\r\n",g,g); feed(&r,line);
        CHECK(!r.active && !r.locked && r.reply==PATH_OK);
    }
    CHECK(connect(&r) == 0);
    CHECK(!Rdk_Begin(&r, "GROUP", 255, 1, 1000));
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
    CHECK(sends == n + 1 && !strcmp(wire, "DISC_CANCEL\r\n"));
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
    Rdk_Init(&r,0,tx,0);r.no_timeout=true;
    CHECK(Rdk_WarehouseBegin(&r,0,0,100));
    Rdk_Tick(&r,60000);CHECK(r.warehouse_active && r.warehouse_reply==PATH_WAIT);
    r.no_timeout=false;Rdk_Tick(&r,60001);CHECK(!r.warehouse_active);
    Rdk_Init(&r,0,tx,0);r.stage=2;r.no_timeout=true;
    CHECK(Rdk_Begin(&r,"GROUP",110,0,100));Rdk_Tick(&r,60000);
    CHECK(r.active && !r.locked);
    r.no_timeout=false;Rdk_Tick(&r,60001);CHECK(r.locked);
    Rdk_Init(&r,0,tx,0);r.stage=2;
    CHECK(Rdk_Begin(&r,"PILLAR",0,0,300000));Rdk_Tick(&r,0);
    feed(&r,"PILLAR_ACK\r\nPILLAR_CAMERA_WAIT\r\n");
    CHECK(r.camera_wait_event && !r.pillar_ready && !r.locked && r.active);
    r.camera_wait_event=false;
    feed(&r,"PILLAR_CAMERA_WAIT\r\n");CHECK(r.camera_wait_event);
    feed(&r,"PILLAR_READY\r\n");CHECK(r.pillar_ready && !r.locked);
    feed(&r,"PILLAR_CAMERA_WAIT\r\n");CHECK(r.locked); /* Not a running-task heartbeat. */
    CHECK(begin_disc(&r,0,60000)==0);
    feed(&r,"DISC_CAMERA_WAIT\r\n");
    CHECK(r.camera_wait_event && !r.disc_camera_ready && !r.locked);
    r.camera_wait_event=false;
    feed(&r,"DISC_CAMERA_READY\r\n");
    CHECK(r.camera_wait_event && r.disc_camera_ready && !r.locked);
    r.camera_wait_event=false;
    feed(&r,"DISC_CAMERA_WAIT\r\n");
    CHECK(!r.camera_wait_event); /* No extension after startup. */
    for(unsigned group=112;group<=120;group++) {
        CHECK(connect(&r)==0);CHECK(Rdk_Begin(&r,"GROUP",group,0,30000));Rdk_Tick(&r,0);
        char expected[40];snprintf(expected,sizeof expected,"GROUP %u\r\n",group);CHECK(!strcmp(wire,expected));
        snprintf(expected,sizeof expected,"GROUP_ACK %u\r\nGROUP_DONE %u\r\n",group,group);
        feed(&r,expected);CHECK(r.reply==PATH_OK && !r.active);
    }
    CHECK(connect(&r)==0);CHECK(Rdk_BlockBegin(&r,3,0,10000));Rdk_Tick(&r,0);
    char result[64];unsigned token=r.warehouse_token;
    snprintf(result,sizeof result,"WAREHOUSE_DIGIT %u 1\r\n",token);feed(&r,result);CHECK(r.warehouse_active);
    snprintf(result,sizeof result,"BLOCK_RESULT %u 4\r\n",token+1);feed(&r,result);CHECK(r.warehouse_active);
    snprintf(result,sizeof result,"BLOCK_RESULT %u 4\r\n",token);feed(&r,result);
    CHECK(!r.warehouse_active && r.warehouse_reply==PATH_OK && r.warehouse_digit==4);
    CHECK(Rdk_BlockBegin(&r,1,1,10000));Rdk_Tick(&r,1);
    feed(&r,result);CHECK(r.warehouse_active); /* Old completed token cannot finish next row. */
    r.no_timeout=true;Rdk_Tick(&r,10002);CHECK(!r.warehouse_active && r.warehouse_reply==PATH_FAILED);
    CHECK(connect(&r)==0);r.sequence=0xfffffffeU;CHECK(Rdk_BlockBegin(&r,2,0,10000));Rdk_Tick(&r,0);
    CHECK(!strcmp(wire,"BLOCK_CHECK 4294967295 2\r\n"));
    feed(&r,"BLOCK_RESULT 4294967295 5\r\n");CHECK(r.warehouse_reply==PATH_FAILED);
    puts("ZHY link tests passed"); return 0;
}
