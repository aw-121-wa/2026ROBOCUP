#include "path_mission.h"
#include "path_warehouse.h"
#include "rdk_link.h"
#include <assert.h>
#include <string.h>
static PathCommand last;
static bool send(void *ctx,const PathCommand *c) {(void)ctx;last=*c;return true;}
int main(void) {
 for(unsigned blue=0;blue<2;blue++) {
  PathMission m;Path_Init(&m,send,0);m.blue=blue;
  PathInput in={.armed=true,.settled=true,.reply=PATH_OK};
  assert(Path_Start(&m,0,&in));assert(last.kind==PC_HELLO);
  Path_Tick(&m,1,&in);assert(last.kind==PC_GROUP && last.argument==3);
  in.reply=PATH_WAIT;Path_Tick(&m,2,&in);assert(m.step==0 && m.phase==98);
  in.reply=PATH_OK;Path_Tick(&m,3,&in);assert(m.step==12 && m.phase==0);
  m.step=13;m.point=9;m.phase=WAREHOUSE_ALIGN_HOME;
  PathWarehouse_Tick(&m,4,&in);assert(m.result==PATH_DONE && last.kind==PC_HOLD);
 }
 RdkLink r;Rdk_Init(&r,0,0,0);assert(Rdk_Scan(&r,3));
 assert(!strcmp(r.scan_request,"WH_CAPTURE 1 3\r\n"));
 return 0;
}
