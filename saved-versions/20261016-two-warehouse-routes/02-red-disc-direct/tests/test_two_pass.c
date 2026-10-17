#include "path_mission.h"
#include "path_chassis.h"
#include "path_warehouse.h"
#include "path_config.h"
#include "rdk_link.h"
#include <assert.h>
#include <string.h>
#include <stdio.h>
static PathCommand last;
static unsigned count;
static bool send(void *ctx,const PathCommand *c){(void)ctx;last=*c;count++;return true;}
static void init(PathMission *m,unsigned blue){Path_Init(m,send,0);m->blue=blue;m->result=PATH_RUNNING;}
static void tick(PathMission *m,PathInput *in){PathWarehouse_Tick(m,1000000,in);assert(m->result==PATH_RUNNING);}
static void scan_ready(PathMission *m,PathInput *in){in->warehouse_digit_reply=PATH_WAIT;in->warehouse_ready=true;tick(m,in);tick(m,in);assert(last.kind==PC_BODY);}
static void unload_empty_column(PathMission *m,PathInput *in){
 for(unsigned i=0;i<3;i++) {assert(m->phase==WAREHOUSE_SELECT_BALL);tick(m,in);}
}
static void routes(void){
 for(unsigned blue=0;blue<2;blue++) {
  PathMission m;PathInput in={.armed=true,.settled=true};init(&m,blue);m.step=4;
  Path_Tick(&m,1,&in);assert(last.kind==PC_GROUP&&last.argument==1&&m.step==4);
  unsigned n=count;Path_Tick(&m,2,&in);assert(count==n&&m.step==4);
  in.reply=PATH_OK;Path_Tick(&m,3,&in);assert(!m.prep_pending&&m.step==(PATH_RED_DISC_TO_WAREHOUSE&&!blue?14:5));
  if(m.step==14){
   Path_Tick(&m,4,&in);assert(last.kind==PC_MOVE&&last.x==-1400);
   in.settled=false;Path_Tick(&m,5,&in);assert(m.phase==0);
   in.settled=true;Path_Tick(&m,6,&in);assert(m.phase==1);
   in.map_yaw_deg=90;Path_Tick(&m,7,&in);assert(last.kind==PC_HOME_ALIGN&&last.x==0);
   in.map_yaw_deg=0;Path_Tick(&m,8,&in);assert(m.phase==2);
   in.reply=PATH_WAIT;Path_Tick(&m,9,&in);assert(last.kind==PC_GROUP&&last.argument==3);
   Path_Tick(&m,10,&in);assert(m.phase==2);
   in.reply=PATH_OK;Path_Tick(&m,11,&in);assert(m.phase==3);
   in.gray=0;Path_Tick(&m,12,&in);assert(last.kind==PC_BODY&&last.y>0);
   in.gray=4;Path_Tick(&m,13,&in);assert(last.kind==PC_HOLD);
   Path_Tick(&m,14,&in);assert(m.step==12&&m.phase==1);
  }
  /* DISC finishes after RFID count. G1 must only run once, then wait for DONE. */
  init(&m,blue);m.step=4;m.prep_pending=m.disc_depart_pending=true;
  in.reply=PATH_WAIT;in.settled=true;Path_Tick(&m,1,&in);assert(m.step==4);
  in.reply=PATH_OK;Path_Tick(&m,2,&in);assert(m.step==4&&last.argument==1&&m.prep_pending);
  n=count;in.reply=PATH_WAIT;Path_Tick(&m,3,&in);assert(count==n&&m.step==4);
  in.reply=PATH_OK;Path_Tick(&m,4,&in);assert(m.step!=4);
 }
}
static void scans(void){
 for(unsigned blue=0;blue<2;blue++) {
  PathMission m;PathInput in={.armed=true,.settled=true,.warehouse_vision=true,.gray=2};
  init(&m,blue);m.step=13;m.phase=WAREHOUSE_ENTRY_BACK;
  tick(&m,&in);assert(last.kind==PC_HOLD);tick(&m,&in);assert(last.kind==PC_WAREHOUSE_DIGIT);
  scan_ready(&m,&in);assert(last.x>0&&m.warehouse_pass==0); /* Initial front OFF is not end. */
  in.gray=6;tick(&m,&in);assert(m.warehouse_edge_armed);
  in.gray=2;tick(&m,&in);assert(m.phase==WAREHOUSE_SCAN_STOP&&last.kind==PC_WAREHOUSE_STOP);
  tick(&m,&in);assert(m.phase==WAREHOUSE_SCAN_STOP); /* Await stop ACK. */
  in.warehouse_digit_reply=PATH_NONE;tick(&m,&in);assert(m.warehouse_pass==1&&last.kind==PC_WAREHOUSE_DIGIT);
  scan_ready(&m,&in);assert(last.x<0);
  in.gray=4;tick(&m,&in);assert(last.kind==PC_WAREHOUSE_STOP);
  in.warehouse_digit_reply=PATH_NONE;tick(&m,&in);
  assert(m.warehouse_mode==WAREHOUSE_DEFAULT_ORDER&&last.kind==PC_MOVE&&last.x==(blue?200:215));
  tick(&m,&in);assert(m.phase==WAREHOUSE_SELECT_BALL);
  assert((PathWarehouse_Code(&m)&15)==1);
  unload_empty_column(&m,&in);tick(&m,&in);assert(last.kind==PC_MOVE&&last.x==200);
  /* Forward and reverse detections both perform body-forward20, then logical number. */
  for(unsigned pass=0;pass<2;pass++){
   init(&m,blue);m.step=13;m.phase=WAREHOUSE_FIRST_DIGIT;m.warehouse_mode=WAREHOUSE_DIGIT_ORDER;
   m.warehouse_pass=pass;in.gray=6;in.warehouse_digit_reply=PATH_OK;in.warehouse_digit=3;
   tick(&m,&in);assert(last.kind==PC_HOLD&&m.phase==WAREHOUSE_BRAKE);
   in.x_mm=pass ? -3 : 3; /* Braking distance counts toward the20mm offset. */
   tick(&m,&in);assert(last.kind==PC_MOVE&&last.x==(pass?23:17)&&last.speed==16);
   in.x_mm=0;
   in.settled=false;tick(&m,&in);assert(m.phase==WAREHOUSE_INFERRED_MOVE);
   in.settled=true;tick(&m,&in);assert((PathWarehouse_Code(&m)&15)==3);
   unload_empty_column(&m,&in);assert(m.warehouse_used==8);tick(&m,&in);
   assert(last.kind==PC_WAREHOUSE_DIGIT&&last.argument==8);
   scan_ready(&m,&in);assert(last.x==(pass?-16:16));
  }
 }
}
static void partial_fallback(void){
 PathMission m;PathInput in={.armed=true,.settled=true,.warehouse_vision=true,.gray=6};
 init(&m,0);m.step=13;m.phase=WAREHOUSE_FIRST_DIGIT;m.warehouse_mode=WAREHOUSE_DIGIT_ORDER;
 m.inventory.occupied=7;m.inventory.code[0]=0x12;m.inventory.code[1]=0x11;m.inventory.code[2]=0x13;
 in.warehouse_digit_reply=PATH_OK;in.warehouse_digit=2;
 tick(&m,&in);tick(&m,&in);tick(&m,&in);
 unsigned guard=0;
 while(m.phase!=WAREHOUSE_MOVE){
  assert(++guard<30);in.reply=PATH_OK;tick(&m,&in);
 }
 assert(m.inventory.placed==1 && m.inventory.occupied==6 && m.warehouse_used==4);
 tick(&m,&in);scan_ready(&m,&in);
 in.gray=2;tick(&m,&in);in.warehouse_digit_reply=PATH_NONE;tick(&m,&in);scan_ready(&m,&in);
 in.gray=4;tick(&m,&in);in.warehouse_digit_reply=PATH_NONE;tick(&m,&in);
 assert(m.point==0&&m.warehouse_mode==WAREHOUSE_DEFAULT_ORDER);
 /* Simulate acknowledged table turns and arm completions for remaining physical columns. */
 for(guard=0;m.point<9 && guard<100;guard++){
  if(m.phase==WAREHOUSE_TURN){
   unsigned hops=(unsigned)last.x;
   for(unsigned i=0;i<hops;i++) BallInventory_Step(&m.inventory,last.argument!=0);
  }
  in.reply=in.turn_reply=PATH_OK;tick(&m,&in);
 }
 assert(m.point==9&&m.inventory.placed==3&&!m.inventory.occupied);
}
static char wire[80];
static bool tx(void *ctx,const char *s,size_t n){(void)ctx;memcpy(wire,s,n);wire[n]=0;return true;}
static void feed(RdkLink *r,const char *s){while(*s)Rdk_Feed(r,(uint8_t)*s++);}
static void cancel_test(void){
 RdkLink r;Rdk_Init(&r,1,tx,0);r.stage=2;r.no_timeout=true;
 assert(Rdk_WarehouseBegin(&r,0,0,100));Rdk_Tick(&r,0);
 assert(Rdk_WarehouseEnd(&r));Rdk_Tick(&r,1000000);assert(strstr(wire,"WAREHOUSE_STOP ")==wire);
 char line[80];snprintf(line,sizeof(line),"WAREHOUSE_DIGIT %lu 2\r\n",(unsigned long)r.warehouse_token);
 feed(&r,line);assert(r.warehouse_reply==PATH_WAIT&&r.warehouse_active);
 snprintf(line,sizeof(line),"WAREHOUSE_STOPPED %lu\r\n",(unsigned long)r.warehouse_token+1);feed(&r,line);assert(r.warehouse_active);
 snprintf(line,sizeof(line),"WAREHOUSE_STOPPED %lu\r\n",(unsigned long)r.warehouse_token);feed(&r,line);
 assert(!r.warehouse_active&&!r.locked&&r.warehouse_reply==PATH_NONE);
 assert(Rdk_WarehouseBegin(&r,4,1000001,100));
}
int main(void){routes();scans();partial_fallback();cancel_test();puts("two-pass route/scan/protocol tests passed");return 0;}
