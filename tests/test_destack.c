#include "path_destack.h"
#include "path_warehouse.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
static PathCommand last;
static unsigned issued;
static bool send(void *ctx,const PathCommand *c) {(void)ctx;last=*c;issued++;return true;}
bool PathHeading_Ready(PathMission *m,uint32_t n,const PathInput *i) {(void)m;(void)n;return i->settled;}
bool PathLine_AlignFour(PathMission *m,uint32_t n,const PathInput *i) {(void)m;(void)n;return i->gray==6;}
bool PathLine_Aligned(const PathMission *m,uint8_t gray) {(void)m;return gray==6;}
static void tick(PathMission *m,PathInput *in,unsigned t) {assert(PathDestack_Tick(m,t,in));}
typedef struct {
 PathMission *m;PathInput *in;
 unsigned queries, checks, picked, placed, balls, admitted;
 unsigned order[3], unload_columns[9];
 bool fallback, stacked;
} Simulation;
static bool simulated_send(void *ctx,const PathCommand *c) {
 Simulation *s=ctx;PathMission *m=s->m;PathInput *in=s->in;
 float direction=m->blue?-1.0f:1.0f;
 if(c->kind==PC_WAREHOUSE_DIGIT) {
  unsigned col=m->point/3;s->queries++;
  in->warehouse_digit_reply=s->fallback?PATH_NONE:PATH_OK;
  in->warehouse_digit=s->order[col];
  in->x_mm=direction*(100+200*col); /* Fresh numeric detection positions. */
 }
 if(c->kind==PC_BLOCK_CHECK) {
  assert(m->destack.scanned && m->destack.row==c->argument);
  assert(m->destack.column==2-s->checks/(m->blue?1:3));
  s->checks++;in->warehouse_digit_reply=PATH_OK;
  in->warehouse_digit=s->stacked ? (m->destack.column==2 ? c->argument : 4) :
      (c->argument==3 ? m->destack.column+1U : 4U);
 }
 if(c->kind==PC_MOVE || c->kind==PC_FINISH_FORWARD) {
  in->x_mm+=direction*c->x;in->y_mm+=direction*c->y;
 }
 if(c->kind==PC_GROUP) {
  in->reply=PATH_OK;
  if(c->argument>=112) {
   if((c->argument-112)%3==1) {s->picked++;assert(!m->destack.carrying);}
   if((c->argument-112)%3==2) {s->placed++;assert(m->destack.carrying);}
  } else if(c->argument>=109 && c->argument<=111) {
   assert(m->destack.cleared && s->checks==(m->blue?3U:9U) && s->placed==3);
   assert((m->inventory.code[m->inventory.current]&15)==(s->fallback?m->destack.column+1U:s->order[m->destack.column]));
   s->unload_columns[s->balls++]=m->destack.column;
  }
 }
 if(c->kind==PC_TURN) {
  for(unsigned i=0;i<(unsigned)c->x;i++) BallInventory_Step(&m->inventory,c->argument!=0);
  in->turn_reply=PATH_OK;
 }
 assert(c->kind!=PC_CANCEL);return true;
}
static void integration(void) {
 const unsigned orders[6][3]={{1,2,3},{1,3,2},{2,1,3},{2,3,1},{3,1,2},{3,2,1}};
 for(unsigned blue=0;blue<2;blue++) for(unsigned order=0;order<7;order++) {
  PathMission m={.result=PATH_RUNNING,.step=13,.phase=WAREHOUSE_FIRST_OFFSET,.send=simulated_send};
  PathInput in={.settled=true,.armed=true,.destack_enabled=true,.warehouse_vision=true,.gray=6,.reply=PATH_OK,.turn_reply=PATH_OK};
  Simulation sim={.m=&m,.in=&in,.fallback=order==6,.stacked=!blue && (order%2)!=0};m.context=&sim;m.blue=blue;
  in.map_yaw_deg=in.yaw_deg=blue?180:0;
  for(unsigned i=0;i<3;i++) sim.order[i]=orders[order%6][i];
  for(unsigned col=1;col<=3;col++) for(unsigned row=1;row<=3;row++) {
   assert(BallInventory_Record(&m.inventory,col*10+row,(row<<4)|col)==BALL_ADDED);
   BallInventory_Step(&m.inventory,false);
  }
  unsigned t=0;
  while(t<10000 && !(m.point==9 && m.phase==WAREHOUSE_ALIGN_HOME)) {
   PathWarehouse_Tick(&m,t++,&in);assert(m.result==PATH_RUNNING);
  }
  assert(t<10000 && sim.checks==(blue?3U:9U) && sim.picked==3 && sim.placed==3 && sim.balls==9);
  assert(sim.queries==(order==6?1U:2U)); /* Third identity inferred, position still visited. */
  for(unsigned i=0;i<9;i++) assert(sim.unload_columns[i]==(blue?2-i/3:i/3));
  assert(!m.destack.carrying && m.destack.occupied==14 && !m.inventory.occupied);
  assert(fabsf((blue?-1.0f:1.0f)*in.x_mm-m.destack.position[blue?0:2])<.01f);
 }
}
int main(void) {
 integration();
 {
  PathMission m={.send=send,.result=PATH_RUNNING,.step=13,.phase=WAREHOUSE_FIRST_DIGIT};
  PathInput in={.armed=true,.settled=false,.destack_enabled=true,.warehouse_vision=true,
      .warehouse_digit_reply=PATH_OK,.warehouse_digit=1};
  unsigned before=issued;
  PathWarehouse_Tick(&m,0,&in);
  assert(m.point==3 && m.phase==WAREHOUSE_MOVE && issued==before);
  PathWarehouse_Tick(&m,1,&in);
  assert(m.phase==WAREHOUSE_FIRST_OFFSET && issued==before);
  in.warehouse_digit_reply=PATH_WAIT;
  PathWarehouse_Tick(&m,2,&in);
  assert(last.kind==PC_FINISH_FORWARD && last.x==300 && issued==before+2);
  in.warehouse_ready=true;PathWarehouse_Tick(&m,3,&in);
  in.x_mm=200;in.warehouse_digit_reply=PATH_OK;in.warehouse_digit=2;
  PathWarehouse_Tick(&m,4,&in);
  assert(m.point==6 && m.phase==WAREHOUSE_MOVE && issued==before+2);
  PathWarehouse_Tick(&m,5,&in);
  assert(last.kind==PC_FINISH_FORWARD && last.x==270 && m.phase==WAREHOUSE_INFERRED_MOVE);
 }
 {
  PathMission m={.send=send,.result=PATH_RUNNING,.step=13,.phase=WAREHOUSE_SELECT_BALL};
  m.destack.enabled=m.destack.scanned=m.destack.cleared=true;
  assert(BallInventory_Record(&m.inventory,1,0x11)==BALL_ADDED);
  m.inventory.current=1;
  PathInput in={.armed=true,.settled=true,.gray=0,.turn_reply=PATH_WAIT};
  PathWarehouse_Tick(&m,0,&in);assert(last.kind==PC_TURN && m.phase==WAREHOUSE_TURN);
  unsigned before=issued;PathWarehouse_Tick(&m,4,&in);assert(issued==before && m.phase==WAREHOUSE_TURN);
  m.inventory.current=0;in.turn_reply=PATH_OK;
  PathWarehouse_Tick(&m,5,&in);PathWarehouse_Tick(&m,6,&in);PathWarehouse_Tick(&m,7,&in);
  assert(m.phase==WAREHOUSE_UNLOAD && last.kind==PC_GROUP && last.argument==111);
 }
 for(unsigned blue=0;blue<2;blue++) for(unsigned source=1;source<=3;source++) for(unsigned digit=1;digit<=3;digit++) {
  PathMission m={.send=send,.blue=blue,.result=PATH_RUNNING,.step=13,.phase=WAREHOUSE_SELECT_BALL};
  PathInput in={.settled=true,.destack_enabled=true,.warehouse_vision=true,.map_yaw_deg=blue?180:0,.yaw_deg=blue?180:0};
  for(unsigned col=0;col<3;col++) {
   m.point=col*3;m.phase=WAREHOUSE_SELECT_BALL;in.x_mm=(blue?-1.0f:1.0f)*col*200;
   tick(&m,&in,col);assert(m.destack.position[col]>(float)(col*200)-.01f);
  }
  assert(m.destack.scanned && m.destack.column==2 && m.phase==DESTACK_POSE);
  m.destack.row=source;
  tick(&m,&in,10);assert(last.kind==PC_GROUP && last.argument==121-3*source);
  unsigned before=issued;in.reply=PATH_WAIT;tick(&m,&in,11);assert(issued==before);
  in.reply=PATH_OK;tick(&m,&in,12);assert(m.phase==DESTACK_CHECK);
  tick(&m,&in,13);assert(last.kind==PC_BLOCK_CHECK && last.argument==source);
  in.warehouse_digit_reply=PATH_OK;in.warehouse_digit=digit;
  tick(&m,&in,14);assert(m.phase==DESTACK_PICK);
  tick(&m,&in,15);assert(last.argument==122-3*source);
  tick(&m,&in,16);assert(m.destack.carrying && m.phase==DESTACK_TO_FOURTH);
  float spacing=blue?200:190;
  assert(fabsf(m.destack.position[2]-m.destack.position[1]-spacing)<.01f);
  assert(fabsf(m.destack.position[1]-m.destack.position[0]-spacing)<.01f);
  tick(&m,&in,17);assert(last.kind==PC_MOVE && fabsf(last.x-spacing)<.01f);
  in.settled=false;before=issued;tick(&m,&in,18);assert(issued==before);
  in.settled=true;in.x_mm=(blue?-1:1)*(400+spacing);tick(&m,&in,19);
  tick(&m,&in,20);assert(last.kind==PC_GROUP && last.argument==123-3*digit);
  tick(&m,&in,21);assert(!m.destack.carrying && (m.destack.occupied&(1U<<digit)));
  tick(&m,&in,22);assert(last.kind==PC_MOVE && fabsf(last.x+spacing+(blue?0:20))<.01f);
  in.x_mm=(blue?-1:1)*(400-(blue?0:20));tick(&m,&in,23);
  assert(m.phase==(!blue && source>1 ? DESTACK_POSE : DESTACK_NEXT));
  if(!blue && source>1) assert(m.destack.column==2 && m.destack.row==source-1);
  else assert(m.destack.column==1);
  m.destack.cleared=true;m.destack.column=0;m.point=2;in.x_mm=0;assert(PathDestack_Advance(&m,30));assert(m.point==3 && m.destack.column==1);
  tick(&m,&in,31);assert(last.kind==PC_MOVE && fabsf(last.x-(400-spacing))<.01f);
  in.x_mm=(blue?-1:1)*(400-spacing);tick(&m,&in,32);assert(m.phase==WAREHOUSE_SELECT_BALL && m.destack.row==3);
  m.destack.column=2;m.destack.unloaded=3;m.point=8;in.x_mm=(blue?-1:1)*400;
  before=issued;assert(PathDestack_Advance(&m,40));tick(&m,&in,41);
  assert(issued==before && m.phase==WAREHOUSE_ALIGN_HOME && m.point==9);
 }
 PathMission m={.send=send,.result=PATH_RUNNING,.phase=DESTACK_CHECK};m.destack.enabled=true;m.destack.row=3;
 PathInput in={.settled=true,.warehouse_digit_reply=PATH_OK};
 tick(&m,&in,0);in.warehouse_digit=0;tick(&m,&in,1);assert(m.phase==DESTACK_POSE && m.destack.row==2 && !m.waiting);
 m.phase=DESTACK_CHECK; tick(&m,&in,2);in.warehouse_digit=4;tick(&m,&in,3);assert(m.destack.row==1 && m.phase==DESTACK_POSE);
 m.phase=DESTACK_CHECK;m.waiting=true;m.destack.occupied=2;in.warehouse_digit=1;
 tick(&m,&in,4);assert(m.result==PATH_ERROR && last.kind==PC_CANCEL);
 m.result=PATH_RUNNING;m.phase=DESTACK_PICK;m.waiting=true;in.reply=PATH_FAILED;
 tick(&m,&in,5);assert(m.result==PATH_ERROR && !m.destack.carrying);
 puts("destack pose/pick/place, traversal, unknown/empty and failures passed");return 0;
}
