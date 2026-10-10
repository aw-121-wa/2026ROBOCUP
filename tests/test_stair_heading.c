#include "path_chassis.h"
#include <assert.h>
#include <math.h>
static PathCommand last;
static bool reject;
static unsigned turns;
static bool send(void *p,const PathCommand *c) {
 (void)p;last=*c;
 if(c->kind==PC_MAP_AXIS || c->kind==PC_HOME_ALIGN || c->kind==PC_LINE_CALIBRATE) turns++;
 return !reject;
}
int main(void) {
 for(unsigned blue=0;blue<2;blue++) for(unsigned step=3;step<=13;step++) {
  if(step!=3 && step!=9 && step!=12 && step!=13) continue;
  for(unsigned gray=0;gray<16;gray++) {
   PathMission m;Path_Init(&m,send,0);m.blue=blue;m.step=step;m.result=PATH_RUNNING;
   PathInput in={.settled=true,.gray=gray,.map_yaw_deg=170};
   turns=0;
   bool aligned=gray==6 || ((step==12 || step==13) && gray==4);
   assert(PathLine_Aligned(&m,gray)==aligned);
   assert(!PathLine_AlignFour(&m,0,&in));
   if(aligned) {
    assert(last.kind==PC_HOLD);
    assert(!PathLine_AlignFour(&m,5,&in));
    assert(PathLine_AlignFour(&m,105,&in));
   } else {
    assert(last.kind==PC_MAP_SEARCH && (blue ? last.y>0 : last.y<0));
   }
   assert(turns==0);
  }
 }
 PathMission m;Path_Init(&m,send,0);m.step=9;m.result=PATH_RUNNING;
 PathInput in={.settled=true,.gray=6,.map_yaw_deg=175};
 assert(!PathLine_Align(&m,0,&in,50000,30));
 in.settled=false;in.gray=15;assert(!PathLine_Align(&m,5,&in,50000,30));
 in.settled=true;assert(!PathLine_Align(&m,10,&in,50000,30));assert(last.kind==PC_MAP_SEARCH);
 in.gray=6;assert(!PathLine_Align(&m,20,&in,50000,30));
 assert(!PathLine_Align(&m,25,&in,50000,30));assert(PathLine_Align(&m,125,&in,50000,30));
 assert(!turns);
 Path_Init(&m,send,0);m.step=9;m.result=PATH_RUNNING;in.map_yaw_deg=NAN;
 assert(!PathLine_AlignFour(&m,0,&in));assert(m.result==PATH_ERROR);
 Path_Init(&m,send,0);m.step=9;m.result=PATH_RUNNING;in.map_yaw_deg=180;reject=true;
 assert(!PathLine_AlignFour(&m,0,&in));assert(m.result==PATH_ERROR);
 return 0;
}
