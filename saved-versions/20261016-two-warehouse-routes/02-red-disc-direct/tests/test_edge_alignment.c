#include "path_chassis.h"
#include <assert.h>
#include <math.h>
static PathCommand last;
static bool send(void *p,const PathCommand *c){(void)p;last=*c;return true;}
int main(void){
 for(unsigned step=9;step<=13;step+=4){
  PathMission m={.send=send,.step=step,.result=PATH_RUNNING};
  PathInput in={.gray=6,.settled=true,.map_yaw_deg=179};
  assert(!PathLine_EdgeHeading(&m,0,&in) && last.kind==PC_LINE_SWEEP && last.speed<0);
  in.settled=false;in.gray=4;in.map_yaw_deg=177;
  assert(!PathLine_EdgeHeading(&m,100,&in) && last.kind==PC_HOLD);
  assert(!PathLine_EdgeHeading(&m,110,&in) && m.edge_stage==2);
  in.settled=true;PathLine_EdgeHeading(&m,200,&in);assert(last.speed>0);
  in.settled=false;in.gray=2;in.map_yaw_deg=-179;
  PathLine_EdgeHeading(&m,300,&in);assert(m.edge_stage==4 && fabsf(m.edge_mid-179)<.01f);
  in.settled=true;PathLine_EdgeHeading(&m,400,&in);assert(last.kind==PC_HOME_ALIGN);
  in.gray=6;in.map_yaw_deg=178.8f;
  assert(!PathLine_EdgeHeading(&m,500,&in) && last.kind==PC_HOME_ALIGN);
  in.map_yaw_deg=179.05f;
  assert(PathLine_EdgeHeading(&m,600,&in) && last.kind==PC_LINE_ANCHOR);
  assert(step==9?m.stair_heading_calibrated:m.warehouse_heading_calibrated);
 }
 PathMission m={.send=send,.step=9,.result=PATH_RUNNING};
 PathInput in={.gray=6,.settled=true,.map_yaw_deg=180};
 PathLine_EdgeHeading(&m,0,&in);in.map_yaw_deg=193;
 assert(!PathLine_EdgeHeading(&m,500,&in) && m.result==PATH_TIMEOUT);
 return 0;
}
