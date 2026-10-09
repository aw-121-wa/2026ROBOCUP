#include "ir_start.h"
#include <assert.h>
static void gesture(IrStart *s,uint32_t t,bool ready) {
 assert(!IrStart_Update(s,t,true,ready));
 assert(!IrStart_Update(s,t+100,true,ready));
 assert(!IrStart_Update(s,t+110,false,ready));
}
int main(void) {
 IrStart s={0};
 assert(!IrStart_Update(&s,0,false,true));
 assert(!IrStart_Update(&s,1000,false,true));
 assert(!IrStart_Update(&s,1010,true,true));
 assert(!IrStart_Update(&s,1050,false,true));
 assert(!IrStart_Update(&s,1200,false,true)); /* glitch */
 gesture(&s,1300,true);
 assert(!IrStart_Update(&s,1509,false,true));
 assert(IrStart_Update(&s,1510,false,true));
 assert(!s.fired); /* Failed ARM may retry with a fresh gesture. */
 gesture(&s,1600,true);assert(IrStart_Update(&s,1810,false,true));
 s.fired=true;IrStart_Clear(&s);
 assert(!IrStart_Update(&s,2000,true,true));
 assert(!IrStart_Update(&s,2200,false,true));
 s=(IrStart){0};gesture(&s,0,false);
 assert(!IrStart_Update(&s,210,false,false));
 assert(IrStart_Update(&s,5000,false,true)); /* Startup gesture retained. */
 s=(IrStart){0};gesture(&s,0,false);
 assert(!IrStart_Update(&s,210,false,false));
 assert(!IrStart_Update(&s,10210,false,true)); /* No stale launch. */
 assert(!IrStart_Update(&s,10310,false,true));
 s=(IrStart){0};gesture(&s,0,false);
 assert(!IrStart_Update(&s,210,false,false));IrStart_Clear(&s);
 assert(!IrStart_Update(&s,300,false,true));
 assert(!IrStart_Update(&s,500,false,true)); /* STOP/fault discards request. */
 s=(IrStart){0};gesture(&s,0xffffff80U,false);
 assert(!IrStart_Update(&s,100,false,false));
 assert(IrStart_Update(&s,200,false,true)); /* tick wrap */
 s=(IrStart){0};
 assert(!IrStart_Update(&s,100,true,true));
 assert(!IrStart_Update(&s,5100,true,true)); /* Strictly greater than five seconds. */
 assert(IrStart_Update(&s,5101,true,true) && s.blue);
 assert(!IrStart_Update(&s,5200,false,true));
 assert(!IrStart_Update(&s,5400,false,true)); /* Long release must not launch red. */
 s=(IrStart){0};
 assert(!IrStart_Update(&s,0,true,false));
 assert(!IrStart_Update(&s,5001,true,false) && s.pending && s.blue);
 assert(IrStart_Update(&s,6000,true,true)); /* No release required after ready. */
 s=(IrStart){0};
 assert(!IrStart_Update(&s,0,true,false));
 assert(!IrStart_Update(&s,5001,true,false));
 assert(!IrStart_Update(&s,15001,true,true) && !s.pending); /* Expired hold cannot renew itself. */
 assert(!IrStart_Update(&s,16000,false,true));
 assert(!IrStart_Update(&s,16200,false,true));
 gesture(&s,17000,true);assert(IrStart_Update(&s,17210,false,true) && !s.blue);
 s=(IrStart){0};
 assert(!IrStart_Update(&s,0xffffff00U,true,false));
 assert(IrStart_Update(&s,4745U,true,true) && s.blue); /* Hold across tick wrap. */
 return 0;
}
