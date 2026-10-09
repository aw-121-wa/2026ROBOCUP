#include "ir_start.h"
#include <assert.h>
int main(void) {
 IrStart s={0};
 assert(!IrStart_Update(&s,0,false,true));
 assert(!IrStart_Update(&s,1000,false,true)); /* idle clear never starts */
 assert(!IrStart_Update(&s,1010,true,true));
 assert(!IrStart_Update(&s,1050,false,true)); /* blocked glitch */
 assert(!IrStart_Update(&s,1200,false,true));
 assert(!IrStart_Update(&s,1300,true,true));
 assert(!IrStart_Update(&s,1400,true,true));
 assert(!IrStart_Update(&s,1410,false,true));
 assert(!IrStart_Update(&s,1509,false,true));
 assert(IrStart_Update(&s,1510,false,true));
 assert(!IrStart_Update(&s,1600,true,true));
 assert(!IrStart_Update(&s,1800,false,true)); /* never retrigger */
 s=(IrStart){0};
 assert(!IrStart_Update(&s,0,true,true));
 assert(!IrStart_Update(&s,100,true,true));
 assert(!IrStart_Update(&s,150,false,false)); /* busy/fault discards gesture */
 assert(!IrStart_Update(&s,160,false,true));
 assert(!IrStart_Update(&s,300,false,true));
 s=(IrStart){0};
 assert(!IrStart_Update(&s,0xffffff80U,true,true));
 assert(!IrStart_Update(&s,0xfffffff0U,true,true));
 assert(!IrStart_Update(&s,0xfffffff8U,false,true));
 assert(IrStart_Update(&s,100,false,true)); /* tick wrap */
 return 0;
}
