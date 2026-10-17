#include "ball_inventory.h"
#include <stdio.h>
#include <string.h>
#define CHECK(x) do { if (!(x)) { printf("FAIL %d: %s\n", __LINE__, #x); return 1; } } while (0)
static int missing_codes(void) {
    for(unsigned missing=0;missing<9;missing++) {
        BallInventory b={.occupied=511};
        for(unsigned i=0;i<9;i++) if(i!=missing) {
            b.code[i]=(uint8_t)(((i/3+1)<<4)|(i%3+1)); b.uid[i]=100+i;
        }
        CHECK(BallInventory_AssignMissing(&b)==1);
        CHECK(b.code[missing]==(((missing/3+1)<<4)|(missing%3+1)));
        CHECK(b.uid[missing]==0 && b.inferred==(1U<<missing));
        CHECK(BallInventory_AssignMissing(&b)==0);
        b.occupied=0; CHECK(BallInventory_AssignMissing(&b)==0);
    }
    BallInventory b={.occupied=4095};
    CHECK(BallInventory_AssignMissing(&b)==9 && b.inferred==511);
    CHECK(b.code[9]==0 && b.code[10]==0 && b.code[11]==0);
    CHECK(BallInventory_AssignMissing(&b)==0);
    b=(BallInventory){.occupied=3,.uncertain=true};
    CHECK(BallInventory_AssignMissing(&b)==0);
    b.uncertain=false;b.code[0]=0x11;b.code[1]=0x11;
    CHECK(BallInventory_AssignMissing(&b)==0);
    return 0;
}
int main(void) {
    CHECK(missing_codes()==0);
    for (unsigned start=0;start<12;start++) for (unsigned target=0;target<12;target++) {
        BallInventory seek={.current=(uint8_t)start};
        unsigned steps=0;
        while (seek.current!=target) {
            BallInventory_Step(&seek,BallInventory_ReverseTo(seek.current,(uint8_t)target));
            CHECK(++steps<=6);
        }
        unsigned forward=(target+12-start)%12, reverse=(start+12-target)%12;
        CHECK(steps==(forward<reverse ? forward : reverse));
    }
    BallInventory b = {0}; uint8_t block[16];
    memset(block,0x23,sizeof(block)); CHECK(BallInventory_Decode(block)==0x23);
    block[15]=0x22; CHECK(BallInventory_Decode(block)==0);
    memset(block,0x35,sizeof(block)); CHECK(BallInventory_Decode(block)==0);
    memset(block,'2',sizeof(block)); CHECK(BallInventory_Decode(block)==0x32); /* literal 0x32 is row 3, col 2 */
    const uint8_t order[]={0x23,0x11,0x32,0x13,0x21,0x33,0x12,0x31,0x22};
    for (unsigned i=0;i<9;i++) {
        CHECK(b.current==i);
        CHECK(BallInventory_Record(&b,100+i,order[i])==BALL_ADDED);
        CHECK(BallInventory_Record(&b,100+i,order[i])==BALL_DUPLICATE);
        CHECK(BallInventory_Find(&b,order[i])==(int)i);
        BallInventory_Step(&b,false);
    }
    CHECK(b.current==9 && b.occupied==0x1ff);
    CHECK(BallInventory_Record(&b,200,0x23)==BALL_CONFLICT);
    CHECK(BallInventory_Record(&b,100,0x22)==BALL_CONFLICT);
    CHECK(BallInventory_Record(&b,300,0x40)==BALL_INVALID);
    CHECK(!BallInventory_Unload(&b,0x11)); /* wrong pocket cannot unload */
    for (unsigned col=1;col<=3;col++) for (unsigned row=1;row<=3;row++) {
        uint8_t code=(uint8_t)((row<<4)|col); int slot=BallInventory_Find(&b,code);
        unsigned steps=0;
        while(b.current!=slot) { BallInventory_Step(&b,true); CHECK(++steps<12); }
        CHECK(BallInventory_Unload(&b,code)); CHECK(BallInventory_Find(&b,code)==-1);
        CHECK(!BallInventory_Unload(&b,code));
    }
    CHECK(b.occupied==0);
    b.current=0; BallInventory_Step(&b,true); CHECK(b.current==11);
    BallInventory_Step(&b,false); CHECK(b.current==0);
    b.uncertain=true; CHECK(BallInventory_Record(&b,400,0x11)==BALL_INVALID);
    CHECK(!BallInventory_Unload(&b,0x11));
    puts("block validation, arbitrary slot order, conflicts and sorted unload passed");
    return 0;
}
