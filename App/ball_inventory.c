#include "ball_inventory.h"
static bool valid_code(uint8_t code)
{
    return (code >> 4) >= 1 && (code >> 4) <= 3 && (code & 15) >= 1 && (code & 15) <= 3;
}
uint8_t BallInventory_Decode(const uint8_t block[16])
{
    if (!block || !valid_code(block[0])) return 0;
    for (unsigned i=1;i<16;i++) if (block[i]!=block[0]) return 0;
    return block[0];
}
BallRecord BallInventory_Record(BallInventory *b, uint32_t uid, uint8_t code)
{
    if (b->uncertain || b->current>=BALL_SLOT_COUNT || !valid_code(code)) return BALL_INVALID;
    for (unsigned i=0;i<BALL_SLOT_COUNT;i++)
    {
        if (b->code[i] && b->uid[i]==uid)
            return b->code[i]==code ? BALL_DUPLICATE : BALL_CONFLICT;
        if (b->code[i]==code) return BALL_CONFLICT;
    }
    if (b->occupied & (1U<<b->current)) return BALL_FULL;
    b->uid[b->current]=uid;
    b->code[b->current]=code;
    b->occupied|=(uint16_t)(1U<<b->current);
    return BALL_ADDED;
}
int BallInventory_Find(const BallInventory *b, uint8_t code)
{
    for (unsigned i=0;i<BALL_SLOT_COUNT;i++)
        if ((b->occupied & (1U<<i)) && b->code[i]==code) return (int)i;
    return -1;
}
void BallInventory_Step(BallInventory *b, bool reverse)
{
    b->current=(uint8_t)((b->current+(reverse ? BALL_SLOT_COUNT-1U : 1U))%BALL_SLOT_COUNT);
}
bool BallInventory_ReverseTo(uint8_t current, uint8_t target)
{
    unsigned forward = (target + BALL_SLOT_COUNT - current) % BALL_SLOT_COUNT;
    unsigned reverse = (current + BALL_SLOT_COUNT - target) % BALL_SLOT_COUNT;
    return reverse <= forward; /* Deterministic half-circle tie. */
}
bool BallInventory_Unload(BallInventory *b, uint8_t code)
{
    if (b->uncertain || b->current>=BALL_SLOT_COUNT ||
        !(b->occupied & (1U<<b->current)) || b->code[b->current]!=code) return false;
    b->occupied&=(uint16_t)~(1U<<b->current);
    b->placed++;
    return true;
}
