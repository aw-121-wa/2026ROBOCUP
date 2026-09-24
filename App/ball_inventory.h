#ifndef BALL_INVENTORY_H
#define BALL_INVENTORY_H
#include <stdbool.h>
#include <stdint.h>
#define BALL_SLOT_COUNT 12U
typedef struct {
    uint32_t uid[BALL_SLOT_COUNT];
    uint8_t code[BALL_SLOT_COUNT]; /* High nibble row, low nibble column; retained after unloading. */
    uint16_t occupied;
    uint8_t current, placed;
    bool uncertain;
} BallInventory;
typedef enum { BALL_ADDED, BALL_DUPLICATE, BALL_CONFLICT, BALL_INVALID, BALL_FULL } BallRecord;
uint8_t BallInventory_Decode(const uint8_t block[16]);
BallRecord BallInventory_Record(BallInventory *b, uint32_t uid, uint8_t code);
int BallInventory_Find(const BallInventory *b, uint8_t code);
void BallInventory_Step(BallInventory *b, bool reverse);
bool BallInventory_ReverseTo(uint8_t current, uint8_t target);
bool BallInventory_Unload(BallInventory *b, uint8_t code);
#endif
