#ifndef HOST_COMMAND_H
#define HOST_COMMAND_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum
{
    HOST_NONE,
    HOST_ARM,
    HOST_FORWARD,
    HOST_SHIFT,
    HOST_STOP,
    HOST_PATH
} HostCommandKind;
typedef struct
{
    HostCommandKind kind;
    float distance_mm;
} HostCommand;
typedef struct
{
    char line[64];
    size_t length;
    bool discard;
} HostParser;
enum
{
    HOST_IDLE = 0,
    HOST_OK = 1,
    HOST_SYNTAX = -1,
    HOST_NOT_READY = -2,
    HOST_BUSY = -3,
    HOST_RANGE = -4,
    HOST_RX_ERROR = -5
};
/* 0: no complete line; 1: command; negative: rejected line. */
int HostCommand_Feed(HostParser *parser, uint8_t byte, HostCommand *command);
int HostCommand_Check(const HostCommand *command, bool armed, bool busy);
#endif
