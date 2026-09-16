#ifndef PATH_MISSION_H
#define PATH_MISSION_H
#include <stdbool.h>
#include <stdint.h>
typedef enum
{
    PATH_IDLE,
    PATH_RUNNING,
    PATH_DONE,
    PATH_CANCELED,
    PATH_TIMEOUT,
    PATH_ERROR
} PathResult;
typedef enum
{
    PATH_WAIT,
    PATH_OK,
    PATH_NONE,
    PATH_FAILED
} PathReply;
typedef enum
{
    PC_MOVE,
    PC_ROTATE,
    PC_BODY,
    PC_HOLD,
    PC_GROUP,
    PC_VISION,
    PC_DISC,
    PC_TURN,
    PC_CANCEL,
    PC_HELLO
} PathCommandKind;
typedef struct
{
    PathCommandKind kind;
    float x, y, speed;
    uint32_t argument, timeout_ms;
} PathCommand;
typedef struct
{
    bool armed, fault, settled;
    float yaw_deg;
    uint8_t gray; /* active-low converted bits MID2 IN2 IN1 MID1: target 0b0110 */
    bool ir;
    uint16_t rfid; /* IDs seen since preceding tick, bit N is raw ID N */
    PathReply reply, turn_reply, interrupted_reply;
} PathInput;
typedef bool (*PathSend)(void *context, const PathCommand *command);
typedef struct
{
    PathResult result;
    unsigned step, phase, part, point, grabs;
    uint32_t entered, started, stable_since, orbit_ms, previous;
    uint16_t ids, candidate;
    uint32_t id_list[64]; /* Full UID, wire byte order represented as big-endian integer. */
    uint8_t id_count;
    bool id_overflow;
    float orbit_yaw;
    bool waiting, stable, expired;
    PathSend send;
    void *context;
} PathMission;
void Path_Init(PathMission *mission, PathSend send, void *context);
bool Path_Start(PathMission *mission, uint32_t now, const PathInput *input);
void Path_Tick(PathMission *mission, uint32_t now, const PathInput *input);
void Path_Cancel(PathMission *mission);
void Path_RecordId(PathMission *mission, uint32_t id);
#endif
