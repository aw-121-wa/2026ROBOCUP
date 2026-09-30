#ifndef PATH_MISSION_H
#define PATH_MISSION_H
#include <stdbool.h>
#include <stdint.h>
#include "ball_inventory.h"
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
    PC_HELLO,
    PC_PILLAR_STOPPED,
    PC_PILLAR_END,
    PC_STAIR,
    PC_ARC,
    PC_ALIGN_ZERO,
    PC_MOVE_ROTATE,
    PC_LINE_REFERENCE,
    PC_LINE_SEARCH,
    PC_LINE_CALIBRATE,
    PC_RETURN_HOME,
    PC_MAP_AXIS,
    PC_MAP_LATERAL,
    PC_MAP_HEADING,
    PC_HOME_ALIGN,
    PC_MAP_SEARCH,
    PC_STAIR_SCAN
} PathCommandKind;
typedef struct
{
    PathCommandKind kind;
    float x, y, angle, speed;
    float start_speed, end_speed;
    bool continuous;
    uint32_t argument, timeout_ms;
} PathCommand;
typedef struct
{
    bool armed, fault, settled, motion_done;
    float yaw_deg;
    float x_mm, y_mm; /* Existing wheel-command odometry, in world frame. */
    float map_yaw_deg; /* Measured yaw relative to the fixed PATH start direction. */
    float imu_yaw_deg; /* Latest validated JY60 angle, without software zero/integration. */
    uint8_t gray; /* active-low: bit3 PD3, bit2 PD0, bit1 PD1, bit0 PB13; stair/warehouse target 0110 */
    bool ir;
    uint16_t rfid; /* IDs seen since preceding tick, bit N is raw ID N */
    PathReply reply, turn_reply, interrupted_reply;
    bool vision_ready;
    uint8_t ball_index, resume_index;
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
    BallInventory inventory;
    bool id_overflow;
    float orbit_yaw;
    float stair_origin_x, stair_origin_y, stair_axis, stair_distance;
    uint32_t stair_started;
    unsigned stair_base_grabs;
    bool stair_scanning;
    float line_scan_yaw;
    uint32_t line_since, line_shift_since;
    unsigned line_scan_stage, line_shift_count;
    unsigned line_best_count;
    unsigned line_recovery, line_retries, line_reversals;
    float line_scan_side;
    uint32_t line_loss_since;
    bool line_losing;
    bool line_active, line_stopping, line_skipped;
    bool stair_heading_locked, warehouse_heading_locked;
    bool waiting, stable, expired;
    bool heading_align_active;
    uint32_t heading_align_since;
    PathSend send;
    void *context;
} PathMission;
void Path_Init(PathMission *mission, PathSend send, void *context);
bool Path_Start(PathMission *mission, uint32_t now, const PathInput *input);
void Path_Tick(PathMission *mission, uint32_t now, const PathInput *input);
void Path_Cancel(PathMission *mission);
void Path_RecordId(PathMission *mission, uint32_t id);
#endif
