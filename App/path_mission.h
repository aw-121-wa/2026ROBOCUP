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
    PC_STAIR_SCAN,
    PC_WAREHOUSE_DIGIT,
    PC_FINISH_FORWARD, /* Retarget current straight motion without stopping. */
    PC_ORBIT_EXIT, /* Transfer moving orbit to map-heading translation. */
    PC_ORBIT_ARC, /* Enter stair arc directly from moving orbit. */
} PathCommandKind;
typedef struct
{
    PathCommandKind kind;
    /* MOVE: x/y mm, angle degrees, speed/start/end rpm.
     * BODY: x/y wheel-equivalent rpm; speed rotation-equivalent rpm.
     * MAP_SEARCH: y mm/s. MAP_HEADING/HOME_ALIGN/MAP_AXIS: x degrees.
     * Port adapter is the only RPM-to-SI conversion boundary. */
    float x, y, angle, speed;
    float start_speed, end_speed;
    float acceleration, deceleration; /* mm/s^2; zero selects the standard profile. */
    bool continuous;
    uint32_t argument, timeout_ms;
} PathCommand;
typedef struct
{
    bool armed, fault, settled, motion_done;
    float yaw_deg;
    float x_mm, y_mm; /* Command-integrated open-loop odometry, in world frame. */
    float map_yaw_deg; /* Measured yaw relative to the fixed PATH start direction. */
    float imu_yaw_deg; /* Latest validated JY60 angle, without software zero/integration. */
    uint8_t gray; /* Active bits: PD3/PD0/PD1/PB13. Acceptance is station-specific. */
    bool ir;
    uint16_t rfid; /* IDs seen since preceding tick, bit N is raw ID N */
    PathReply reply, turn_reply, interrupted_reply;
    bool warehouse_vision;
    bool warehouse_ready;
    uint8_t warehouse_digit;
    PathReply warehouse_digit_reply;
    bool vision_ready;
    uint8_t ball_index, resume_index;
    uint8_t disc_completed; /* Actual action-complete events, independent of RFID. */
} PathInput;
typedef bool (*PathSend)(void *context, const PathCommand *command);
typedef enum {
    LINE_SEARCH_IDLE, LINE_SWEEP_FIRST,
    LINE_FALLBACK_HEADING, LINE_BRAKE_REVERSE, LINE_SWEEP_REVERSE,
    LINE_BRAKE_FINAL, LINE_SWEEP_FINAL
} PathLineSearchState;
typedef struct
{
    bool blue; /* Runtime side; fixed for an entire mission. */
    PathResult result;
    unsigned step, phase, part, point, grabs;
    uint32_t entered, started, stable_since, orbit_ms, previous;
    uint16_t ids, candidate;
    uint32_t id_list[64]; /* Full UID, wire byte order represented as big-endian integer. */
    uint8_t id_count;
    BallInventory inventory;
    bool warehouse_plan_ready;
    uint8_t warehouse_order[9];
    uint8_t warehouse_mode; /* WarehouseMode values; see path_warehouse.h. */
    uint8_t warehouse_columns[3], warehouse_used;
    bool warehouse_query;
    bool warehouse_ignore_line; /* Latched only after the first acknowledged unload. */
    bool stair_prep_started, stair_ready_started;
    bool id_overflow;
    float orbit_yaw;
    float stair_origin_x, stair_origin_y, stair_axis, stair_distance;
    uint32_t stair_started;
    unsigned stair_base_grabs;
    bool stair_scanning;
    bool approach_started, approach_slow;
    float approach_x, approach_y;
    float home_line_y; /* Map-Y projection at first home-line detection. */
    uint32_t line_since;
    bool line_entry_detected;
    PathLineSearchState line_search_state;
    bool line_active, line_stopping, line_skipped;
    bool stair_heading_locked, warehouse_heading_locked;
    bool waiting, stable, expired;
    bool prep_pending;
    bool disc_depart_pending;
    bool pillar_depart_pending;
    uint32_t prep_since;
    bool heading_align_active;
    bool stair_heading_calibrated, warehouse_heading_calibrated;
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
