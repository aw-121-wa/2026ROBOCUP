#ifndef PATH_WAREHOUSE_H
#define PATH_WAREHOUSE_H
#include "path_mission.h"

/* Values are retained for existing telemetry and capture tools. */
typedef enum {
    WAREHOUSE_MOVE = 0,
    WAREHOUSE_SELECT_BALL = 1,
    WAREHOUSE_TURN = 2,
    WAREHOUSE_UNLOAD = 3,
    WAREHOUSE_ALIGN_LINE = 4,
    WAREHOUSE_RETURN_HOME = 5,
    WAREHOUSE_ALIGN_HOME = 6,
    WAREHOUSE_CHECK_HEADING = 7,
    WAREHOUSE_FIRST_DIGIT = 8,
    WAREHOUSE_BRAKE = 9,
    WAREHOUSE_WAIT_CAMERA = 10,
    WAREHOUSE_FIRST_OFFSET = 11,
    WAREHOUSE_RETURN_LINE = 12,
    WAREHOUSE_CHECK_OFFSET_LINE = 13,
    WAREHOUSE_HOME_BRAKE = 14,
    WAREHOUSE_ENTRY_BACK = 15,
    WAREHOUSE_HOME_RIGHT = 16,
    WAREHOUSE_HOME_SEARCH = 17,
    WAREHOUSE_HOME_ADVANCE = 18,
    WAREHOUSE_INFERRED_MOVE = 19,
    WAREHOUSE_HOME_FINAL_ALIGN = 20,
    WAREHOUSE_DESTACK_UNLOAD_OFFSET = 21
} WarehousePhase;

typedef enum {
    WAREHOUSE_UNDECIDED = 0,
    WAREHOUSE_DEFAULT_ORDER = 1,
    WAREHOUSE_DIGIT_ORDER = 2
} WarehouseMode;
uint8_t PathWarehouse_Code(const PathMission *m);
void PathWarehouse_Tick(PathMission *m, uint32_t now, const PathInput *in);
#endif
