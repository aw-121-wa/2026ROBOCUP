#ifndef PATH_CONFIG_H
#define PATH_CONFIG_H
/* Warehouse creep uses rpm; distances are millimetres, acceleration mm/s^2. */
#define PATH_WAREHOUSE_FIRST_RIGHT_MM 15.0f
#define PATH_WAREHOUSE_CREEP_LIMIT_MM 200.0f
#define PATH_WAREHOUSE_CREEP_SPEED_RPM 10.0f
#define PATH_WAREHOUSE_CREEP_ACCEL 150.0f
#define PATH_WAREHOUSE_DIGIT_TIMEOUT_MS 10000U
#define PATH_WAREHOUSE_DIGIT_GUARD_MS 11000U
#define PATH_WAREHOUSE_CREEP_TIMEOUT_MS 12000U
#define PATH_STAIR_ENTRY_ADVANCE_MM 5.0f
#define PATH_STAIR_FAST_ACCEL 850.0f

/* Shared outer/inner deadline for multi-stage disc line alignment. */
#define PATH_DISC_LINE_TIMEOUT_MS 30000U
#ifndef PATH_STOP_AT_STAIR_LINE
#define PATH_STOP_AT_STAIR_LINE 0
#endif
/* Full mission enabled; host diagnostics may explicitly override to 0. */
#ifndef PATH_VISION_ENABLE
#define PATH_VISION_ENABLE 1
#endif
#if PATH_VISION_ENABLE != 0 && PATH_VISION_ENABLE != 1
#error "PATH_VISION_ENABLE must be 0 or 1"
#endif
#endif
