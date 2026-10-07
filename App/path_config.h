#ifndef PATH_CONFIG_H
#define PATH_CONFIG_H
#include "heading_tuning.h"
/* Full blue material flow; set to 0 only for collection-without-storage tests. */
#ifndef PATH_BLUE_MATERIAL_ENABLE
#define PATH_BLUE_MATERIAL_ENABLE 1
#endif
#define PATH_SKIP_MATERIAL(m) ((m)->blue && !PATH_BLUE_MATERIAL_ENABLE)
#ifndef PATH_BLUE_WAREHOUSE_TEST
#define PATH_BLUE_WAREHOUSE_TEST 0
#endif
#ifndef PATH_BLUE_STAIR_TEST
#define PATH_BLUE_STAIR_TEST 0
#endif
#ifndef PATH_BLUE_PILLAR_TEST
#define PATH_BLUE_PILLAR_TEST 0
#endif
#ifndef PATH_BLUE_DISC_TEST
#define PATH_BLUE_DISC_TEST 0
#endif
/* Warehouse creep uses rpm; distances are millimetres, acceleration mm/s^2. */
#define PATH_WAREHOUSE_TARGET_DEG(blue) 0.0f
#define PATH_WAREHOUSE_ENTRY_BACK_MM 10.0f
#define PATH_WAREHOUSE_FIRST_RIGHT_MM 15.0f
#define PATH_WAREHOUSE_CREEP_LIMIT_MM 200.0f
#define PATH_WAREHOUSE_CREEP_SPEED_RPM 16.0f
#define PATH_WAREHOUSE_DIGIT_ADVANCE_MM 25.0f
#define PATH_WAREHOUSE_CREEP_ACCEL 250.0f
#define PATH_WAREHOUSE_DIGIT_TIMEOUT_MS 10000U
#define PATH_WAREHOUSE_DIGIT_GUARD_MS 11000U
#define PATH_WAREHOUSE_CREEP_TIMEOUT_MS 12000U
#define PATH_STAIR_ENTRY_ADVANCE_MM 5.0f
#define PATH_STAIR_SCAN_SPEED_RPM 40.0f
#define PATH_STAIR_FAST_SPEED_RPM 120.0f
#define PATH_STAIR_FAST_ACCEL 850.0f

/* Line acquisition and admission; lateral speeds are mm/s, approach speed rpm. */
#define PATH_STAIR_SEARCH_FAST_DISTANCE_MM 1080.0f
#define PATH_STAIR_SEARCH_FAST_RPM 90.0f
#define PATH_STAIR_SEARCH_SLOW_RPM 30.0f
#define PATH_STAIR_LINE_MM_S 40.0f
#define PATH_WAREHOUSE_LINE_MM_S 15.0f
#define PATH_LINE_STABLE_MS 100U
#define PATH_LINE_SWEEP_MS 2000U
#define PATH_LINE_ENTRY_SWEEP_MS 1000U
#define PATH_LINE_ENTRY_REVERSE_MS 5000U
#define PATH_WAREHOUSE_REVERSE_MS 4000U
#define PATH_LINE_SEARCH_TIMEOUT_MS 3000U
#define PATH_WAREHOUSE_SEARCH_TIMEOUT_MS 12000U
#define PATH_HEADING_TIMEOUT_MS 30000U
#define PATH_WAREHOUSE_HEADING_TOLERANCE_DEG HEADING_STATIC_TOLERANCE_DEG
#define PATH_MOVE_ACCEL_MM_S2 650.0f
#define PATH_MOVE_DECEL_MM_S2 650.0f

/* Shared outer/inner deadline for multi-stage disc line alignment. */
#define PATH_DISC_LINE_TIMEOUT_MS 30000U
#ifndef PATH_STOP_AT_WAREHOUSE_LINE
#define PATH_STOP_AT_WAREHOUSE_LINE 0
#endif
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
#if PATH_BLUE_DISC_TEST && PATH_VISION_ENABLE
#error "Blue disc test requires vision disabled"
#endif
#if PATH_BLUE_PILLAR_TEST && (!PATH_BLUE_DISC_TEST || PATH_VISION_ENABLE)
#error "Blue pillar test requires blue mirrored chassis-only route"
#endif
#if PATH_BLUE_STAIR_TEST && !PATH_BLUE_PILLAR_TEST
#error "Blue stair test requires blue pillar test"
#endif
#if PATH_BLUE_WAREHOUSE_TEST && !PATH_BLUE_STAIR_TEST
#error "Warehouse test requires blue stair route"
#endif
#define PATH_RDK_ENABLE (PATH_VISION_ENABLE || PATH_BLUE_WAREHOUSE_TEST)
#endif
