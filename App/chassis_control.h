#ifndef CHASSIS_CONTROL_H
#define CHASSIS_CONTROL_H
#include <stdbool.h>
#include <stdint.h>
#include "zdt_x42s.h"
#include "heading.h"
#include "chassis_policy.h"
typedef struct
{
    float wheel_radius_mm, half_track_mm, half_wheelbase_mm;
    float rpm_limit, kp, ki, gyro_damping, wz_limit;
    float left_gain, right_gain, forward_lateral_comp, left_odom_scale, right_odom_scale;
    uint8_t motor_id[4];
    int8_t motor_sign[4];
    bool calibrated;
    ZDT_CommandMode command_mode; /* Change only while disarmed and transport idle. */
} ChassisConfig;
typedef struct
{
    float x_mm, y_mm, yaw_rad, gyro_bias_dps, dt;
    float velocity[3], rpm_applied[4], yaw_error;
    float rpm_requested[4], rpm_pending[4], rpm_inflight[4];
    uint32_t deadline_misses;
    float applied_path_speed, committed_path_speed;
    uint32_t fault, updates;
    bool armed, bias_ready;
} ChassisState;
typedef struct {
    HeadingMode mode;
    float requested_rad_s, quantized_rad_s, measured_rad_s;
    float tolerance_deg;
    bool within_tolerance; /* Tracking metric, not permission to stop correcting. */
} ChassisHeadingDiagnostics;
extern volatile ChassisHeadingDiagnostics chassis_heading_diagnostics;
extern ChassisConfig chassis_config;
/* Debugger mailbox: fill arguments first, then increment sequence. Task-owned APIs. */
typedef struct
{
    uint32_t sequence, command;
    float x, y, vmax, amax, dmax;
    uint32_t acknowledged;
    int32_t result;
} ChassisDebugCommand;
extern volatile ChassisDebugCommand chassis_debug;
bool Chassis_Init(void);
void Chassis_Update(void);
void Chassis_RecordDeadlineMiss(void);
bool Chassis_Arm(void);
void Chassis_Stop(void);
bool Chassis_Move(float x_mm, float y_mm, float vmax, float amax, float dmax);
/* Translation is fixed in the starting body frame; yaw advances with path progress. */
bool Chassis_MoveRotate(float x_mm, float y_mm, float degrees, float vmax, float amax, float dmax);
/* Nonzero end speed uses smooth yaw and hands off without stopping for yaw settling.
 * Following boundary/arc moves retain the final heading frame until Hold/Stop. */
bool Chassis_MoveRotateBoundary(float x_mm, float y_mm, float degrees, float vmax,
                                 float amax, float dmax, float start_speed, float end_speed);
bool Chassis_MoveBoundary(float x_mm, float y_mm, float vmax, float amax, float dmax,
                          float start_speed, float end_speed);
bool Chassis_MoveArc(float radius_mm, float start_angle_deg, float turn_degrees,
                     float vmax, float amax, float dmax, float start_speed, float end_speed);
/* Sole chassis task owner; PATH uses these without invoking another motor stack. */
/* Called once after an accepted PATH/DISC start; manual moves keep local headings. */
void Chassis_BeginPath(void);
void Chassis_SetRoutePolicy(ChassisRoutePolicy policy);
bool Chassis_ReturnHome(unsigned leg); /* 0: back, 1: right transit, 2: right line approach */
bool Chassis_ExitOrbitArc(float radius, float target_deg, float speed, float acceleration, float deceleration);
bool Chassis_ExitOrbit(float forward_mm, float lateral_mm, float target_deg, float speed, float end_speed, float acceleration, float deceleration);
bool Chassis_FinishForward(float distance_mm, float speed, float acceleration, float deceleration);
bool Chassis_AlignHome(float target_deg);
bool Chassis_AlignMapAxis(void);
bool Chassis_SetMapHeading(float degrees);
bool Chassis_MapLateral(float mm);
bool Chassis_MapSearch(float mm_s);
/* Normal stop ramps to zero; fault/cancel paths bypass the ramp. */
void Chassis_Hold(void);
void Chassis_HoldCapture(void); /* Faster smooth braking; still requires IsSettled. */
void Chassis_HoldImmediate(void);
bool Chassis_Rotate(float degrees);
bool Chassis_AlignZero(void);
bool Chassis_Body(float forward_mm_s, float left_mm_s, float radians_s);
bool Chassis_MotionBusy(void);
bool Chassis_IsSettled(void);
float Chassis_ContinuousYaw(void);
float Chassis_MapYaw(void);
float Chassis_MeasuredYaw(void);
float Chassis_LineYaw(void);
bool Chassis_SetLineReference(void);
bool Chassis_LineSearch(float lateral_mm_s, float radians_s);
bool Chassis_CalibrateLine(void);
const ChassisState *Chassis_GetState(void);
#endif
