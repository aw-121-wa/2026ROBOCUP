#ifndef CHASSIS_CONTROL_H
#define CHASSIS_CONTROL_H
#include <stdbool.h>
#include <stdint.h>
typedef struct
{
    float wheel_radius_mm, half_track_mm, half_wheelbase_mm;
    float rpm_limit, kp, ki, gyro_damping, wz_limit;
    float left_gain, right_gain, left_odom_scale, right_odom_scale;
    uint8_t motor_id[4];
    int8_t motor_sign[4];
    bool calibrated;
} ChassisConfig;
typedef struct
{
    float x_mm, y_mm, yaw_rad, gyro_bias_dps, dt;
    float velocity[3], rpm_sent[4], yaw_error;
    uint32_t fault, updates;
    bool armed, bias_ready;
} ChassisState;
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
bool Chassis_Arm(void);
void Chassis_Stop(void);
bool Chassis_Move(float x_mm, float y_mm, float vmax, float amax, float dmax);
const ChassisState *Chassis_GetState(void);
#endif
