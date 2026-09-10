#include "chassis_control.h"
#include "motion.h"
#include "pin_config.h"
#include "bsp_dwt.h"
#include "jy60.h"
#include "zdt_x42s.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>
#define RAD 0.017453292519943295f
ChassisConfig chassis_config = {35, 150, 150,          100,          2,    0.05f, 0.15f, 1, 1, 1,
                                1,  1,   {2, 1, 3, 4}, {1, 1, 1, 1}, false};
static ChassisState state;
volatile ChassisDebugCommand chassis_debug;
static Planner planner;
static float dx, dy, heading, integral, bias_sum, bias_elapsed;
static float jog_x, jog_y, jog_w, jog_remaining;
static uint32_t last_cycle, last_gyro, last_angle, bias_count, telemetry_cycle;
static uint8_t tx[36] __attribute__((aligned(32)));
static float inflight[4], pending[4];
static volatile bool tx_done, tx_busy, tx_error;
static bool pending_valid;
static uint32_t tx_cycle;
static volatile uint32_t tx_complete_cycle;
static uint32_t odom_cycle;
static uint8_t telemetry[60];
static Geometry geometry(void)
{
    return (Geometry){chassis_config.wheel_radius_mm,
                      chassis_config.half_track_mm + chassis_config.half_wheelbase_mm};
}
static float clamp(float x, float m)
{
    return fmaxf(-m, fminf(m, x));
}
static void integrate_odom(uint32_t cycle)
{
    float dt = DWT_DeltaSec(cycle, odom_cycle);
    odom_cycle = cycle;
    if (dt > 0.05f || !isfinite(dt))
        return;
    Mecanum_Forward(geometry(), state.rpm_sent, state.velocity);
    float y = state.velocity[1] * (state.velocity[1] >= 0 ? chassis_config.left_odom_scale
                                                          : chassis_config.right_odom_scale);
    state.x_mm += (cosf(state.yaw_rad) * state.velocity[0] - sinf(state.yaw_rad) * y) * dt;
    state.y_mm += (sinf(state.yaw_rad) * state.velocity[0] + cosf(state.yaw_rad) * y) * dt;
}
static bool config_valid(void)
{
    ChassisConfig *c = &chassis_config;
    if (!isfinite(c->wheel_radius_mm) || !isfinite(c->half_track_mm) ||
        !isfinite(c->half_wheelbase_mm) || !isfinite(c->rpm_limit) || c->wheel_radius_mm <= 0 ||
        c->half_track_mm <= 0 || c->half_wheelbase_mm <= 0 || c->rpm_limit < 1 ||
        c->rpm_limit > 1000)
        return false;
    if (!isfinite(c->kp) || !isfinite(c->ki) || !isfinite(c->gyro_damping) ||
        !isfinite(c->wz_limit) || c->kp < 0 || c->ki < 0 || c->gyro_damping < 0 || c->wz_limit <= 0)
        return false;
    if (!isfinite(c->left_gain) || !isfinite(c->right_gain) || !isfinite(c->left_odom_scale) ||
        !isfinite(c->right_odom_scale) || c->left_gain <= 0 || c->right_gain <= 0 ||
        c->left_odom_scale <= 0 || c->right_odom_scale <= 0)
        return false;
    for (int i = 0; i < 4; i++)
    {
        if (c->motor_id[i] == 0 || c->motor_id[i] > 247 ||
            (c->motor_sign[i] != 1 && c->motor_sign[i] != -1))
            return false;
        for (int j = 0; j < i; j++)
            if (c->motor_id[i] == c->motor_id[j])
                return false;
    }
    return true;
}
uint32_t PinConfig_Validate(void)
{
    uint32_t e = 0;
    if (PINCFG_JY60_UART == PINCFG_ZDT_UART)
        e |= PINCFG_ERR_UART_CONFLICT;
    if (PINCFG_JY60_UART->Init.BaudRate != PINCFG_JY60_BAUDRATE)
        e |= PINCFG_ERR_JY60_BAUDRATE;
    if (PINCFG_ZDT_UART->Init.BaudRate != PINCFG_ZDT_BAUDRATE)
        e |= PINCFG_ERR_ZDT_BAUDRATE;
    if (!PINCFG_JY60_UART->hdmarx)
        e |= PINCFG_ERR_JY60_DMA_MISSING;
    else if (PINCFG_JY60_UART->hdmarx->Init.Mode != DMA_CIRCULAR)
        e |= PINCFG_ERR_JY60_DMA_NOT_CIRC;
    return e;
}
/* F6 speed frames, followed by broadcast synchronous execution (fixed 0x6B). */
static void service_tx(void)
{
    if (tx_error)
    {
        state.fault |= 4;
        Chassis_Stop();
        return;
    }
    if (tx_done)
    {
        integrate_odom(tx_complete_cycle);
        tx_done = false;
        tx_busy = false;
        memcpy(state.rpm_sent, inflight, sizeof(inflight));
    }
    if (tx_busy)
    {
        if (DWT_DeltaSec(DWT_GetCycle(), tx_cycle) > 0.05f)
        {
            state.fault |= 4;
            Chassis_Stop();
        }
        return;
    }
    if (!pending_valid)
        return;
    for (int i = 0; i < 4; i++)
    {
        int16_t speed = (int16_t)((int)pending[i] * chassis_config.motor_sign[i]);
        ZDT_BuildSpeed(&tx[i * 8], chassis_config.motor_id[i], speed, 0);
    }
    ZDT_BuildSync(&tx[32]);
    memcpy(inflight, pending, sizeof(inflight));
    tx_done = false;
    tx_busy = true;
    tx_cycle = DWT_GetCycle();
    if (HAL_UART_Transmit_DMA(PINCFG_ZDT_UART, tx, sizeof(tx)) == HAL_OK)
        pending_valid = false;
    else
    {
        tx_busy = false;
        state.fault |= 4;
        Chassis_Stop();
    }
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uart)
{
    if (uart == PINCFG_ZDT_UART)
    {
        tx_complete_cycle = DWT_GetCycle();
        tx_done = true;
    }
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    if (uart == PINCFG_ZDT_UART)
        tx_error = true;
}
bool Chassis_Init(void)
{
    if (!DWT_Time_Init())
        return false;
    /* Configure once before RX starts; preserve CubeMX-owned initialization. */
    DMA_HandleTypeDef *dma = PINCFG_JY60_UART->hdmarx;
    if (!dma || !PINCFG_ZDT_UART->hdmatx)
        return false;
    dma->Init.Mode = DMA_CIRCULAR;
    if (HAL_DMA_Init(dma) != HAL_OK || PinConfig_Validate() != 0)
        return false;
    /* Existing firmware leaves D-cache disabled. DMA buffers require that policy. */
    if ((SCB->CCR & SCB_CCR_DC_Msk) != 0)
        return false;
    if (!JY60_Init())
        return false;
    last_cycle = DWT_GetCycle();
    telemetry_cycle = last_cycle;
    odom_cycle = last_cycle;
    return true;
}
bool Chassis_Arm(void)
{
    if (!chassis_config.calibrated || !config_valid() || !state.bias_ready || state.fault ||
        JY60_GetState()->trust != JY60_TRUST_GOOD)
        return false;
    state.armed = true;
    heading = state.yaw_rad;
    integral = 0;
    return true;
}
void Chassis_Stop(void)
{
    state.armed = false;
    planner.active = false;
    jog_remaining = 0;
    integral = 0;
    memset(pending, 0, sizeof(pending));
    pending_valid = true;
}
bool Chassis_Move(float x, float y, float v, float a, float d)
{
    if (!state.armed || !isfinite(x) || !isfinite(y))
        return false;
    float length = hypotf(x, y);
    if (!Planner_Start(&planner, length, v, a, d))
        return false;
    jog_remaining = 0;
    dx = x / length;
    dy = y / length;
    heading = state.yaw_rad;
    return true;
}
const ChassisState *Chassis_GetState(void)
{
    return &state;
}
static void send_telemetry(float vx, float vy, float wz, uint32_t now)
{
    if (DWT_DeltaSec(now, telemetry_cycle) < 0.05f ||
        PINCFG_VOFA_UART->gState != HAL_UART_STATE_READY)
        return;
    telemetry_cycle = now;
    float channels[14] = {vx,
                          vy,
                          wz,
                          state.velocity[0],
                          state.velocity[1],
                          state.velocity[2],
                          state.yaw_rad / RAD,
                          state.yaw_error / RAD,
                          state.rpm_sent[0],
                          state.rpm_sent[1],
                          state.rpm_sent[2],
                          state.rpm_sent[3],
                          (float)JY60_GetState()->trust,
                          state.dt};
    memcpy(telemetry, channels, sizeof(channels));
    telemetry[56] = 0;
    telemetry[57] = 0;
    telemetry[58] = 0x80;
    telemetry[59] = 0x7f;
    (void)HAL_UART_Transmit_IT(PINCFG_VOFA_UART, telemetry, sizeof(telemetry));
}
void Chassis_Update(void)
{
    uint32_t now = DWT_GetCycle();
    float dt = DWT_DeltaSec(now, last_cycle);
    service_tx();
    if (dt < 0.005f)
        return;
    last_cycle = now;
    state.dt = dt;
    state.updates++;
    JY60_Process();
    const JY60_State_t *imu = JY60_GetState();
    if (dt > 0.05f)
    {
        state.fault |= 1;
        Chassis_Stop();
        dt = 0;
    }
    if (!config_valid())
    {
        state.fault |= 8;
        Chassis_Stop();
        return;
    }
    if (imu->trust == JY60_TRUST_LOST)
    {
        if (state.armed)
            state.fault |= 2;
        Chassis_Stop();
    }
    if (!state.bias_ready && imu->trust != JY60_TRUST_GOOD)
    {
        bias_elapsed = 0;
        bias_sum = 0;
        bias_count = 0;
    }
    if (!state.bias_ready && imu->trust == JY60_TRUST_GOOD)
    {
        if (imu->gyro_frame_count != last_gyro)
        {
            if (fabsf(imu->gz_dps) < 3)
            {
                bias_sum += imu->gz_dps;
                bias_count++;
            }
            else
            {
                bias_sum = 0;
                bias_count = 0;
                bias_elapsed = 0;
            }
        }
        bias_elapsed += dt;
        if (bias_elapsed >= 2 && bias_count >= 20)
        {
            state.gyro_bias_dps = bias_sum / (float)bias_count;
            state.bias_ready = true;
        }
    }
    if (imu->trust != JY60_TRUST_LOST)
    {
        state.yaw_rad = Angle_Wrap(state.yaw_rad + (imu->gz_dps - state.gyro_bias_dps) * RAD * dt);
        if (imu->angle_frame_count != last_angle)
        {
            float error = Angle_Wrap(imu->yaw_deg * RAD - state.yaw_rad);
            state.yaw_rad = Angle_Wrap(state.yaw_rad + (last_angle ? 0.2f : 1) * error);
        }
    }
    last_gyro = imu->gyro_frame_count;
    last_angle = imu->angle_frame_count;
    if (!tx_busy)
        integrate_odom(DWT_GetCycle());
    uint32_t sequence = chassis_debug.sequence;
    if (sequence != chassis_debug.acknowledged)
    {
        uint32_t command = chassis_debug.command;
        bool ok = false;
        if (command == 1)
        {
            Chassis_Stop();
            ok = true;
        }
        if (command == 2)
            ok = Chassis_Arm();
        if (command == 3)
            ok = Chassis_Move(chassis_debug.x, chassis_debug.y, chassis_debug.vmax,
                              chassis_debug.amax, chassis_debug.dmax);
        /* Timed low-speed jog: x/y mm/s, vmax rad/s, amax duration seconds. */
        if (command == 4 && state.armed && isfinite(chassis_debug.x) && isfinite(chassis_debug.y) &&
            isfinite(chassis_debug.vmax) && isfinite(chassis_debug.amax) &&
            chassis_debug.amax > 0 && chassis_debug.amax <= 2)
        {
            jog_x = clamp(chassis_debug.x, 100);
            jog_y = clamp(chassis_debug.y, 100);
            jog_w = clamp(chassis_debug.vmax, 0.3f);
            jog_remaining = chassis_debug.amax;
            planner.active = false;
            ok = true;
        }
        chassis_debug.result = ok ? 0 : -1;
        chassis_debug.acknowledged = sequence;
    }
    float speed = state.armed ? Planner_Update(&planner, dt) : 0;
    float vx = speed * dx, vy = speed * dy, wz = 0;
    state.yaw_error = Angle_Wrap(heading - state.yaw_rad);
    if (state.armed)
    {
        float error = fabsf(state.yaw_error) < 0.005f ? 0 : state.yaw_error;
        integral = clamp(integral + error * dt, 0.5f);
        wz = clamp(chassis_config.kp * error + chassis_config.ki * integral -
                       chassis_config.gyro_damping * (imu->gz_dps - state.gyro_bias_dps) * RAD,
                   chassis_config.wz_limit);
    }
    if (state.armed && jog_remaining > 0)
    {
        vx = jog_x;
        vy = jog_y;
        wz = jog_w;
        jog_remaining = fmaxf(0, jog_remaining - dt);
        heading = state.yaw_rad;
        integral = 0;
    }
    vy *= vy >= 0 ? chassis_config.left_gain : chassis_config.right_gain;
    float t[4], r[4], out[4];
    Mecanum_Inverse(geometry(), vx, vy, 0, t);
    Mecanum_Inverse(geometry(), 0, 0, wz, r);
    Wheel_Limit(t, r, chassis_config.rpm_limit, out);
    for (int i = 0; i < 4; i++)
    {
        pending[i] = roundf(out[i]);
    }
    pending_valid = true;
    service_tx();
    send_telemetry(vx, vy, wz, now);
}
