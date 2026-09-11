#include "chassis_control.h"
#include "host_command.h"
#include "host_uart.h"
#include "motion.h"
#include "heading.h"
#include "relative_yaw.h"
#include "chassis_odom.h"
#include "pin_config.h"
#include "bsp_dwt.h"
#include "jy60.h"
#include "zdt_x42s.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>
#define RAD 0.017453292519943295f
ChassisConfig chassis_config = {.wheel_radius_mm = 35.0f,
                                .half_track_mm = 128.5f, /* Measure before arming. */
                                .half_wheelbase_mm = 130.5f,
                                .rpm_limit = 200.0f,
                                .kp = 1.8f,
                                .ki = 0.00f,
                                .gyro_damping = 0.0f,
                                .wz_limit = 1.9f,
                                .left_gain = 1.0f,
                                .right_gain = 1.0f,
                                .left_odom_scale = 1.0f,
                                .right_odom_scale = 1.0f,
                                .motor_id = {2, 1, 3, 4},
                                .motor_sign = {1, -1, 1, -1}, /* FL/FR/RL/RR: IDs 2/1/3/4. */
                                .calibrated = true,
                                .command_mode = ZDT_MULTI_COMMAND};
static ChassisState state;
volatile ChassisDebugCommand chassis_debug;
static Planner planner;
static RelativeYaw relative_yaw;
static float dx, dy, heading, integral, bias_sum, bias_elapsed;
static float segment_progress;
static float jog_x, jog_y, jog_w, jog_remaining;
static uint32_t last_cycle, last_gyro, bias_count, telemetry_cycle;
static uint8_t tx[ZDT_MULTI_SPEED_SIZE] __attribute__((aligned(32)));
static volatile bool tx_done, tx_busy, tx_error;
static bool pending_valid;
static uint32_t tx_cycle;
static volatile uint32_t tx_complete_cycle;
static uint32_t odom_cycle;
static uint8_t telemetry[104]; /* 25 float channels + JustFloat tail. */
static HostParser host_parser;
static int host_result;
static uint32_t host_sequence, host_byte_cycle;
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
    ChassisOdomDelta delta = ChassisOdom_Integrate(
        geometry(), state.rpm_applied, state.yaw_rad, chassis_config.left_odom_scale,
        chassis_config.right_odom_scale, dx, dy, dt, state.velocity);
    if (planner.active)
        segment_progress += delta.progress;
    state.x_mm += delta.x;
    state.y_mm += delta.y;
}
static bool config_valid(void)
{
    ChassisConfig *c = &chassis_config;
    if (c->command_mode != ZDT_MULTI_COMMAND && c->command_mode != ZDT_LEGACY_SYNC)
        return false;
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
/* Wire buffer, inflight RPM and mode remain immutable until the batch completes. */
static uint8_t tx_part;
static ZDT_CommandMode tx_mode;
static size_t tx_length;
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
        tx_done = false;
        tx_busy = false;
        tx_cycle = tx_complete_cycle;
        if (tx_mode == ZDT_MULTI_COMMAND || tx_part == 4)
        {
            integrate_odom(tx_complete_cycle);
            memcpy(state.rpm_applied, state.rpm_inflight, sizeof(state.rpm_applied));
            tx_part = 0;
        }
        else
            tx_part++;
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
    if (tx_part != 0 && DWT_DeltaSec(DWT_GetCycle(), tx_cycle) < 0.003f)
        return;
    if (tx_part == 0)
    {
        if (!pending_valid)
            return;
        int16_t physical_rpm[4];
        for (int i = 0; i < 4; ++i)
            physical_rpm[i] = (int16_t)((int)state.rpm_pending[i] * chassis_config.motor_sign[i]);
        tx_mode = chassis_config.command_mode;
        if (tx_mode == ZDT_MULTI_COMMAND)
        {
            tx_length =
                ZDT_BuildMultiSpeed(tx, sizeof(tx), chassis_config.motor_id, physical_rpm, 0);
            if (tx_length == 0)
            {
                state.fault |= 4;
                Chassis_Stop();
                return;
            }
        }
        else if (tx_mode == ZDT_LEGACY_SYNC)
        {
            for (int i = 0; i < 4; ++i)
                ZDT_BuildLegacySpeed(&tx[i * 8], chassis_config.motor_id[i], physical_rpm[i], 0);
            ZDT_BuildSync(&tx[32]);
        }
        else
        {
            state.fault |= 4;
            Chassis_Stop();
            return;
        }
        memcpy(state.rpm_inflight, state.rpm_pending, sizeof(state.rpm_inflight));
        pending_valid = false;
    }
    tx_busy = true;
    tx_cycle = DWT_GetCycle();
    uint8_t *data = tx_mode == ZDT_MULTI_COMMAND ? tx : &tx[tx_part * 8];
    uint16_t length = tx_mode == ZDT_MULTI_COMMAND ? (uint16_t)tx_length : (tx_part == 4 ? 4 : 8);
    if (HAL_UART_Transmit_DMA(PINCFG_ZDT_UART, data, length) != HAL_OK)
    {
        tx_busy = false;
        state.fault |= 4;
        tx_error = true;
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
    HostUart_Error(uart);
    if (uart == PINCFG_ZDT_UART)
        tx_error = true;
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
    HostUart_RxComplete(uart);
}
bool Chassis_Init(void)
{
    /* 1. 初始化统一高精度时间基准 */
    if (!DWT_Time_Init())
    {
        return false;
    }

    /* 2. 检查 CubeMX / pin_config 是否配置正确 */
    if (PinConfig_Validate() != PINCFG_OK)
    {
        return false;
    }

    /* 3. 启动 JY60 Circular DMA 并初始化解析器 */
    if (!JY60_Init())
    {
        return false;
    }

    /* 4. 初始化底盘时间戳 */
    uint32_t now = DWT_GetCycle();

    last_cycle = now;
    telemetry_cycle = now;
    odom_cycle = now;

    if (!HostUart_Init())
        return false;

    return true;
}
bool Chassis_Arm(void)
{
    if (!chassis_config.calibrated || !config_valid() || !state.bias_ready || !relative_yaw.ready || state.fault ||
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
    memset(state.rpm_requested, 0, sizeof(state.rpm_requested));
    memset(state.rpm_pending, 0, sizeof(state.rpm_pending));
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
    segment_progress = 0;
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
    const JY60_State_t *imu = JY60_GetState();
    float channels[25] = {vx,
                          vy,
                          wz,
                          state.velocity[0],
                          state.velocity[1],
                          state.velocity[2],
                          state.yaw_rad / RAD,
                          state.yaw_error / RAD,
                          state.rpm_applied[0],
                          state.rpm_applied[1],
                          state.rpm_applied[2],
                          state.rpm_applied[3],
                          (float)JY60_GetState()->trust,
                          state.dt,
                          state.armed ? 1.0f : 0.0f,
                          (float)state.fault,
                          (float)host_result,
                          (float)host_sequence,
                          (planner.active || jog_remaining > 0) ? 1.0f : 0.0f,
                          state.bias_ready ? 1.0f : 0.0f,
                          imu->raw_yaw_deg,
                          imu->gz_dps,
                          state.gyro_bias_dps,
                          (float)(imu->raw_angle_frame_count & 0x00ffffffU),
                          (float)(imu->gyro_frame_count & 0x00ffffffU)};
    memcpy(telemetry, channels, sizeof(channels));
    telemetry[sizeof(channels)] = 0;
    telemetry[sizeof(channels) + 1] = 0;
    telemetry[sizeof(channels) + 2] = 0x80;
    telemetry[sizeof(channels) + 3] = 0x7f;
    (void)HAL_UART_Transmit_IT(PINCFG_VOFA_UART, telemetry, sizeof(telemetry));
}
void Chassis_RecordDeadlineMiss(void)
{
    state.deadline_misses++;
}
/* Only called by the chassis task, after IMU/fault checks and debug commands. */
static void service_host_commands(uint32_t now)
{
    uint8_t bytes[HOST_UART_CAPACITY];
    int count = HostUart_Read(bytes, sizeof(bytes));
    if (count < 0)
    {
        Chassis_Stop();
        host_parser = (HostParser){.discard = true};
        host_result = HOST_RX_ERROR;
        host_sequence = (host_sequence + 1U) & 0x00ffffffU;
        return;
    }
    /* Never join an old partial command with a much later fragment. */
    if (host_parser.length && DWT_DeltaSec(now, host_byte_cycle) > 0.5f)
        host_parser.discard = true;
    if (count > 0)
        host_byte_cycle = now;
    for (int i = 0; i < count; ++i)
    {
        HostCommand command;
        int result = HostCommand_Feed(&host_parser, bytes[i], &command);
        if (result == HOST_IDLE)
            continue;
        host_sequence = (host_sequence + 1U) & 0x00ffffffU;
        host_result = result;
        if (result < 0)
            continue;
        host_result = HostCommand_Check(&command, state.armed, planner.active || jog_remaining > 0);
        if (host_result != HOST_OK)
            continue;
        if (command.kind == HOST_STOP)
        {
            Chassis_Stop();
            HostUart_Flush();
            host_parser = (HostParser){0};
            break; /* Discard the rest of this snapshot too. */
        }
        if (command.kind == HOST_ARM)
        {
            if (!state.armed && !Chassis_Arm())
                host_result = HOST_NOT_READY;
        }
        else
        {
            float x = command.kind == HOST_FORWARD ? command.distance_mm : 0;
            float y = command.kind == HOST_SHIFT ? command.distance_mm : 0;
            if (!Chassis_Move(x, y, 450.519f, 550.0f, 550.0f))
                host_result = HOST_NOT_READY;
        }
    }
}
void Chassis_Update(void)
{
    uint32_t now = DWT_GetCycle();
    float dt = DWT_DeltaSec(now, last_cycle);
    service_tx();
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
    if (state.armed && !config_valid())
    {
        state.fault |= 8;
        Chassis_Stop();
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
    /* Experimental heading: accumulate accepted angle-frame deltas, no gz fusion.
     * LOST clears the reference; only GOOD may establish a fresh zero. */
    bool yaw_was_ready = relative_yaw.ready;
    RelativeYaw_Update(&relative_yaw, imu->yaw_deg, imu->angle_frame_count,
                       imu->trust != JY60_TRUST_LOST, imu->trust == JY60_TRUST_GOOD);
    state.yaw_rad = relative_yaw.yaw_rad;
    if (!relative_yaw.ready)
    {
        if (state.armed)
            state.fault |= 2;
        Chassis_Stop();
    }
    if (!relative_yaw.ready || !yaw_was_ready)
    {
        heading = state.yaw_rad;
        integral = 0;
    }
    last_gyro = imu->gyro_frame_count;
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
    service_host_commands(now);
    state.applied_path_speed =
        ChassisOdom_PathSpeed(geometry(), state.rpm_applied, chassis_config.left_odom_scale,
                              chassis_config.right_odom_scale, dx, dy);
    bool committed = tx_busy || tx_part != 0;
    state.committed_path_speed =
        committed
            ? ChassisOdom_PathSpeed(geometry(), state.rpm_inflight, chassis_config.left_odom_scale,
                                    chassis_config.right_odom_scale, dx, dy)
            : state.applied_path_speed;
    /* Reserve a control cycle plus both in-progress and subsequent wire time.
     * Legacy batches span multiple task wakeups, so use a conservative 60 ms. */
    bool legacy =
        chassis_config.command_mode == ZDT_LEGACY_SYNC || (committed && tx_mode == ZDT_LEGACY_SYNC);
    float response_delay =
        legacy ? 0.060f : 0.010f + 2.0f * ZDT_MULTI_SPEED_SIZE * 10.0f / PINCFG_ZDT_BAUDRATE;
    float speed = state.armed ? Planner_UpdateProgress(&planner, dt, segment_progress,
                                                       state.applied_path_speed,
                                                       state.committed_path_speed, response_delay)
                              : 0;
    float vx = speed * dx, vy = speed * dy, wz = 0;
    state.yaw_error = Angle_Wrap(heading - state.yaw_rad);
    if (state.armed)
    {
        wz = Heading_Update(state.yaw_error, (imu->gz_dps - state.gyro_bias_dps) * RAD, dt,
                            chassis_config.kp, chassis_config.ki, chassis_config.gyro_damping,
                            chassis_config.wz_limit, &integral);
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
    if (!config_valid())
    {
        Chassis_Stop();
        service_tx();
        send_telemetry(0, 0, 0, now);
        return;
    }
    float t[4], r[4], out[4];
    Mecanum_Inverse(geometry(), vx, vy, 0, t);
    if (planner.braking && planner.active && jog_remaining <= 0)
    {
        /* Prevent lateral compensation from amplifying the braking envelope.
         * Rotation-priority limiting may only reduce this translation. */
        float projected = ChassisOdom_PathSpeed(geometry(), t, chassis_config.left_odom_scale,
                                                chassis_config.right_odom_scale, dx, dy);
        if (projected > speed && projected > 0)
            for (int i = 0; i < 4; ++i)
                t[i] *= speed / projected;
    }
    Mecanum_Inverse(geometry(), 0, 0, wz, r);
    Wheel_Limit(t, r, chassis_config.rpm_limit, out);
    for (int i = 0; i < 4; i++)
    {
        state.rpm_requested[i] = out[i];
        state.rpm_pending[i] = roundf(state.rpm_requested[i]);
    }
    pending_valid = true;
    service_tx();
    send_telemetry(vx, vy, wz, now);
}
