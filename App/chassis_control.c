#include "path_config.h"
#include "chassis_control.h"
#include "host_command.h"
#include "host_uart.h"
#include "path_ports.h"
#include "path_mission.h"
#include "path_yaw.h"
#include "stair_heading.h"
#include "forward_comp.h"
#include "motion.h"
#include "heading.h"
#include "heading_tuning.h"
#include "chassis_tuning.h"
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
#ifndef CHASSIS_TELEMETRY_ENABLE
#define CHASSIS_TELEMETRY_ENABLE 1
#endif
#ifndef CHASSIS_TELEMETRY_FULL
#define CHASSIS_TELEMETRY_FULL 1
#endif
ChassisConfig chassis_config = {.wheel_radius_mm = 37.5f,
                                .half_track_mm = 128.5f, /* Measure before arming. */
                                .half_wheelbase_mm = 130.5f,
                                .rpm_limit = 260.0f,
                                .kp = 4.0f,
                                .ki = 0.00f,
                                .gyro_damping = 0.15f,
                                .wz_limit = 1.9f,
                                .left_gain = 1.0f,
                                .right_gain = 1.0f,
                                .forward_lateral_comp = 0.017f,
                                .left_odom_scale = 1.0f,
                                .right_odom_scale = 1.0f,
                                .motor_id = {2, 1, 3, 4},
                                .motor_sign = {1, -1, 1, -1}, /* FL/FR/RL/RR: IDs 2/1/3/4. */
                                .calibrated = true,
                                .command_mode = ZDT_MULTI_COMMAND};
static ChassisState state;
volatile ChassisHeadingDiagnostics chassis_heading_diagnostics;
static ChassisRoutePolicy route_policy = {.stair_target_deg=STAIR_MAP_TARGET_DEG, .travel_speed_scale=1.0f};
void Chassis_SetRoutePolicy(ChassisRoutePolicy policy)
{
    if (isfinite(policy.stair_target_deg)) route_policy=policy;
}
volatile ChassisDebugCommand chassis_debug;
static Planner planner;
static RelativeYaw relative_yaw;
static HeadingEstimator heading_estimator;
static PathYaw path_yaw;
static float dx, dy, heading, integral, bias_sum, bias_elapsed;
static float segment_progress;
static float jog_x, jog_y, jog_w, jog_remaining;
static bool path_rotation, path_body, path_arc, zero_output;
static bool path_blend, blend_continuous, path_heading_chain;
/* Final-heading translation frame, retained across moving handoffs. */
static float chain_yaw, chain_x, chain_y;
static float blend_x, blend_y, blend_yaw, blend_turn;
static float path_target, body_x, body_y, body_w, arc_start, arc_turn;
static uint8_t rotate_stable_count;
static float rotate_tolerance_deg = HEADING_STATIC_TOLERANCE_DEG;
static bool rotate_measured_zero;
static bool rotate_map_precision;
static float line_yaw_reference;
static float route_heading;
static bool path_heading_enabled, normal_stopping, line_search;
static bool capture_braking;
static float body_output[3]; /* Last wheel-limited body command, not measured velocity. */
static bool path_heading_active(void)
{
    return path_heading_enabled && PathPorts_Busy();
}
static uint32_t moving_tick;
static uint32_t last_cycle, last_gyro, bias_count;
#if CHASSIS_TELEMETRY_ENABLE
static uint32_t telemetry_cycle, telemetry_sequence;
#endif
static uint8_t tx[ZDT_MULTI_SPEED_SIZE] __attribute__((aligned(32)));
static volatile bool tx_done, tx_busy, tx_error;
static bool pending_valid;
static uint32_t tx_cycle;
static volatile uint32_t tx_complete_cycle;
static uint32_t odom_cycle;
#if CHASSIS_TELEMETRY_ENABLE
#if CHASSIS_TELEMETRY_FULL
static uint8_t telemetry[200]; /* 38 existing + 11 diagnostic floats + tail. */
#else
static uint8_t telemetry[180]; /* 33 existing + 11 diagnostic floats + tail. */
#endif
#endif
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
static void update_path_direction(void)
{
    if (path_blend)
        Motion_FixedDirection(blend_x, blend_y, path_yaw.continuous - blend_yaw, &dx, &dy);
    else if (path_heading_chain)
    {
        float x = chain_x, y = chain_y;
        if (path_arc)
            Motion_ArcDirection(arc_start, arc_turn, segment_progress, planner.distance, &x, &y);
        Motion_FixedDirection(x, y, path_yaw.continuous - chain_yaw, &dx, &dy);
    }
    else if (planner.active && path_arc)
        Motion_ArcDirection(arc_start, arc_turn, segment_progress, planner.distance, &dx, &dy);
}
static void integrate_odom(uint32_t cycle)
{
    update_path_direction();
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
    if (!ForwardComp_ConfigValid(c->forward_lateral_comp))
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
        int16_t physical_deci_rpm[4];
        for (int i = 0; i < 4; ++i)
            physical_deci_rpm[i] = (int16_t)lroundf(state.rpm_pending[i] * 10.0f * chassis_config.motor_sign[i]);
        tx_mode = chassis_config.command_mode;
        if (tx_mode == ZDT_MULTI_COMMAND)
        {
            tx_length =
                ZDT_BuildMultiSpeedDeci(tx, sizeof(tx), chassis_config.motor_id, physical_deci_rpm, 0);
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
                ZDT_BuildLegacySpeedDeci(&tx[i * 8], chassis_config.motor_id[i], physical_deci_rpm[i], 0);
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
    PathPorts_Error(uart);
    HostUart_Error(uart);
    if (uart == PINCFG_ZDT_UART) {
        tx_error = true;
    }
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
    PathPorts_RxComplete(uart);
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
    HeadingEstimator_Reset(&heading_estimator);

    /* 4. 初始化底盘时间戳 */
    uint32_t now = DWT_GetCycle();

    last_cycle = now;
#if CHASSIS_TELEMETRY_ENABLE
    telemetry_cycle = now;
#endif
    odom_cycle = now;

    if (!HostUart_Init())
        return false;

    /* A failed mission peripheral prevents PATH, not manual chassis diagnosis. */
    PathPorts_Init();
    moving_tick = HAL_GetTick();

    return true;
}
bool Chassis_Arm(void)
{
    if (!chassis_config.calibrated || !config_valid() || !state.bias_ready || !relative_yaw.ready ||
        !heading_estimator.ready || state.fault || JY60_GetState()->trust != JY60_TRUST_GOOD)
        return false;
    state.armed = true;
    heading = state.yaw_rad;
    integral = 0;
    return true;
}
static float home_x, home_y, map_yaw, home_right_target;
static bool home_return_ready;
void Chassis_BeginPath(void)
{
    home_x = state.x_mm; home_y = state.y_mm; map_yaw = state.yaw_rad;
    home_return_ready=false;
    route_heading = heading = state.yaw_rad;
    path_heading_enabled = true;
    line_yaw_reference = Chassis_MeasuredYaw();
    integral = 0;
}
bool Chassis_ReturnHome(unsigned leg)
{
    if (!path_heading_enabled || !state.armed || !Chassis_IsSettled()) return false;
    if (route_policy.mirror_map_y) {
        if (leg>2) return false;
        /* Blue fixed return: keep the unloading heading and use body-frame directions. */
        float boost=PATH_TRAVEL_BOOST*route_policy.travel_speed_scale;
        return Chassis_Move(leg==0 ? 2250.0f : 0,leg==1 ? -1000.0f : leg==2 ? -CHASSIS_HOME_SEARCH_MM : 0,
                            (leg==2 ? CHASSIS_HOME_SEARCH_SPEED_MM_S : CHASSIS_HOME_SPEED_MM_S*boost),CHASSIS_HOME_ACCEL_MM_S2*boost,
                            CHASSIS_HOME_BRAKE_MM_S2*route_policy.travel_speed_scale);
    }
    /* Return travel keeps heading control active without an angle admission gate. */
    if (leg>2 || !isfinite(state.x_mm) || !isfinite(state.y_mm) || !isfinite(state.yaw_rad)) return false;
    float travel;
    float current_y=-sinf(map_yaw)*state.x_mm+cosf(map_yaw)*state.y_mm;
    if (leg==0) {
        float x=home_x-state.x_mm, y=home_y-state.y_mm;
        float distance=hypotf(x,y);
        float scale=distance>=5 ? (distance+CHASSIS_HOME_DIAGONAL_EXTEND_MM)/distance : 1;
        float my=(-sinf(map_yaw)*x+cosf(map_yaw)*y)*scale;
        travel=1600.0f;
        home_right_target=current_y+-1*(fabsf(my)+(distance>=5 ? CHASSIS_HOME_Y_EXTEND_MM : 0));
        home_return_ready=true;
    } else {
        if (!home_return_ready) return false;
        float remaining=fmaxf(0,1*(current_y-home_right_target));
        travel=leg==1 ? fmaxf(0,remaining-CHASSIS_HOME_SEARCH_MM) : remaining+CHASSIS_HOME_SEARCH_MM;
    }
    if (travel < 0.5f) return true;
    float angle=state.yaw_rad-map_yaw, c=cosf(angle), sn=sinf(angle);
    float lateral=-travel;
    float forward=route_policy.mirror_map_y ? travel : -travel;
    float bx=leg==0 ? forward*c : lateral*sn;
    float by=leg==0 ? -forward*sn : lateral*c;
    route_heading = heading = map_yaw;
    integral = 0;
    float boost=leg==2 ? 1.0f : PATH_TRAVEL_BOOST;
    return Chassis_Move(bx, by, (leg==2 ? CHASSIS_HOME_SEARCH_SPEED_MM_S : CHASSIS_HOME_SPEED_MM_S)*route_policy.travel_speed_scale*boost,
                        CHASSIS_HOME_ACCEL_MM_S2*route_policy.travel_speed_scale*boost, CHASSIS_HOME_BRAKE_MM_S2*route_policy.travel_speed_scale);
}
void Chassis_Stop(void)
{
    path_heading_enabled = false;
    normal_stopping = false;
    memset(body_output, 0, sizeof(body_output));
    path_blend = blend_continuous = path_heading_chain = false;
    PathPorts_Cancel();
    state.armed = false;
    path_rotation = path_body = path_arc = false;
    line_search = false;
    rotate_measured_zero = false;
    rotate_map_precision = false;
    zero_output = true;
    planner.active = false;
    jog_remaining = 0;
    integral = 0;
    memset(state.rpm_requested, 0, sizeof(state.rpm_requested));
    memset(state.rpm_pending, 0, sizeof(state.rpm_pending));
    pending_valid = true;
}
bool Chassis_Move(float x, float y, float v, float a, float d)
{
    return Chassis_MoveBoundary(x, y, v, a, d, 0, 0);
}
bool Chassis_MoveBoundary(float x, float y, float v, float a, float d,
                          float start_speed, float end_speed)
{
    if (!state.armed || Chassis_MotionBusy() || !isfinite(x) || !isfinite(y))
        return false;
    float length = hypotf(x, y);
    if (!Planner_StartBoundary(&planner, length, v, a, d, start_speed, end_speed))
        return false;
    jog_remaining = 0;
    path_arc = false;
    zero_output = false;
    segment_progress = 0;
    dx = chain_x = x / length;
    dy = chain_y = y / length;
    if (path_heading_active()) heading = route_heading;
    else if (!path_heading_chain) heading = state.yaw_rad;
    update_path_direction();
    return true;
}
bool Chassis_ExitOrbitArc(float radius, float target_deg, float speed, float acceleration, float deceleration)
{
    if (!state.armed || !path_heading_enabled || normal_stopping || planner.active ||
        path_blend || path_arc || path_rotation || !isfinite(target_deg)) return false;
    bool was_body=path_body;
    float old_heading=heading, old_route=route_heading;
    path_body=false;
    heading=route_heading=Angle_Wrap(map_yaw+target_deg*RAD);
    /* Keep wheel output continuous; the existing slew limiter handles the transition. */
    if (Chassis_MoveArc(radius,180,-90,speed,acceleration,deceleration,speed,speed)) return true;
    path_body=was_body;heading=old_heading;route_heading=old_route;
    return false;
}
bool Chassis_ExitOrbit(float forward, float lateral, float target_deg, float speed, float end_speed, float acceleration, float deceleration)
{
    float distance=hypotf(forward,lateral);
    if (!state.armed || !path_heading_enabled || normal_stopping || planner.active || path_blend ||
        path_arc || path_rotation || !isfinite(distance) || fabsf(distance)<1 ||
        !isfinite(target_deg) || !isfinite(state.yaw_rad) || !isfinite(speed) || speed<=0 ||
        !isfinite(end_speed) || end_speed<=0 || !isfinite(acceleration) || acceleration<=0 ||
        !isfinite(deceleration) || deceleration<=0)
        return false;
    float target=map_yaw+target_deg*RAD;
    float turn=Angle_Wrap(target-state.yaw_rad);
    /* Translate along the stair axis while yaw converges, carrying orbit velocity.
     * Do not call Hold: output slew limits handle the angular-rate transition. */
    float x=forward*cosf(turn)-lateral*sinf(turn);
    float y=forward*sinf(turn)+lateral*cosf(turn);
    float start=fmaxf(0,(state.velocity[0]*x+state.velocity[1]*y)/fabsf(distance));
    bool was_body=path_body;
    float old_heading=heading, old_route=route_heading;
    path_body=false;
    heading=route_heading=state.yaw_rad;
    bool ok;
    if (fabsf(turn)<0.01f*RAD) {
        route_heading=heading=Angle_Wrap(target);
        ok=Chassis_MoveBoundary(x,y,fmaxf(speed,start),acceleration,deceleration,start,end_speed);
    } else ok=Chassis_MoveRotateBoundary(x,y,turn/RAD,fmaxf(speed,start),acceleration,deceleration,start,end_speed);
    if (!ok) { path_body=was_body;heading=old_heading;route_heading=old_route; }
    return ok;
}
bool Chassis_FinishForward(float distance, float speed, float acceleration, float deceleration)
{
    if (!state.armed || !isfinite(distance) || fabsf(distance)<0.01f) return false;
    if (!Chassis_MotionBusy()) return Chassis_Move(distance,0,speed,acceleration,deceleration);
    if (!planner.active || normal_stopping || path_arc || path_blend || path_rotation || path_body ||
        fabsf(chain_x-(distance>0 ? 1.0f : -1.0f))>0.001f || fabsf(chain_y)>0.001f) return false;
    float start=fmaxf(0, state.velocity[0]*dx+state.velocity[1]*dy);
    Planner next;
    if (!Planner_StartBoundary(&next,fabsf(distance),fmaxf(speed,start),acceleration,deceleration,start,0)) return false;
    planner=next;
    segment_progress=0;
    return true;
}
bool Chassis_MoveRotate(float x, float y, float degrees, float v, float a, float d)
{
    return Chassis_MoveRotateBoundary(x, y, degrees, v, a, d, 0, 0);
}
bool Chassis_MoveRotateBoundary(float x, float y, float degrees, float v, float a, float d,
                                 float start_speed, float end_speed)
{
    if (!isfinite(degrees) || fabsf(degrees) > 360 || fabsf(degrees) < 0.01f)
        return false;
    if (!Chassis_MoveBoundary(x, y, v, a, d, start_speed, end_speed)) return false;
    path_heading_chain = false;
    blend_continuous = end_speed > 0;
    dx = blend_x = x / planner.distance;
    dy = blend_y = y / planner.distance;
    heading = path_heading_active() ? route_heading : state.yaw_rad;
    blend_yaw = path_yaw.continuous + Angle_Wrap(heading - state.yaw_rad);
    blend_turn = degrees * RAD;
    if (path_heading_active()) route_heading = Angle_Wrap(heading + blend_turn);
    rotate_stable_count = 0;
    integral = 0;
    path_blend = true;
    return true;
}
bool Chassis_MoveArc(float radius, float start_degrees, float turn_degrees,
                     float v, float a, float d, float start_speed, float end_speed)
{
    if (!state.armed || Chassis_MotionBusy() || !isfinite(radius) || !isfinite(start_degrees) ||
        !isfinite(turn_degrees) || radius <= 0 || fabsf(turn_degrees) < 0.01f ||
        fabsf(turn_degrees) > 180.0f)
        return false;
    float turn = turn_degrees * RAD;
    float length = radius * fabsf(turn);
    if (!Planner_StartBoundary(&planner, length, v, a, d, start_speed, end_speed))
        return false;
    jog_remaining = 0;
    zero_output = false;
    segment_progress = 0;
    path_arc = true;
    arc_start = start_degrees * RAD;
    arc_turn = turn;
    if (path_heading_active()) heading = route_heading;
    else if (!path_heading_chain) heading = state.yaw_rad;
    update_path_direction();
    return true;
}
void Chassis_Hold(void)
{
    capture_braking = false;
    path_blend = blend_continuous = path_heading_chain = false;
    planner.active = false;
    path_rotation = path_body = path_arc = false;
    line_search = false;
    rotate_measured_zero = false;
    rotate_map_precision = false;
    rotate_stable_count = 0;
    jog_remaining = 0;
    zero_output = true;
    normal_stopping = body_output[0] != 0 || body_output[1] != 0 || body_output[2] != 0;
    heading = path_heading_active() ? route_heading : state.yaw_rad;
    integral = 0;
}
void Chassis_HoldCapture(void)
{
    Chassis_Hold();
    capture_braking = true;
}
void Chassis_HoldImmediate(void)
{
    Chassis_Hold();
    normal_stopping = false;
    memset(body_output, 0, sizeof(body_output));
    memset(state.rpm_requested, 0, sizeof(state.rpm_requested));
    memset(state.rpm_pending, 0, sizeof(state.rpm_pending));
    pending_valid = true;
}
bool Chassis_MotionBusy(void)
{
    return normal_stopping || planner.active || path_blend || path_rotation || path_body || jog_remaining > 0;
}
bool Chassis_IsSettled(void)
{
    if (Chassis_MotionBusy())
        return false;
    for (int i = 0; i < 4; ++i)
        if (state.rpm_applied[i] != 0 || state.rpm_requested[i] != 0 ||
            ((tx_busy || tx_part) && state.rpm_inflight[i] != 0))
            return false;
    return (uint32_t)(HAL_GetTick() - moving_tick) >= CHASSIS_SETTLED_MS;
}
bool Chassis_Rotate(float degrees)
{
    if (!state.armed || Chassis_MotionBusy() || !isfinite(degrees) || fabsf(degrees) > 360)
        return false;
    rotate_tolerance_deg = HEADING_STATIC_TOLERANCE_DEG;
    rotate_measured_zero = false;
    rotate_map_precision = false;
    float start_heading = path_heading_active() ? route_heading : state.yaw_rad;
    path_target = path_yaw.continuous + Angle_Wrap(start_heading - state.yaw_rad) + degrees * RAD;
    if (path_heading_active()) route_heading = Angle_Wrap(path_target);
    path_rotation = true;
    rotate_stable_count = 0;
    zero_output = false;
    integral = 0;
    return true;
}
bool Chassis_SetMapHeading(float degrees)
{
    if (!isfinite(degrees) || !path_heading_enabled || !state.armed || !Chassis_IsSettled()) return false;
    route_heading = heading = Angle_Wrap(map_yaw + degrees * RAD);
    integral = 0;
    return true; /* No stationary rotation; subsequent moves close the yaw loop. */
}
static bool align_map_heading(float degrees, float tolerance)
{
    if (!path_heading_enabled || !Chassis_IsSettled()) return false;
    float target = Angle_Wrap(map_yaw + degrees * RAD);
    float error = Angle_Wrap(target - state.yaw_rad);
    if (!isfinite(error) || !Chassis_Rotate(0)) return false;
    path_target = path_yaw.continuous + error;
    route_heading = heading = target;
    rotate_tolerance_deg = tolerance;
    rotate_map_precision = true;
    return true;
}

bool Chassis_AlignMapAxis(void) { return align_map_heading(route_policy.stair_target_deg, STAIR_HEADING_STOP_TOLERANCE_DEG); }
bool Chassis_AlignHome(float target_deg) { return align_map_heading(target_deg, HEADING_STATIC_TOLERANCE_DEG); }
bool Chassis_MapSearch(float mm_s)
{
    if (!path_heading_enabled) return false;
    float angle = map_yaw - state.yaw_rad;
    return Chassis_Body(-sinf(angle)*mm_s, cosf(angle)*mm_s, 0);
}
bool Chassis_MapLateral(float mm)
{
    if (!path_heading_enabled || !Chassis_IsSettled()) return false;
    float angle = map_yaw - state.yaw_rad;
    return Chassis_Move(-sinf(angle)*mm, cosf(angle)*mm, 30*route_policy.travel_speed_scale, 300*route_policy.travel_speed_scale, 300*route_policy.travel_speed_scale);
}
bool Chassis_AlignZero(void)
{
    if (!Chassis_IsSettled() ||
        !Chassis_Rotate(StairHeading_Error(Chassis_LineYaw())))
        return false;
    /* This command is already an error from actual yaw, not a relative route turn. */
    path_target = path_yaw.continuous + StairHeading_Error(Chassis_LineYaw()) * RAD;
    if (path_heading_active()) route_heading = Angle_Wrap(path_target);
    rotate_tolerance_deg = STAIR_HEADING_STOP_TOLERANCE_DEG;
    rotate_measured_zero = true;
    return true;
}
bool Chassis_Body(float x, float y, float w)
{
    if (!state.armed || normal_stopping || planner.active || path_blend || path_rotation || !isfinite(x) || !isfinite(y) ||
        !isfinite(w))
        return false;
    if (!path_body)
        heading = path_heading_active() ? route_heading : state.yaw_rad;
    line_search = false;
    body_x = x;
    body_y = y;
    body_w = w;
    path_body = true;
    zero_output = false;
    return true;
}
bool Chassis_LineSearch(float lateral, float w)
{
    if (!Chassis_Body(0, lateral, w)) return false;
    line_search = true; /* Gray probes own yaw during physical alignment. */
    return true;
}
bool Chassis_CalibrateLine(void)
{
    float yaw = Chassis_MeasuredYaw();
    if (!Chassis_IsSettled() || !isfinite(yaw)) return false;
    route_heading = heading = state.yaw_rad;
    line_yaw_reference = yaw;
    integral = 0;
    return true;
}
float Chassis_MeasuredYaw(void)
{
    const JY60_State_t *imu = JY60_GetState();
    return imu->has_angle && imu->trust != JY60_TRUST_LOST ? imu->yaw_deg : NAN;
}
float Chassis_LineYaw(void)
{
    return remainderf(Chassis_MeasuredYaw()-line_yaw_reference,360.0f);
}
bool Chassis_SetLineReference(void)
{
    float yaw=Chassis_MeasuredYaw();
    if (!Chassis_IsSettled() || !isfinite(yaw)) return false;
    /* Detection alone is not a physical heading calibration. Store the intended
     * route heading in the raw IMU frame so stair checks still see residual error. */
    line_yaw_reference = path_heading_active()
                             ? yaw + Angle_Wrap(route_heading - state.yaw_rad) / RAD : yaw;
    return true;
}
float Chassis_MapYaw(void)
{
    return Angle_Wrap(state.yaw_rad - map_yaw) / RAD;
}
float Chassis_ContinuousYaw(void)
{
    return path_yaw.continuous;
}
const ChassisState *Chassis_GetState(void)
{
    return &state;
}
#if CHASSIS_TELEMETRY_ENABLE
static void send_telemetry(float vx, float vy, float wz, float forward_comp_vy, float vy_original,
                           uint32_t now)
{
    if (DWT_DeltaSec(now, telemetry_cycle) < 0.025f ||
        PINCFG_VOFA_UART->gState != HAL_UART_STATE_READY)
        return;
    telemetry_cycle = now;
    const JY60_State_t *imu = JY60_GetState();
    /* These snapshots never write the sensor or controller state. */
    unsigned mode = (!state.armed || zero_output) ? 0U :
                    path_blend ? 6U : path_rotation ? (rotate_measured_zero ? 3U : 2U) :
                    path_body ? 4U : jog_remaining > 0 ? 5U : planner.active ? 1U : 0U;
    float feedback = mode == 3 ? Chassis_LineYaw() :
                     (mode == 2 || mode == 6) ? path_yaw.continuous / RAD : state.yaw_rad / RAD;
    float target = mode == 3 ? 0 : mode == 2 ? path_target / RAD : heading / RAD;
    if (mode == 6)
    {
        float fraction = planner.active ? fmaxf(0, fminf(1, segment_progress / planner.distance)) : 1;
        float turn = blend_turn * fraction, unused_rate;
        if (blend_continuous)
            Motion_SmoothTurn(fraction * planner.distance, planner.distance, blend_turn,
                              0, &turn, &unused_rate);
        target = (blend_yaw + turn) / RAD;
    }
    float error = (mode == 2 || mode == 6) ? target - feedback :
                  Angle_Wrap((target - feedback) * RAD) / RAD;
    float channels[] = {vx,
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
                        (float)imu->trust,
                        state.dt,
                        state.armed ? 1.0f : 0.0f,
                        (float)state.fault,
                        (float)host_result,
                        (float)host_sequence,
                        (Chassis_MotionBusy() || PathPorts_Busy()) ? 1.0f : 0.0f,
                        state.bias_ready ? 1.0f : 0.0f,
#if CHASSIS_TELEMETRY_FULL
                        imu->raw_yaw_deg,
                        imu->gz_dps,
                        state.gyro_bias_dps,
                        (float)(imu->raw_angle_frame_count & 0x00ffffffU),
                        (float)(imu->gyro_frame_count & 0x00ffffffU),
#endif
                        forward_comp_vy,
                        vy_original,
                        (float)path_diagnostics.result,
                        (float)path_diagnostics.step,
                        (float)path_diagnostics.accepted_ids,
                        (float)path_diagnostics.fault,
                        (float)path_diagnostics.link_stage,
                        (float)path_diagnostics.link_error,
                        (float)path_diagnostics.gray,
                        (float)path_diagnostics.phase,
                        (float)path_diagnostics.rfid_count,
                        (float)path_diagnostics.ir_raw,
                        (float)path_diagnostics.settled,
                        (float)(HAL_GetTick() & 0x00ffffffU),
                        (float)(telemetry_sequence++ & 0x00ffffffU),
                        imu->yaw_deg,
                        (float)(imu->angle_frame_count & 0x00ffffffU),
                        (float)mode,
                        target,
                        feedback,
                        error,
                        (float)path_diagnostics.point,
                        (float)(path_diagnostics.phase_elapsed_ms & 0x00ffffffU),
                        (float)imu->angle_age_ms};
    _Static_assert(sizeof(channels) + 4 == sizeof(telemetry), "VOFA frame size mismatch");
    const size_t channel_bytes = sizeof(channels);
    memcpy(telemetry, channels, channel_bytes);
    telemetry[channel_bytes] = 0;
    telemetry[channel_bytes + 1] = 0;
    telemetry[channel_bytes + 2] = 0x80;
    telemetry[channel_bytes + 3] = 0x7f;
    (void)HAL_UART_Transmit_IT(PINCFG_VOFA_UART, telemetry, sizeof(telemetry));
}
#else
#define send_telemetry(vx, vy, wz, forward_comp_vy, vy_original, now) ((void)0)
#endif
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
        host_result = HostCommand_Check(&command, state.armed,
                                        Chassis_MotionBusy() ||
                                            (command.kind != HOST_RDK_RESET && command.kind != HOST_RED && command.kind != HOST_BLUE && PathPorts_Busy()));
        if (host_result != HOST_OK)
            continue;
        if (command.kind == HOST_STOP)
        {
            Chassis_Stop();
            HostUart_Flush();
            host_parser = (HostParser){0};
            break; /* Discard the rest of this snapshot too. */
        }
        if (command.kind == HOST_PING)
        {
            if (!PathPorts_Ping()) host_result = HOST_NOT_READY;
        }
        else if (command.kind == HOST_RDK_RESET)
        {
            if (!PathPorts_Reset()) host_result = HOST_NOT_READY;
        }
        else host_result = HOST_NOT_READY; /* Starts/motion now require the infrared gesture. */
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
        if (state.armed) {
            state.fault |= 2;
            Chassis_Stop();
        } /* Initial IMU warmup must not cancel the automatic RDK boot handshake. */
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
    /* HWT101CT and control loop both run at 200 Hz. Propagate with corrected
     * gyro between asynchronous angle arrivals; LOST clears both references. */
    bool yaw_was_ready = heading_estimator.ready;
    RelativeYaw_Update(&relative_yaw, imu->yaw_deg, imu->angle_frame_count,
                       imu->trust != JY60_TRUST_LOST, imu->trust == JY60_TRUST_GOOD);
    float gyro_rad_s = (imu->gz_dps - state.gyro_bias_dps) * RAD;
    bool yaw_ready =
        relative_yaw.ready &&
        HeadingEstimator_Update(&heading_estimator, relative_yaw.yaw_rad, imu->angle_frame_count,
                                gyro_rad_s, dt, imu->trust != JY60_TRUST_LOST);
    state.yaw_rad = yaw_ready ? heading_estimator.yaw_rad : 0;
    PathYaw_Update(&path_yaw, state.yaw_rad, yaw_ready);
    if (!yaw_ready)
    {
        if (state.armed) {
            state.fault |= 2;
            Chassis_Stop();
        } /* Initial IMU warmup must not cancel the automatic RDK boot handshake. */
    }
    if (!yaw_ready || !yaw_was_ready)
    {
        heading = state.yaw_rad;
        integral = 0;
    }
    last_gyro = imu->gyro_frame_count;
    update_path_direction();
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
        if (command == 2 && !PathPorts_Busy())
            ok = Chassis_Arm();
        if (command == 3 && !PathPorts_Busy())
            ok = Chassis_Move(chassis_debug.x, chassis_debug.y, chassis_debug.vmax,
                              chassis_debug.amax, chassis_debug.dmax);
        /* Timed low-speed jog: x/y mm/s, vmax rad/s, amax duration seconds. */
        if (command == 4 && !PathPorts_Busy() && state.armed && isfinite(chassis_debug.x) &&
            isfinite(chassis_debug.y) && isfinite(chassis_debug.vmax) &&
            isfinite(chassis_debug.amax) && chassis_debug.amax > 0 && chassis_debug.amax <= 2)
        {
            jog_x = clamp(chassis_debug.x, 100);
            jog_y = clamp(chassis_debug.y, 100);
            jog_w = clamp(chassis_debug.vmax, 0.3f);
            jog_remaining = chassis_debug.amax;
            planner.active = false;
            zero_output = false;
            ok = true;
        }
        chassis_debug.result = ok ? 0 : -1;
        chassis_debug.acknowledged = sequence;
    }
    service_host_commands(now);
    PathPorts_Tick();
    update_path_direction();
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
    bool blending_this_cycle = path_blend;
    float vx = speed * dx, vy = speed * dy, wz = 0;
    float lateral_direction = dy;
    if (route_policy.force_stair_heading && path_heading_enabled &&
        !path_rotation && !path_blend)
        route_heading=heading=Angle_Wrap(map_yaw+route_policy.stair_target_deg*RAD);
    state.yaw_error = Angle_Wrap(heading - state.yaw_rad);
    bool translating = !zero_output && !path_rotation &&
                       (planner.active || (path_body && body_w == 0 && !line_search));
    HeadingControlConfig yaw_config={chassis_config.kp, chassis_config.ki,
                                     chassis_config.gyro_damping, chassis_config.wz_limit};
    HeadingRequest yaw_request={.mode=translating ? HEADING_TRAVEL : HEADING_FIXED,
                                .error=state.yaw_error, .gyro=gyro_rad_s};
    if (state.armed && path_blend)
    {
        float fraction = planner.active ? fmaxf(0, fminf(1, segment_progress / planner.distance)) : 1;
        float turn = blend_turn * fraction;
        /* Target progress follows committed wheel commands, not the uncapped planner. */
        float progress_speed = fmaxf(0,state.applied_path_speed);
        float feedforward = planner.active ? blend_turn * progress_speed / planner.distance : 0;
        if (blend_continuous)
            Motion_SmoothTurn(fraction * planner.distance, planner.distance, blend_turn,
                              progress_speed, &turn, &feedforward);
        float error = blend_yaw + turn - path_yaw.continuous;
        state.yaw_error = error;
        yaw_request.mode=HEADING_DYNAMIC;
        yaw_request.error=error;
        yaw_request.feedforward=feedforward;
        if (route_policy.brake_turn_endpoint) {
            float remaining=blend_yaw+blend_turn-path_yaw.continuous;
            float direction=blend_turn>0 ? 1.0f : -1.0f;
            bool blue=route_policy.mirror_map_y;
            float projected=remaining*direction;
            if(!blue) projected-=fmaxf(0,gyro_rad_s*direction)*CHASSIS_BLEND_END_LOOKAHEAD_S;
            float distance=blue ? fabsf(remaining) : fmaxf(0,projected);
            float limit=sqrtf(2.0f*(blue ? 1.5f : CHASSIS_BLEND_END_BRAKE_RAD_S2)*distance);
            yaw_config.limit=fminf(yaw_config.limit,fmaxf(HEADING_FINE_LIMIT,limit));
            if(!blue) yaw_config.damping=fmaxf(yaw_config.damping,CHASSIS_BLEND_END_DAMPING);
            if (projected<=0) {
                if(!blue) integral=0;
                yaw_request.feedforward=0;
                yaw_request.error=remaining;state.yaw_error=remaining;
            } else yaw_request.feedforward=fmaxf(-yaw_config.limit,
                                      fminf(yaw_config.limit,yaw_request.feedforward));
        }
        if (blend_continuous && !planner.active)
        {
            /* Keep this cycle's end speed; next tick emits the arc without Hold.
             * Carry commanded heading so residual yaw error is corrected in motion. */
            chain_yaw = blend_yaw + blend_turn;
            Motion_FixedDirection(blend_x, blend_y, blend_turn, &chain_x, &chain_y);
            heading = Angle_Wrap(chain_yaw);
            path_heading_chain = true;
            path_blend = blend_continuous = false;
        }
        else if (!blend_continuous)
        {
            if (!planner.active && fabsf(error) < 0.5f * RAD && fabsf(gyro_rad_s) < 2 * RAD)
                rotate_stable_count++;
            else
                rotate_stable_count = 0;
            if (rotate_stable_count >= HEADING_SETTLE_CYCLES) Chassis_Hold();
        }
    }
    if (state.armed && jog_remaining > 0)
    {
        vx = jog_x;
        vy = jog_y;
        lateral_direction = jog_y;
        yaw_request.mode=HEADING_MANUAL;
        yaw_request.feedforward=jog_w;
        jog_remaining = fmaxf(0, jog_remaining - dt);
        heading = state.yaw_rad;
        integral = 0;
    }
    if (state.armed && path_rotation)
    {
        /* Stair zeroing follows each accepted sensor angle, not the integrated estimate. */
        float error = rotate_measured_zero
                          ? StairHeading_Error(imu->yaw_deg-line_yaw_reference) * RAD
                          : path_target - path_yaw.continuous; /* continuous for normal turns */
        vx = vy = 0;
        float limit = 60.0f * (2.0f * 3.141592654f * chassis_config.wheel_radius_mm / 60.0f) /
                      (chassis_config.half_track_mm + chassis_config.half_wheelbase_mm);
        if (!rotate_map_precision && !rotate_measured_zero) limit *= route_policy.travel_speed_scale;
        float abs_error = fabsf(error);
        float rotate_limit;
        if (abs_error > 15.0f * RAD)
            rotate_limit = fminf(limit, chassis_config.wz_limit);
        else if (abs_error > 3.0f * RAD)
            rotate_limit = fminf(limit, chassis_config.wz_limit * 0.6f);
        else
            rotate_limit = fminf(limit, 0.25f * chassis_config.wz_limit);
        yaw_request.mode=rotate_map_precision && abs_error<=HEADING_FINE_RANGE
                             ? HEADING_PRECISION : HEADING_ROTATE;
        yaw_request.error=error;
        yaw_request.limit=rotate_limit;
        state.yaw_error=error;
        if (rotate_measured_zero) yaw_config.kp=STAIR_HEADING_KP;
        if (abs_error < rotate_tolerance_deg * RAD && fabsf(imu->gz_dps - state.gyro_bias_dps) < 2)
            rotate_stable_count++;
        else
            rotate_stable_count = 0;
        if (rotate_stable_count >= HEADING_SETTLE_CYCLES)
            Chassis_Hold();
    }
    if (state.armed && path_body)
    {
        vx = body_x;
        vy = body_y;
        lateral_direction = body_y;
        if (body_w != 0 || line_search)
        {
            yaw_request.mode=HEADING_MANUAL;
            yaw_request.feedforward=body_w;
            heading = path_heading_active() ? route_heading : state.yaw_rad;
            integral = 0;
        }
    }
    /* Keep stationary yaw active from post-orbit alignment through the stairs.
     * STOP, faults, other stages and motion commands retain their original behavior. */
    float stair_hold_error = Angle_Wrap(map_yaw + route_policy.stair_target_deg * RAD - state.yaw_rad);
    bool stair_hold = state.armed && !state.fault && path_heading_enabled &&
                      zero_output && !normal_stopping && route_policy.stationary_hold &&
                      (route_policy.force_stair_heading || route_policy.hold_during_action ||
                       fabsf(stair_hold_error)<HEADING_HOLD_ENTRY);
    if (zero_output || !state.armed) {
        vx=vy=0;
        yaw_request.mode=HEADING_OFF;
    }
    if (stair_hold) {
        heading=Angle_Wrap(map_yaw+route_policy.stair_target_deg*RAD);
        state.yaw_error=Angle_Wrap(heading-state.yaw_rad);
        yaw_request.mode=HEADING_HOLD;
        yaw_request.error=state.yaw_error;
    }
    /* Only the final selected mode evaluates the angle loop. */
    if (yaw_request.mode==HEADING_DYNAMIC && route_policy.use_start_turn_kp)
        yaw_config.kp=HEADING_START_TURN_KP;
    wz=HeadingControl_Update(&yaw_request,&yaw_config,dt,&integral);
    chassis_heading_diagnostics.mode=yaw_request.mode;
    chassis_heading_diagnostics.requested_rad_s=wz;
    chassis_heading_diagnostics.measured_rad_s=gyro_rad_s;
    bool tracking_translation=fabsf(vx)>0.01f || fabsf(vy)>0.01f ||
                     fabsf(state.velocity[0])>0.01f || fabsf(state.velocity[1])>0.01f;
    chassis_heading_diagnostics.tolerance_deg=tracking_translation ? HEADING_MOVING_TOLERANCE_DEG
                                                         : HEADING_STATIC_TOLERANCE_DEG;
    chassis_heading_diagnostics.within_tolerance=
        yaw_request.mode!=HEADING_OFF && yaw_request.mode!=HEADING_MANUAL &&
        isfinite(yaw_request.error) &&
        fabsf(yaw_request.error)<chassis_heading_diagnostics.tolerance_deg*RAD;
    if (stair_hold) {
        wz=body_output[2]+clamp(wz-body_output[2],HEADING_FINE_SLEW*dt);
        if (fabsf(state.yaw_error)<STAIR_HEADING_STOP_TOLERANCE_DEG*RAD &&
            fabsf(gyro_rad_s)<HEADING_HOLD_STOP_RATE) wz=0;
    }
    ForwardCompResult forward_comp =
        ForwardComp_Apply(vx, vy, lateral_direction, chassis_config.left_gain,
                          chassis_config.right_gain,
                          (path_heading_active() && route_policy.suppress_lateral_comp)
                              ? 0.0f : chassis_config.forward_lateral_comp);
    if (!blending_this_cycle) vy = forward_comp.vy_final;
    if (!config_valid())
    {
        Chassis_Stop();
        service_tx();
        send_telemetry(0, 0, 0, 0, 0, now);
        return;
    }
    if (!state.armed || (zero_output && !normal_stopping && !stair_hold))
    {
        memset(body_output, 0, sizeof(body_output));
        vx = vy = wz = 0; /* STOP/fault bypass all smoothing. */
    }
    else if (path_body || normal_stopping)
    {
        float target[3] = {vx, vy, wz};
        /* Stair capture strengthens translation braking only; orbit/yaw stay unchanged. */
        float capture_scale = route_policy.stationary_hold ?
            CHASSIS_STAIR_CAPTURE_BRAKE_SCALE : CHASSIS_CAPTURE_BRAKE_SCALE;
        if (route_policy.stationary_hold && !route_policy.mirror_map_y)
            capture_scale=3.0f;
        Motion_SlewVelocity(body_output, target, dt,
                            (normal_stopping ? BODY_BRAKE_MM_S2 * (capture_braking ? capture_scale : 1.0f) : BODY_ACCEL_MM_S2) * route_policy.travel_speed_scale,
                            normal_stopping ? YAW_BRAKE_RAD_S2 * (capture_braking ? CHASSIS_CAPTURE_BRAKE_SCALE : 1.0f) : YAW_ACCEL_RAD_S2);
        vx = body_output[0]; vy = body_output[1]; wz = body_output[2];
        if (normal_stopping && vx == 0 && vy == 0 && wz == 0) normal_stopping = false;
    }
    else
    {
        /* Preserve the planner's distance braking envelope; smooth yaw only. */
        wz = body_output[2] + clamp(wz - body_output[2],
            (path_rotation && rotate_map_precision ? HEADING_FINE_SLEW : YAW_ACCEL_RAD_S2) * dt);
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
    Mecanum_Forward(geometry(), out, body_output);
    for (int i = 0; i < 4; i++)
    {
        state.rpm_requested[i] = out[i];
        state.rpm_pending[i] = roundf(state.rpm_requested[i] * 10.0f) * CHASSIS_WHEEL_RPM_QUANTUM;
    }
    float quantized_body[3];
    Mecanum_Forward(geometry(), state.rpm_pending, quantized_body);
    chassis_heading_diagnostics.quantized_rad_s=quantized_body[2];
    pending_valid = true;
    service_tx();
    for (int i = 0; i < 4; ++i)
        if (state.rpm_applied[i] != 0 || state.rpm_requested[i] != 0 ||
            ((tx_busy || tx_part) && state.rpm_inflight[i] != 0))
            moving_tick = HAL_GetTick();
    send_telemetry(vx, vy, wz, forward_comp.forward_comp_vy, forward_comp.vy_original, now);
}
