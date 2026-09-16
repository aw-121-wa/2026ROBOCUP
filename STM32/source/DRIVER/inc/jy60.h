#ifndef JY60_H
#define JY60_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* ============================================================
 * JY60 协议
 * ============================================================ */

#define JY60_FRAME_HEADER 0x55U

#define JY60_FRAME_ACC 0x51U
#define JY60_FRAME_GYRO 0x52U
#define JY60_FRAME_ANGLE 0x53U

#define JY60_FRAME_SIZE 11U

/*
 * DMA Circular Buffer
 *
 * 9600bps 下：
 *
 * 256Byte ≈ 266ms UART 数据
 *
 * 对 2~5ms 控制任务来说余量非常大。
 */
#define JY60_DMA_BUFFER_SIZE 256U

/* ============================================================
 * Freshness
 *
 * JY60 官方输出频率：20Hz
 * 一个周期理论为 50ms。
 * ============================================================ */

#define JY60_GOOD_TIMEOUT_MS 80U
#define JY60_LOST_TIMEOUT_MS 180U

/* ============================================================
 * IMU 信任状态
 * ============================================================ */

typedef enum
{
    JY60_TRUST_LOST = 0,

    JY60_TRUST_DEGRADED,

    JY60_TRUST_GOOD

} JY60_Trust_t;

/* ============================================================
 * JY60 状态
 * ============================================================ */

typedef struct
{
    /* -------------------- 加速度 -------------------- */

    float ax_g;
    float ay_g;
    float az_g;

    /* -------------------- 角速度 -------------------- */

    float gx_dps;
    float gy_dps;
    float gz_dps;

    /* -------------------- 姿态角 -------------------- */

    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    /* Diagnostic only: checksum-valid angle before health rejection. */
    float raw_yaw_deg;
    uint32_t raw_angle_frame_count;

    /* -------------------- DWT 时间戳 -------------------- */

    uint32_t accel_cycle;
    uint32_t gyro_cycle;
    uint32_t angle_cycle;

    /* -------------------- 数据有效标志 -------------------- */

    bool has_accel;
    bool has_gyro;
    bool has_angle;

    /* -------------------- 帧统计 -------------------- */

    uint32_t frame_count;

    uint32_t accel_frame_count;
    uint32_t gyro_frame_count;
    uint32_t angle_frame_count;

    uint32_t checksum_error_count;
    uint32_t sync_drop_count;
    uint32_t unknown_frame_count;

    /* -------------------- DMA 调试 -------------------- */

    uint16_t dma_read_pos;
    uint16_t dma_write_pos;

    /* -------------------- 最近有效帧 -------------------- */

    uint8_t last_frame[JY60_FRAME_SIZE];
    uint8_t last_frame_type;

    /* -------------------- Freshness -------------------- */

    uint32_t gyro_age_ms;
    uint32_t angle_age_ms;

    /* -------------------- Trust -------------------- */

    JY60_Trust_t trust;
    JY60_Trust_t freshness;
    uint8_t confidence;
    uint32_t plausibility_errors;

} JY60_State_t;

/* ============================================================
 * API
 * ============================================================ */

/**
 * @brief 初始化 JY60 Circular DMA
 */
bool JY60_Init(void);

/**
 * @brief
 * 从 DMA Circular Buffer 中提取并解析新数据。
 *
 * 建议每 2~5ms 调用一次。
 *
 * 不 Stop DMA，不 Restart DMA。
 */
void JY60_Process(void);

/**
 * @brief 获取当前 JY60 状态
 */
const JY60_State_t *JY60_GetState(void);

#endif
