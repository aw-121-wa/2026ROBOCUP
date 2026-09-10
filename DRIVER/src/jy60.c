#include "jy60.h"
#include "imu_health.h"
static ImuHealth health;
static uint32_t raw_angle_cycle;

#include "pin_config.h"
#include "bsp_dwt.h"

#include <string.h>

/* ============================================================
 * Private Data
 * ============================================================ */

/*
 * 当前 bring-up 阶段建议暂时关闭 D-Cache。
 *
 * 后面会把所有 DMA buffer 统一放到 DMA memory 层，
 * 再正式解决 Cortex-M7 Cache Coherency。
 */
static uint8_t s_jy60_dma_buffer[JY60_DMA_BUFFER_SIZE] __attribute__((aligned(32)));

/*
 * CPU 已经处理到的 Circular Buffer 位置。
 */
static uint16_t s_dma_read_pos = 0U;

/*
 * JY60 当前状态。
 */
static JY60_State_t s_jy60;

/*
 * LOST 状态恢复保护。
 *
 * 如果 IMU 已经因为超时进入 LOST，
 * 必须重新收到新的 gyro 和 angle，
 * 才允许恢复。
 *
 * 这样同时解决 DWT 32bit 长时间回绕可能造成的
 * stale timestamp 假性变新的问题。
 */
static uint32_t s_lost_gyro_frame_count = 0U;
static uint32_t s_lost_angle_frame_count = 0U;

/* ============================================================
 * Private Functions
 * ============================================================ */

/**
 * @brief Little-Endian uint8[2] -> int16
 */
static int16_t JY60_ReadS16(const uint8_t *data)
{
    uint16_t value;

    value = ((uint16_t)data[0]) | ((uint16_t)data[1] << 8);

    return (int16_t)value;
}

/**
 * @brief 验证 11 字节 JY60 帧 checksum
 *
 * 标准协议：
 *
 * checksum =
 *     frame[0] + frame[1] + ... + frame[9]
 *
 * 只保留最低 8 bit。
 */
static bool JY60_CheckFrame(const uint8_t *frame)
{
    uint8_t sum = 0U;

    for (uint32_t i = 0U; i < 10U; i++)
    {
        sum += frame[i];
    }

    return (sum == frame[10]);
}

/**
 * @brief 获取 DMA 当前写入位置
 *
 * DMA Circular:
 *
 * write_pos =
 *      BufferSize - NDTR
 */
static uint16_t JY60_GetDmaWritePos(void)
{
    DMA_HandleTypeDef *hdma;

    hdma = PINCFG_JY60_UART->hdmarx;

    if (hdma == NULL)
    {
        return 0U;
    }

    uint16_t remain = (uint16_t)__HAL_DMA_GET_COUNTER(hdma);

    uint16_t pos = (uint16_t)(JY60_DMA_BUFFER_SIZE - remain);

    /*
     * 在 DMA reload 的极短瞬间可能得到 BufferSize。
     */
    if (pos >= JY60_DMA_BUFFER_SIZE)
    {
        pos = 0U;
    }

    return pos;
}

/**
 * @brief Circular Buffer 两位置之间有多少未处理字节
 */
static uint16_t JY60_RingAvailable(uint16_t read_pos, uint16_t write_pos)
{
    if (write_pos >= read_pos)
    {
        return (uint16_t)(write_pos - read_pos);
    }

    return (uint16_t)(JY60_DMA_BUFFER_SIZE - read_pos + write_pos);
}

/**
 * @brief Circular Buffer Peek
 */
static uint8_t JY60_RingPeek(uint16_t pos, uint16_t offset)
{
    uint16_t index;

    index = (uint16_t)(pos + offset);

    if (index >= JY60_DMA_BUFFER_SIZE)
    {
        index = (uint16_t)(index % JY60_DMA_BUFFER_SIZE);
    }

    return s_jy60_dma_buffer[index];
}

/**
 * @brief Circular Buffer read position 前进
 */
static uint16_t JY60_RingAdvance(uint16_t pos, uint16_t count)
{
    pos = (uint16_t)(pos + count);

    if (pos >= JY60_DMA_BUFFER_SIZE)
    {
        pos = (uint16_t)(pos % JY60_DMA_BUFFER_SIZE);
    }

    return pos;
}

/**
 * @brief 从 Ring Buffer 复制一个完整 JY60 frame
 */
static void JY60_CopyFrame(uint16_t pos, uint8_t *frame)
{
    for (uint32_t i = 0U; i < JY60_FRAME_SIZE; i++)
    {
        frame[i] = JY60_RingPeek(pos, (uint16_t)i);
    }
}

/* ============================================================
 * Frame Parser
 * ============================================================ */

/**
 * @brief 解析 0x51 加速度帧
 */
static void JY60_ParseAccel(const uint8_t *frame, uint32_t now)
{
    int16_t raw_x = JY60_ReadS16(&frame[2]);
    int16_t raw_y = JY60_ReadS16(&frame[4]);
    int16_t raw_z = JY60_ReadS16(&frame[6]);

    /*
     * JY60 acceleration range:
     *
     * -32768 ... +32767
     *       ↓
     *     ±16 g
     */
    const float scale = 16.0f / 32768.0f;

    s_jy60.ax_g = (float)raw_x * scale;

    s_jy60.ay_g = (float)raw_y * scale;

    s_jy60.az_g = (float)raw_z * scale;

    s_jy60.accel_cycle = now;

    s_jy60.has_accel = true;

    s_jy60.accel_frame_count++;
}

/**
 * @brief 解析 0x52 角速度帧
 */
static void JY60_ParseGyro(const uint8_t *frame, uint32_t now)
{
    int16_t raw_x = JY60_ReadS16(&frame[2]);
    int16_t raw_y = JY60_ReadS16(&frame[4]);
    int16_t raw_z = JY60_ReadS16(&frame[6]);

    /*
     * JY60 gyro:
     *
     * -32768 ... +32767
     *       ↓
     *    ±2000 deg/s
     */
    const float scale = 2000.0f / 32768.0f;

    s_jy60.gx_dps = (float)raw_x * scale;

    s_jy60.gy_dps = (float)raw_y * scale;

    s_jy60.gz_dps = (float)raw_z * scale;

    s_jy60.gyro_cycle = now;

    s_jy60.has_gyro = true;

    s_jy60.gyro_frame_count++;
}

/**
 * @brief 解析 0x53 姿态角
 */
static void JY60_ParseAngle(const uint8_t *frame, uint32_t now)
{
    int16_t raw_roll = JY60_ReadS16(&frame[2]);
    int16_t raw_pitch = JY60_ReadS16(&frame[4]);
    int16_t raw_yaw = JY60_ReadS16(&frame[6]);

    /*
     * Angle:
     *
     * -32768 ... +32767
     *       ↓
     *     ±180 deg
     */
    const float scale = 180.0f / 32768.0f;

    float interval = DWT_DeltaSec(now, raw_angle_cycle);
    raw_angle_cycle = now;
    if (!ImuHealth_Angle(&health, (float)raw_yaw * scale, s_jy60.gz_dps, interval))
        return;

    s_jy60.roll_deg = (float)raw_roll * scale;

    s_jy60.pitch_deg = (float)raw_pitch * scale;

    s_jy60.yaw_deg = (float)raw_yaw * scale;

    s_jy60.angle_cycle = now;

    s_jy60.has_angle = true;

    s_jy60.angle_frame_count++;
}

/**
 * @brief 处理一个已经 checksum 正确的 frame
 */
static void JY60_ParseFrame(const uint8_t *frame)
{
    uint32_t now = BSP_DWT_GetCycle();

    /*
     * 保存原始 frame。
     *
     * 调试时非常有价值。
     */
    memcpy(s_jy60.last_frame, frame, JY60_FRAME_SIZE);

    s_jy60.last_frame_type = frame[1];

    switch (frame[1])
    {
    case JY60_FRAME_ACC:

        JY60_ParseAccel(frame, now);

        break;

    case JY60_FRAME_GYRO:

        JY60_ParseGyro(frame, now);

        break;

    case JY60_FRAME_ANGLE:

        JY60_ParseAngle(frame, now);

        break;

    default:

        s_jy60.unknown_frame_count++;

        break;
    }

    s_jy60.frame_count++;
}

/* ============================================================
 * Trust / Freshness
 * ============================================================ */

static void JY60_EnterLost(void)
{
    /*
     * 只在第一次进入 LOST 时锁存。
     */
    if (s_jy60.trust != JY60_TRUST_LOST)
    {
        s_lost_gyro_frame_count = s_jy60.gyro_frame_count;

        s_lost_angle_frame_count = s_jy60.angle_frame_count;
    }

    s_jy60.trust = JY60_TRUST_LOST;
}

static void JY60_UpdateTrust(uint32_t now)
{
    /*
     * 没收到基本数据。
     */
    if ((!s_jy60.has_gyro) || (!s_jy60.has_angle))
    {
        JY60_EnterLost();

        return;
    }

    uint32_t gyro_cycles = BSP_DWT_ElapsedCycles(now, s_jy60.gyro_cycle);

    uint32_t angle_cycles = BSP_DWT_ElapsedCycles(now, s_jy60.angle_cycle);

    s_jy60.gyro_age_ms = BSP_DWT_CyclesToMs(gyro_cycles);

    s_jy60.angle_age_ms = BSP_DWT_CyclesToMs(angle_cycles);

    /*
     * 如果已经 LOST：
     *
     * 必须 gyro 和 angle 都真正重新收到新帧，
     * 才允许恢复。
     *
     * 不能因为 DWT 回绕导致旧时间戳又看起来很新。
     */
    if (s_jy60.trust == JY60_TRUST_LOST)
    {
        bool new_gyro = s_jy60.gyro_frame_count > s_lost_gyro_frame_count;

        bool new_angle = s_jy60.angle_frame_count > s_lost_angle_frame_count;

        if ((!new_gyro) || (!new_angle))
        {
            return;
        }
    }

    /*
     * GOOD
     */
    if ((s_jy60.gyro_age_ms <= JY60_GOOD_TIMEOUT_MS) &&
        (s_jy60.angle_age_ms <= JY60_GOOD_TIMEOUT_MS))
    {
        s_jy60.trust = JY60_TRUST_GOOD;

        return;
    }

    /*
     * DEGRADED
     */
    if ((s_jy60.gyro_age_ms <= JY60_LOST_TIMEOUT_MS) &&
        (s_jy60.angle_age_ms <= JY60_LOST_TIMEOUT_MS))
    {
        s_jy60.trust = JY60_TRUST_DEGRADED;

        return;
    }

    /*
     * LOST
     */
    JY60_EnterLost();
}

/* ============================================================
 * Public API
 * ============================================================ */

bool JY60_Init(void)
{
    memset(&health, 0, sizeof(health));
    raw_angle_cycle = 0;
    /*
     * 清零所有状态。
     */
    memset(&s_jy60, 0, sizeof(s_jy60));

    memset(s_jy60_dma_buffer, 0, sizeof(s_jy60_dma_buffer));

    s_dma_read_pos = 0U;

    s_jy60.trust = JY60_TRUST_LOST;

    s_lost_gyro_frame_count = 0U;
    s_lost_angle_frame_count = 0U;

    /*
     * 基本硬件检查。
     */

    if (PINCFG_JY60_UART->hdmarx == NULL)
    {
        return false;
    }

    /*
     * 启动 Circular DMA。
     *
     * 注意：
     *
     * 之后正常工作过程中不再 Stop / Restart。
     */
    HAL_StatusTypeDef status;

    status = HAL_UART_Receive_DMA(PINCFG_JY60_UART, s_jy60_dma_buffer, JY60_DMA_BUFFER_SIZE);

    if (status != HAL_OK)
    {
        return false;
    }

    return true;
}

void JY60_Process(void)
{
    /*
     * DMA 当前写位置。
     *
     * 我们只处理 write_pos 之前已经完整进入 RAM 的数据。
     */
    uint16_t write_pos = JY60_GetDmaWritePos();

    s_jy60.dma_write_pos = write_pos;

    while (1)
    {
        uint16_t available = JY60_RingAvailable(s_dma_read_pos, write_pos);

        /*
         * 连 header + type 都不够。
         */
        if (available < 2U)
        {
            break;
        }

        /*
         * 寻找 0x55 帧头。
         */
        if (JY60_RingPeek(s_dma_read_pos, 0U) != JY60_FRAME_HEADER)
        {
            s_dma_read_pos = JY60_RingAdvance(s_dma_read_pos, 1U);

            s_jy60.sync_drop_count++;

            continue;
        }

        /*
         * 找到了 0x55，
         * 但是完整 11Byte 还没到。
         *
         * 留到下一个 Process 再处理。
         */
        if (available < JY60_FRAME_SIZE)
        {
            break;
        }

        uint8_t frame[JY60_FRAME_SIZE];

        JY60_CopyFrame(s_dma_read_pos, frame);

        /*
         * checksum 不正确。
         *
         * 不直接丢掉 11Byte。
         *
         * 只丢 1Byte，再重新寻找 0x55，
         * 防止一次错位导致后面全部错位。
         */
        if (!JY60_CheckFrame(frame))
        {
            s_jy60.checksum_error_count++;
            ImuHealth_ChecksumError(&health);

            s_dma_read_pos = JY60_RingAdvance(s_dma_read_pos, 1U);

            continue;
        }

        /*
         * 合法 frame。
         */
        JY60_ParseFrame(frame);

        /*
         * 一个完整合法 frame 已消费。
         */
        s_dma_read_pos = JY60_RingAdvance(s_dma_read_pos, JY60_FRAME_SIZE);
    }

    s_jy60.dma_read_pos = s_dma_read_pos;

    /*
     * 即使本周期没有新数据，
     * 也必须更新 freshness。
     */
    s_jy60.trust = s_jy60.freshness;
    JY60_UpdateTrust(BSP_DWT_GetCycle());
    s_jy60.freshness = s_jy60.trust;
    s_jy60.confidence = (uint8_t)ImuHealth_Confidence(&health);
    s_jy60.plausibility_errors = health.rejected;
    if (s_jy60.confidence < (unsigned)s_jy60.trust)
        s_jy60.trust = (JY60_Trust_t)s_jy60.confidence;
}

const JY60_State_t *JY60_GetState(void)
{
    return &s_jy60;
}
