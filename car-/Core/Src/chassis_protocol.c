/**
  ******************************************************************************
  * @file    chassis_protocol.c
  * @brief   底盘串口帧协议与状态回传。
  *
  * 帧格式（小端）：
  *   AA 55 | LEN | SEQ | TYPE | PAYLOAD[LEN] | CRC16_L | CRC16_H
  *   CRC16/MODBUS，计算范围：LEN、SEQ、TYPE、PAYLOAD。
  *
  * 上位机命令：
  *   0x01 速度命令：i16 线速度(mm/s), i16 角速度(mrad/s), u16 超时(ms)
  *   0x02 四轮转速：i16 rpm_x10[4]
  *   0x03 PID 参数：u8 轮号(0~3 或 0xFF), u8 保留,
  *                  i16 kp_x1000, i16 ki_x1000, i16 kd_x1000
  *   0x04 使能：u8 enable, u8 clear_faults
  *   0x05 心跳：无载荷，立即回一帧状态
  *   0x06 停止：无载荷，退出使能
  *
  * 下位机状态：0x81，载荷 44 字节，详见 chassis_protocol_send_status()。
  ******************************************************************************
  */
#include "chassis_protocol.h"
#include "chassis_config.h"
#include "chassis_vars.h"
#include "nav_vars.h"
#include "motion_control.h"
#include "param_store.h"
#include "tof400.h"
#include "usart.h"
#include "main.h"

/*---------------------------------------------------------------------------*/
/* 协议常量                                                                    */
/*---------------------------------------------------------------------------*/
#define PROTO_HEADER_LEN        5U
#define PROTO_CRC_LEN           2U
#define PROTO_STATUS_LEN        44U
#define PROTO_NAV_LEN           30U
#define PROTO_MOTION_LEN        28U
#define PROTO_PARAM_LEN         8U
#define PROTO_CALIB_LEN         16U
#define PROTO_TX_MAX_LEN        (PROTO_HEADER_LEN + PROTO_STATUS_LEN + PROTO_CRC_LEN)
#define PROTO_FRAME_BUF_LEN     (3U + CHASSIS_FRAME_MAX_PAYLOAD)

#define CMD_SET_VELOCITY        0x01U
#define CMD_SET_WHEEL_RPM       0x02U
#define CMD_SET_PID             0x03U
#define CMD_SET_ENABLE          0x04U
#define CMD_PING                0x05U
#define CMD_STOP                0x06U
#define CMD_SET_NAV_MODE        0x10U
#define CMD_LINE                0x11U
#define CMD_TURN                0x12U
#define CMD_ARC                 0x13U
#define CMD_NAV_STOP            0x14U
#define CMD_SET_SPEED           0x15U
#define CMD_PATH_BEGIN          0x16U
#define CMD_PATH_COMMIT         0x17U
#define CMD_PATH_ABORT          0x18U
#define CMD_CALIB               0x19U
#define CMD_PARAM               0x1AU

#define RSP_STATUS              0x81U
#define RSP_NAV                 0x82U
#define RSP_NAV_CONTROL         0x83U
#define RSP_PARAM               0x84U
#define RSP_CALIB               0x85U

/*---------------------------------------------------------------------------*/
/* 接收环形缓冲（单生产者：UART 中断；单消费者：主循环）                        */
/*---------------------------------------------------------------------------*/
static volatile uint8_t s_rx_ring[CHASSIS_UART_RX_RING_SIZE];
static volatile uint16_t s_rx_head;
static volatile uint16_t s_rx_tail;
static uint8_t s_rx_byte;

/*---------------------------------------------------------------------------*/
/* 解析器状态                                                                  */
/*---------------------------------------------------------------------------*/
typedef enum
{
    PARSER_WAIT_SOF1 = 0,
    PARSER_WAIT_SOF2,
    PARSER_WAIT_LEN,
    PARSER_WAIT_SEQ,
    PARSER_WAIT_TYPE,
    PARSER_WAIT_PAYLOAD,
    PARSER_WAIT_CRC_L,
    PARSER_WAIT_CRC_H
} parser_state_t;

static parser_state_t s_parser_state;
static uint8_t s_frame[PROTO_FRAME_BUF_LEN]; /* [0]=LEN [1]=SEQ [2]=TYPE [3..]=PAYLOAD */
static uint8_t s_len;
static uint8_t s_payload_index;
static uint8_t s_crc_low;
static uint8_t s_crc_high;

static uint8_t s_tx_seq;
static uint8_t s_tx_frame[PROTO_TX_MAX_LEN];
static uint32_t s_last_telemetry_ms;
static uint32_t s_last_nav_telemetry_ms;
static uint32_t s_last_motion_telemetry_ms;
static volatile uint32_t s_last_rx_byte_ms;
static int8_t s_last_cmd_status;

/*---------------------------------------------------------------------------*/
/* 小工具                                                                      */
/*---------------------------------------------------------------------------*/
static uint16_t proto_rd_u16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static int16_t proto_rd_i16(const uint8_t *p)
{
    return (int16_t)proto_rd_u16(p);
}

static uint32_t proto_rd_u32(const uint8_t *p)
{
    return ((uint32_t)p[0]) | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void proto_wr_u16(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)(value & 0xFFU);
    p[1] = (uint8_t)((value >> 8) & 0xFFU);
}

static void proto_wr_i16(uint8_t *p, int16_t value)
{
    proto_wr_u16(p, (uint16_t)value);
}

static void proto_wr_u32(uint8_t *p, uint32_t value)
{
    p[0] = (uint8_t)(value & 0xFFU);
    p[1] = (uint8_t)((value >> 8) & 0xFFU);
    p[2] = (uint8_t)((value >> 16) & 0xFFU);
    p[3] = (uint8_t)((value >> 24) & 0xFFU);
}

static int16_t proto_float_to_i16(float value)
{
    if (value != value)
    {
        return 0;
    }
    if (value > 32766.5f)
    {
        return 32767;
    }
    if (value < -32767.5f)
    {
        return -32768;
    }
    return (int16_t)((value >= 0.0f) ? (value + 0.5f) : (value - 0.5f));
}

/* 直接轮询 USART1 发送，不占用 HAL 的 huart1->Lock。
 * 这样 RX 完成中断里的 HAL_UART_Receive_IT() 不会因为主循环正在发送而
 * 返回 HAL_BUSY，避免状态回传期间丢命令字节。 */
static bool proto_uart_send(const uint8_t *data, uint16_t length)
{
    uint32_t start_ms = HAL_GetTick();
    uint16_t i;

    for (i = 0U; i < length; i++)
    {
        while ((USART1->SR & USART_SR_TXE) == 0U)
        {
            if ((uint32_t)(HAL_GetTick() - start_ms) > 20U)
            {
                return false;
            }
        }
        USART1->DR = data[i];
    }

    while ((USART1->SR & USART_SR_TC) == 0U)
    {
        if ((uint32_t)(HAL_GetTick() - start_ms) > 25U)
        {
            return false;
        }
    }

    return true;
}

static uint16_t proto_float_to_u16(float value)
{
    if (value != value)
    {
        return 0U;
    }
    if (value <= 0.0f)
    {
        return 0U;
    }
    if (value >= 65535.0f)
    {
        return 65535U;
    }
    return (uint16_t)(value + 0.5f);
}

static uint16_t proto_crc16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;
    uint16_t i;
    uint8_t bit;

    for (i = 0U; i < length; i++)
    {
        crc ^= (uint16_t)data[i];
        for (bit = 0U; bit < 8U; bit++)
        {
            if ((crc & 1U) != 0U)
            {
                crc = (uint16_t)((crc >> 1) ^ 0xA001U);
            }
            else
            {
                crc >>= 1;
            }
        }
    }

    return crc;
}

/*---------------------------------------------------------------------------*/
/* 状态回传                                                                    */
/*---------------------------------------------------------------------------*/
static void chassis_protocol_send_status(uint32_t now_ms)
{
    chassis_snapshot_t snapshot;
    uint16_t flags;
    uint16_t age_ms;
    uint16_t crc;
    uint8_t *p;
    uint8_t i;

    get_chassis_snapshot(&snapshot);

    flags = (uint16_t)((snapshot.enabled ? 1U : 0U) |
                       ((uint16_t)snapshot.mode << 1) |
                       ((uint16_t)CHASSIS_PROTOCOL_VERSION << 8));

    age_ms = (uint16_t)(now_ms - snapshot.last_command_ms);
    if (age_ms > 32767U)
    {
        age_ms = 32767U;
    }

    s_tx_frame[0] = CHASSIS_FRAME_SOF1;
    s_tx_frame[1] = CHASSIS_FRAME_SOF2;
    s_tx_frame[2] = PROTO_STATUS_LEN;
    s_tx_frame[3] = s_tx_seq++;
    s_tx_frame[4] = RSP_STATUS;

    p = &s_tx_frame[PROTO_HEADER_LEN];

    proto_wr_u16(&p[0], snapshot.faults);
    proto_wr_u16(&p[2], flags);
    proto_wr_i16(&p[4], proto_float_to_i16(snapshot.linear_mps * 1000.0f));
    proto_wr_i16(&p[6], proto_float_to_i16(snapshot.angular_radps * 1000.0f));
    proto_wr_i16(&p[8], (int16_t)age_ms);

    for (i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
        proto_wr_i16(&p[10U + (i * 2U)],
                     proto_float_to_i16(snapshot.target_rpm[i] * 10.0f));
    }
    for (i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
        proto_wr_i16(&p[18U + (i * 2U)],
                     proto_float_to_i16(snapshot.measured_rpm[i] * 10.0f));
    }
    for (i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
        proto_wr_i16(&p[26U + (i * 2U)], snapshot.pwm_permille[i]);
    }

    proto_wr_u16(&p[34], (uint16_t)(snapshot.rx_frames_ok & 0xFFFFU));
    proto_wr_u16(&p[36], (uint16_t)(snapshot.rx_frames_bad & 0xFFFFU));
    proto_wr_u32(&p[38], snapshot.control_cycles);
    p[42] = (uint8_t)s_last_cmd_status;
    p[43] = 0U;

    crc = proto_crc16(&s_tx_frame[2], (uint16_t)(3U + PROTO_STATUS_LEN));
    s_tx_frame[PROTO_HEADER_LEN + PROTO_STATUS_LEN] = (uint8_t)(crc & 0xFFU);
    s_tx_frame[PROTO_HEADER_LEN + PROTO_STATUS_LEN + 1U] = (uint8_t)((crc >> 8) & 0xFFU);

    (void)proto_uart_send(s_tx_frame, PROTO_TX_MAX_LEN);
}

static void chassis_protocol_send_nav(uint32_t now_ms)
{
    nav_snapshot_t nav;
    uint16_t flags = 0U;
    uint16_t crc;
    uint8_t *p;

    (void)now_ms;
    get_nav_snapshot(&nav);

    if (nav.ekf_ok) flags |= (1U << 0);
    if (nav.imu_ok) flags |= (1U << 1);
    if (nav.tof_valid[0]) flags |= (1U << 2);
    if (nav.tof_valid[1]) flags |= (1U << 3);
    if (nav.tof_obstacle[0]) flags |= (1U << 4);
    if (nav.tof_obstacle[1]) flags |= (1U << 5);
    if (nav.mode == 1U) flags |= (1U << 6);

    s_tx_frame[0] = CHASSIS_FRAME_SOF1;
    s_tx_frame[1] = CHASSIS_FRAME_SOF2;
    s_tx_frame[2] = PROTO_NAV_LEN;
    s_tx_frame[3] = s_tx_seq++;
    s_tx_frame[4] = RSP_NAV;
    p = &s_tx_frame[PROTO_HEADER_LEN];

    proto_wr_u16(&p[0], flags);
    proto_wr_i16(&p[2], proto_float_to_i16(nav.x_m * 1000.0f));
    proto_wr_i16(&p[4], proto_float_to_i16(nav.y_m * 1000.0f));
    proto_wr_i16(&p[6], proto_float_to_i16(nav.theta_rad * 1000.0f));
    proto_wr_i16(&p[8], proto_float_to_i16(nav.v_mps * 1000.0f));
    proto_wr_i16(&p[10], proto_float_to_i16(nav.omega_radps * 1000.0f));
    proto_wr_i16(&p[12], proto_float_to_i16(nav.roll_rad * 1000.0f));
    proto_wr_i16(&p[14], proto_float_to_i16(nav.pitch_rad * 1000.0f));
    proto_wr_u16(&p[16], nav.tof_mm[0]);
    proto_wr_u16(&p[18], nav.tof_mm[1]);
    proto_wr_u16(&p[20], proto_float_to_u16(nav.p_xx * 10000.0f));
    proto_wr_u16(&p[22], proto_float_to_u16(nav.p_yy * 10000.0f));
    proto_wr_u16(&p[24], proto_float_to_u16(nav.p_tt * 1000000.0f));
    proto_wr_u16(&p[26], (uint16_t)(nav.tof_updates[0] & 0xFFFFU));
    proto_wr_u16(&p[28], (uint16_t)(nav.tof_updates[1] & 0xFFFFU));

    crc = proto_crc16(&s_tx_frame[2], (uint16_t)(3U + PROTO_NAV_LEN));
    s_tx_frame[PROTO_HEADER_LEN + PROTO_NAV_LEN] = (uint8_t)(crc & 0xFFU);
    s_tx_frame[PROTO_HEADER_LEN + PROTO_NAV_LEN + 1U] =
        (uint8_t)((crc >> 8) & 0xFFU);

    (void)proto_uart_send(s_tx_frame,
                          (uint16_t)(PROTO_HEADER_LEN + PROTO_NAV_LEN + PROTO_CRC_LEN));
}

static void chassis_protocol_send_motion(uint32_t now_ms)
{
    motion_status_t motion;
    uint8_t flags = 0U;
    uint16_t crc;
    uint8_t *p;

    (void)now_ms;
    motion_get_status(&motion);
    if (get_nav_mode() == 1U) flags |= 0x01U;
    if (motion.state == MOTION_RUNNING) flags |= 0x02U;
    if (motion.state == MOTION_ERROR) flags |= 0x04U;

    s_tx_frame[0] = CHASSIS_FRAME_SOF1;
    s_tx_frame[1] = CHASSIS_FRAME_SOF2;
    s_tx_frame[2] = PROTO_MOTION_LEN;
    s_tx_frame[3] = s_tx_seq++;
    s_tx_frame[4] = RSP_NAV_CONTROL;
    p = &s_tx_frame[PROTO_HEADER_LEN];

    p[0] = motion.state;
    p[1] = motion.active_segment;
    p[2] = motion.total_segments;
    p[3] = motion.completed_segments;
    p[4] = motion.building;
    p[5] = motion.last_error;
    p[6] = flags;
    p[7] = 0U;
    proto_wr_i16(&p[8], proto_float_to_i16(motion.target_x_m * 1000.0f));
    proto_wr_i16(&p[10], proto_float_to_i16(motion.target_y_m * 1000.0f));
    proto_wr_i16(&p[12], proto_float_to_i16(motion.cross_track_m * 1000.0f));
    proto_wr_i16(&p[14], proto_float_to_i16(motion.heading_error_rad * 1000.0f));
    proto_wr_i16(&p[16], proto_float_to_i16(motion.command_v_mps * 1000.0f));
    proto_wr_i16(&p[18], proto_float_to_i16(motion.command_w_radps * 1000.0f));
    proto_wr_u16(&p[20], proto_float_to_u16(motion.max_speed_mps * 1000.0f));
    proto_wr_u16(&p[22], proto_float_to_u16(motion.max_omega_radps * 1000.0f));
    proto_wr_u16(&p[24], 0U);
    proto_wr_u16(&p[26], 0U);

    crc = proto_crc16(&s_tx_frame[2], (uint16_t)(3U + PROTO_MOTION_LEN));
    s_tx_frame[PROTO_HEADER_LEN + PROTO_MOTION_LEN] = (uint8_t)(crc & 0xFFU);
    s_tx_frame[PROTO_HEADER_LEN + PROTO_MOTION_LEN + 1U] =
        (uint8_t)((crc >> 8) & 0xFFU);
    (void)proto_uart_send(s_tx_frame,
                          (uint16_t)(PROTO_HEADER_LEN + PROTO_MOTION_LEN + PROTO_CRC_LEN));
}

static void chassis_protocol_send_param(uint8_t param_id, uint8_t status, int32_t value)
{
    uint8_t *p = &s_tx_frame[PROTO_HEADER_LEN];
    uint16_t crc;

    s_tx_frame[0] = CHASSIS_FRAME_SOF1;
    s_tx_frame[1] = CHASSIS_FRAME_SOF2;
    s_tx_frame[2] = PROTO_PARAM_LEN;
    s_tx_frame[3] = s_tx_seq++;
    s_tx_frame[4] = RSP_PARAM;
    p[0] = param_id;
    p[1] = status;
    proto_wr_u32(&p[2], (uint32_t)value);
    p[6] = 0U;
    p[7] = 0U;
    crc = proto_crc16(&s_tx_frame[2], (uint16_t)(3U + PROTO_PARAM_LEN));
    s_tx_frame[PROTO_HEADER_LEN + PROTO_PARAM_LEN] = (uint8_t)(crc & 0xFFU);
    s_tx_frame[PROTO_HEADER_LEN + PROTO_PARAM_LEN + 1U] =
        (uint8_t)((crc >> 8) & 0xFFU);
    (void)proto_uart_send(s_tx_frame,
                          (uint16_t)(PROTO_HEADER_LEN + PROTO_PARAM_LEN + PROTO_CRC_LEN));
}

static void chassis_protocol_send_calib(uint8_t action, uint8_t sensor,
                                         uint8_t count, uint8_t status,
                                         uint16_t known_mm, uint16_t raw_mm,
                                         float gain, float offset)
{
    uint8_t *p = &s_tx_frame[PROTO_HEADER_LEN];
    uint16_t crc;

    s_tx_frame[0] = CHASSIS_FRAME_SOF1;
    s_tx_frame[1] = CHASSIS_FRAME_SOF2;
    s_tx_frame[2] = PROTO_CALIB_LEN;
    s_tx_frame[3] = s_tx_seq++;
    s_tx_frame[4] = RSP_CALIB;
    p[0] = action;
    p[1] = sensor;
    p[2] = count;
    p[3] = status;
    proto_wr_u16(&p[4], known_mm);
    proto_wr_u16(&p[6], raw_mm);
    proto_wr_i16(&p[8], proto_float_to_i16(gain * 1000.0f));
    proto_wr_i16(&p[10], proto_float_to_i16(offset));
    proto_wr_u16(&p[12], 0U);
    proto_wr_u16(&p[14], 0U);
    crc = proto_crc16(&s_tx_frame[2], (uint16_t)(3U + PROTO_CALIB_LEN));
    s_tx_frame[PROTO_HEADER_LEN + PROTO_CALIB_LEN] = (uint8_t)(crc & 0xFFU);
    s_tx_frame[PROTO_HEADER_LEN + PROTO_CALIB_LEN + 1U] =
        (uint8_t)((crc >> 8) & 0xFFU);
    (void)proto_uart_send(s_tx_frame,
                          (uint16_t)(PROTO_HEADER_LEN + PROTO_CALIB_LEN + PROTO_CRC_LEN));
}

/*---------------------------------------------------------------------------*/
/* 命令处理                                                                    */
/*---------------------------------------------------------------------------*/
static var_status_t protocol_handle_frame(uint8_t type,
                                          const uint8_t *payload,
                                          uint8_t length,
                                          uint32_t now_ms)
{
    float rpm[CHASSIS_WHEEL_COUNT];
    float kp;
    float ki;
    float kd;
    uint16_t timeout_ms;
    uint8_t wheel;
    uint8_t enable;
    uint8_t clear_faults;
    uint8_t i;
    var_status_t status = VAR_OK;

    switch (type)
    {
    case CMD_SET_VELOCITY:
        if (length != 6U)
        {
            return VAR_ERR_ARG;
        }
        timeout_ms = proto_rd_u16(&payload[4]);
        if (timeout_ms == 0U)
        {
            timeout_ms = CHASSIS_COMMAND_TIMEOUT_DEFAULT_MS;
        }
        /* 手动速度优先：收到摇杆速度时终止正在执行的路径。 */
        motion_abort(now_ms);
        status = give_motion_command((float)proto_rd_i16(&payload[0]) / 1000.0f,
                                     (float)proto_rd_i16(&payload[2]) / 1000.0f,
                                     timeout_ms,
                                     now_ms);
        break;

    case CMD_SET_WHEEL_RPM:
        if (length != 8U)
        {
            return VAR_ERR_ARG;
        }
        for (i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
        {
            rpm[i] = (float)proto_rd_i16(&payload[i * 2U]) / 10.0f;
        }
        motion_abort(now_ms);
        status = give_wheel_command(rpm, now_ms);
        break;

    case CMD_SET_PID:
        if (length != 8U)
        {
            return VAR_ERR_ARG;
        }
        wheel = payload[0];
        kp = (float)proto_rd_i16(&payload[2]) / 1000.0f;
        ki = (float)proto_rd_i16(&payload[4]) / 1000.0f;
        kd = (float)proto_rd_i16(&payload[6]) / 1000.0f;

        if (wheel == 0xFFU)
        {
            for (i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
            {
                status = give_pid_gains(i, kp, ki, kd);
                if (status != VAR_OK)
                {
                    break;
                }
            }
        }
        else
        {
            status = give_pid_gains(wheel, kp, ki, kd);
        }
        break;

    case CMD_SET_ENABLE:
        if (length != 2U)
        {
            return VAR_ERR_ARG;
        }
        enable = payload[0];
        clear_faults = payload[1];
        if ((enable > 1U) || (clear_faults > 1U))
        {
            return VAR_ERR_RANGE;
        }
        if (enable == 0U)
        {
            motion_abort(now_ms);
        }
        status = give_enable((enable != 0U),
                             (clear_faults != 0U),
                             now_ms);
        break;

    case CMD_PING:
        if (length != 0U)
        {
            return VAR_ERR_ARG;
        }
        chassis_protocol_send_status(now_ms);
        status = VAR_OK;
        break;

    case CMD_STOP:
        if (length != 0U)
        {
            return VAR_ERR_ARG;
        }
        motion_abort(now_ms);
        status = give_enable(false, false, now_ms);
        break;

    case CMD_SET_NAV_MODE:
        if (length != 1U)
        {
            return VAR_ERR_ARG;
        }
        status = give_nav_mode(payload[0]);
        if (status == VAR_OK)
        {
            (void)param_store_set_value(PARAM_ID_NAV_MODE, (int32_t)payload[0]);
        }
        break;

    case CMD_LINE:
        if (length != 8U)
        {
            return VAR_ERR_ARG;
        }
        if (!motion_line_to((float)proto_rd_i16(&payload[0]) / 1000.0f,
                            (float)proto_rd_i16(&payload[2]) / 1000.0f,
                            (float)proto_rd_u16(&payload[4]) / 1000.0f))
        {
            status = VAR_ERR_RANGE;
        }
        break;

    case CMD_TURN:
        if (length != 4U)
        {
            return VAR_ERR_ARG;
        }
        if (!motion_turn_to((float)proto_rd_i16(&payload[0]) / 1000.0f,
                            (float)proto_rd_i16(&payload[2]) / 1000.0f))
        {
            status = VAR_ERR_RANGE;
        }
        break;

    case CMD_ARC:
        if (length != 8U)
        {
            return VAR_ERR_ARG;
        }
        if (!motion_arc((float)proto_rd_i16(&payload[0]) / 1000.0f,
                        (float)proto_rd_i16(&payload[2]) / 1000.0f,
                        (float)proto_rd_u16(&payload[4]) / 1000.0f))
        {
            status = VAR_ERR_RANGE;
        }
        break;

    case CMD_NAV_STOP:
        if (length != 0U)
        {
            return VAR_ERR_ARG;
        }
        motion_abort(now_ms);
        break;

    case CMD_SET_SPEED:
        if (length != 4U)
        {
            return VAR_ERR_ARG;
        }
        motion_set_limits((float)proto_rd_u16(&payload[0]) / 1000.0f,
                          (float)proto_rd_u16(&payload[2]) / 1000.0f);
        break;

    case CMD_PATH_BEGIN:
        if (length != 2U)
        {
            return VAR_ERR_ARG;
        }
        if (!motion_path_begin(payload[0], now_ms))
        {
            status = VAR_ERR_RANGE;
        }
        break;

    case CMD_PATH_COMMIT:
        if (length != 0U)
        {
            return VAR_ERR_ARG;
        }
        if (!motion_path_commit(now_ms))
        {
            status = VAR_ERR_STATE;
        }
        break;

    case CMD_PATH_ABORT:
        if (length != 0U)
        {
            return VAR_ERR_ARG;
        }
        motion_abort(now_ms);
        break;

    case CMD_CALIB:
    {
        uint8_t action;
        uint8_t sensor;
        uint8_t count = 0U;
        uint8_t report_status = 0U;
        uint16_t known_mm;
        uint16_t raw_mm = 0U;
        float gain = 1.0f;
        float offset = 0.0f;

        if (length != 6U)
        {
            return VAR_ERR_ARG;
        }
        action = payload[0];
        sensor = payload[1];
        known_mm = proto_rd_u16(&payload[2]);
        if (sensor > 1U)
        {
            return VAR_ERR_RANGE;
        }

        (void)tof400_get_last_raw(sensor, &raw_mm);
        (void)tof400_get_calibration(sensor, &gain, &offset);
        count = tof400_calib_count(sensor);

        switch (action)
        {
        case 0U: /* start */
            report_status = tof400_calib_start(sensor) ? 0U : 1U;
            break;
        case 1U: /* sample */
            report_status = tof400_calib_add(sensor, known_mm) ? 0U : 1U;
            break;
        case 2U: /* fit and apply */
            report_status = tof400_calib_fit(sensor, &gain, &offset) ? 0U : 1U;
            break;
        case 3U: /* save */
            report_status = param_store_save() ? 0U : 1U;
            break;
        case 4U: /* load */
            report_status = param_store_load() ? 0U : 1U;
            break;
        case 5U: /* reset */
            tof400_calib_reset(sensor);
            gain = 1.0f;
            offset = 0.0f;
            break;
        default:
            return VAR_ERR_ARG;
        }

        count = tof400_calib_count(sensor);
        (void)tof400_get_last_raw(sensor, &raw_mm);
        (void)tof400_get_calibration(sensor, &gain, &offset);
        chassis_protocol_send_calib(action, sensor, count, report_status,
                                    known_mm, raw_mm, gain, offset);
        status = (report_status == 0U) ? VAR_OK : VAR_ERR_STATE;
        break;
    }

    case CMD_PARAM:
    {
        uint8_t action;
        uint8_t param_id;
        int32_t value = 0;
        uint8_t report_status = 0U;

        if (length != 6U)
        {
            return VAR_ERR_ARG;
        }
        action = payload[0];
        param_id = payload[1];
        value = (int32_t)proto_rd_u32(&payload[2]);

        switch (action)
        {
        case 0U: /* get */
            report_status = param_store_get_value(param_id, &value) ? 0U : 1U;
            break;
        case 1U: /* set */
            report_status = param_store_set_value(param_id, value) ? 0U : 1U;
            break;
        case 2U: /* save */
            report_status = param_store_save() ? 0U : 1U;
            break;
        case 3U: /* load */
            report_status = param_store_load() ? 0U : 1U;
            break;
        case 4U: /* reset defaults */
            report_status = param_store_reset_defaults() ? 0U : 1U;
            break;
        default:
            return VAR_ERR_ARG;
        }

        (void)param_store_get_value(param_id, &value);
        chassis_protocol_send_param(param_id, report_status, value);
        status = (report_status == 0U) ? VAR_OK : VAR_ERR_STATE;
        break;
    }

    default:
        status = VAR_ERR_ARG;
        break;
    }

    return status;
}

/*---------------------------------------------------------------------------*/
/* 帧解析                                                                      */
/*---------------------------------------------------------------------------*/
static void protocol_finish_frame(uint32_t now_ms)
{
    uint16_t crc_received;
    uint16_t crc_calculated;
    var_status_t status;

    crc_received = (uint16_t)((uint16_t)s_crc_low |
                              ((uint16_t)s_crc_high << 8));

    crc_calculated = proto_crc16(s_frame, (uint16_t)(3U + s_len));

    if (crc_received != crc_calculated)
    {
        (void)give_comm_result(false);
        s_parser_state = PARSER_WAIT_SOF1;
        return;
    }

    status = protocol_handle_frame(s_frame[2], &s_frame[3], s_len, now_ms);
    s_last_cmd_status = (int8_t)status;
    (void)give_comm_result(true);

    s_parser_state = PARSER_WAIT_SOF1;
}

static void protocol_feed_byte(uint8_t byte, uint32_t now_ms)
{
    switch (s_parser_state)
    {
    case PARSER_WAIT_SOF1:
        if (byte == CHASSIS_FRAME_SOF1)
        {
            s_parser_state = PARSER_WAIT_SOF2;
        }
        break;

    case PARSER_WAIT_SOF2:
        if (byte == CHASSIS_FRAME_SOF2)
        {
            s_parser_state = PARSER_WAIT_LEN;
        }
        else if (byte != CHASSIS_FRAME_SOF1)
        {
            s_parser_state = PARSER_WAIT_SOF1;
        }
        break;

    case PARSER_WAIT_LEN:
        if (byte > CHASSIS_FRAME_MAX_PAYLOAD)
        {
            (void)give_comm_result(false);
            s_parser_state = PARSER_WAIT_SOF1;
        }
        else
        {
            s_len = byte;
            s_frame[0] = byte;
            s_parser_state = PARSER_WAIT_SEQ;
        }
        break;

    case PARSER_WAIT_SEQ:
        s_frame[1] = byte;
        s_parser_state = PARSER_WAIT_TYPE;
        break;

    case PARSER_WAIT_TYPE:
        s_frame[2] = byte;
        s_payload_index = 0U;
        s_parser_state = (s_len == 0U) ? PARSER_WAIT_CRC_L : PARSER_WAIT_PAYLOAD;
        break;

    case PARSER_WAIT_PAYLOAD:
        s_frame[3U + s_payload_index] = byte;
        s_payload_index++;
        if (s_payload_index >= s_len)
        {
            s_parser_state = PARSER_WAIT_CRC_L;
        }
        break;

    case PARSER_WAIT_CRC_L:
        s_crc_low = byte;
        s_parser_state = PARSER_WAIT_CRC_H;
        break;

    case PARSER_WAIT_CRC_H:
        s_crc_high = byte;
        protocol_finish_frame(now_ms);
        break;

    default:
        s_parser_state = PARSER_WAIT_SOF1;
        break;
    }
}

/*---------------------------------------------------------------------------*/
/* 公共接口                                                                    */
/*---------------------------------------------------------------------------*/
void chassis_protocol_init(void)
{
    s_rx_head = 0U;
    s_rx_tail = 0U;
    s_parser_state = PARSER_WAIT_SOF1;
    s_len = 0U;
    s_payload_index = 0U;
    s_crc_low = 0U;
    s_crc_high = 0U;
    s_tx_seq = 0U;
    s_last_cmd_status = (int8_t)VAR_OK;
    s_last_telemetry_ms = HAL_GetTick();
    s_last_nav_telemetry_ms = HAL_GetTick();
    s_last_motion_telemetry_ms = HAL_GetTick();
    s_last_rx_byte_ms = HAL_GetTick();

    (void)HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1U);
}

void chassis_protocol_poll(uint32_t now_ms)
{
    uint8_t byte;

    /* 错误回调后若接收没有重新挂起，这里补一次。 */
    if (huart1.RxState == HAL_UART_STATE_READY)
    {
        (void)HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1U);
    }

    /* 半帧后超过 50 ms 没有新字节，丢弃残帧，避免下一帧被当成载荷吞掉。 */
    if ((s_parser_state != PARSER_WAIT_SOF1) &&
        ((uint32_t)(now_ms - s_last_rx_byte_ms) > 50U))
    {
        (void)give_comm_result(false);
        s_parser_state = PARSER_WAIT_SOF1;
    }

    while (s_rx_tail != s_rx_head)
    {
        byte = s_rx_ring[s_rx_tail];
        s_rx_tail = (uint16_t)((s_rx_tail + 1U) % CHASSIS_UART_RX_RING_SIZE);
        protocol_feed_byte(byte, now_ms);
    }

    if ((uint32_t)(now_ms - s_last_telemetry_ms) >= CHASSIS_TELEMETRY_PERIOD_MS)
    {
        s_last_telemetry_ms = now_ms;
        chassis_protocol_send_status(now_ms);
    }

    if ((uint32_t)(now_ms - s_last_nav_telemetry_ms) >= CHASSIS_TELEMETRY_PERIOD_MS)
    {
        s_last_nav_telemetry_ms = now_ms;
        chassis_protocol_send_nav(now_ms);
    }

    if ((uint32_t)(now_ms - s_last_motion_telemetry_ms) >= CHASSIS_TELEMETRY_PERIOD_MS)
    {
        s_last_motion_telemetry_ms = now_ms;
        chassis_protocol_send_motion(now_ms);
    }
}

/*---------------------------------------------------------------------------*/
/* HAL 回调                                                                    */
/*---------------------------------------------------------------------------*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint16_t next_head;

    if (huart->Instance != USART1)
    {
        return;
    }

    s_last_rx_byte_ms = HAL_GetTick();
    next_head = (uint16_t)((s_rx_head + 1U) % CHASSIS_UART_RX_RING_SIZE);
    if (next_head != s_rx_tail)
    {
        s_rx_ring[s_rx_head] = s_rx_byte;
        s_rx_head = next_head;
    }
    else
    {
        (void)give_comm_result(false);
    }

    (void)HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1U);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1)
    {
        return;
    }

    __HAL_UART_CLEAR_PEFLAG(huart);
    huart->ErrorCode = HAL_UART_ERROR_NONE;
    (void)HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1U);
}
