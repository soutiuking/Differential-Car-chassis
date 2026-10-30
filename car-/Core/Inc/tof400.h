/**
  ******************************************************************************
  * @file    tof400.h
  * @brief   两片 TOF400C / VL53L1X 测距驱动。
  *
  * 两片传感器共用 I2C2，PB12/PB13 分别控制 XSHUT，启动时依次改地址。
  ******************************************************************************
  */
#ifndef TOF400_H
#define TOF400_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    TOF_FRONT = 0,   /* 前向：车体 +x */
    TOF_RIGHT = 1    /* 右向：车体 -y */
} tof_id_t;

typedef struct
{
    uint16_t distance_mm;   /* 标定后的距离 */
    uint16_t raw_mm;        /* 传感器原始距离 */
    uint8_t range_status;
    bool valid;
    uint32_t timestamp_ms;
} tof_sample_t;

/* 返回 bit0=前向成功，bit1=右向成功。 */
uint8_t tof400_init(void);

/* 非阻塞式读取：只有新数据准备好时才访问 I2C 并返回 true。 */
bool tof400_read(uint8_t sensor, tof_sample_t *sample);

const char *tof400_name(uint8_t sensor);

/* 多点线性标定：corrected = gain * raw + offset。 */
bool tof400_set_calibration(uint8_t sensor, float gain, float offset);
bool tof400_get_calibration(uint8_t sensor, float *gain, float *offset);
bool tof400_get_last_raw(uint8_t sensor, uint16_t *raw_mm);
bool tof400_calib_start(uint8_t sensor);
bool tof400_calib_add(uint8_t sensor, uint16_t known_mm);
uint8_t tof400_calib_count(uint8_t sensor);
bool tof400_calib_fit(uint8_t sensor, float *gain, float *offset);
void tof400_calib_reset(uint8_t sensor);

#ifdef __cplusplus
}
#endif

#endif /* TOF400_H */
