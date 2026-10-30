/**
  ******************************************************************************
  * @file    mpu6050.h
  * @brief   MPU6050 六轴传感器驱动（I2C1）。
  ******************************************************************************
  */
#ifndef MPU6050_H
#define MPU6050_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    float accel_g[3];      /* ax, ay, az，单位 g */
    float gyro_dps[3];     /* gx, gy, gz，单位 deg/s，已减零偏 */
    float temperature_c;
} mpu6050_sample_t;

bool mpu6050_init(void);
bool mpu6050_read(mpu6050_sample_t *sample);

/* 静止状态采样并计算陀螺零偏；调用时小车必须完全静止。 */
bool mpu6050_calibrate_gyro(uint16_t samples);

void mpu6050_get_gyro_bias(float bias_dps[3]);

#ifdef __cplusplus
}
#endif

#endif /* MPU6050_H */
