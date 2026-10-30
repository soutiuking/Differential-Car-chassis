/**
  ******************************************************************************
  * @file    vl53l1_platform.h
  * @brief   VL53L1X 平台层：STM32 HAL I2C2 读写。
  *
  * 该平台层只保留 ST ULD API 需要的 I2C 原语；XSHUT/地址分配在 tof400.c
  * 中完成，以支持同一 I2C 总线上的两片 VL53L1X。
  ******************************************************************************
  */
#ifndef VL53L1_PLATFORM_H
#define VL53L1_PLATFORM_H

#include "main.h"

int8_t VL53L1_WriteMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count);
int8_t VL53L1_ReadMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count);
int8_t VL53L1_WrByte(uint16_t dev, uint16_t index, uint8_t data);
int8_t VL53L1_WrWord(uint16_t dev, uint16_t index, uint16_t data);
int8_t VL53L1_WrDWord(uint16_t dev, uint16_t index, uint32_t data);
int8_t VL53L1_RdByte(uint16_t dev, uint16_t index, uint8_t *pdata);
int8_t VL53L1_RdWord(uint16_t dev, uint16_t index, uint16_t *pdata);
int8_t VL53L1_RdDWord(uint16_t dev, uint16_t index, uint32_t *pdata);

#endif /* VL53L1_PLATFORM_H */
