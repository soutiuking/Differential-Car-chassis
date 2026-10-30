/*
 * VL53L1X platform layer for STM32F1 HAL.
 *
 * Copyright (c) 2016, STMicroelectronics - All Rights Reserved
 * License: BSD 3-clause "New" or "Revised" License.
 *
 * Adapted for the dual-sensor TOF400C board: the device address is passed in
 * by the ST ULD API, while both sensors share I2C2.
 */
#include "vl53l1_platform.h"
#include "i2c.h"

#define VL53L1_I2C_HANDLE       (&hi2c2)
#define VL53L1_I2C_TIMEOUT_MS   20U
#define VL53L1_IO_ERROR         ((int8_t)-13)

int8_t VL53L1_ReadMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count)
{
    if (HAL_I2C_Mem_Read(VL53L1_I2C_HANDLE, dev, index,
                         I2C_MEMADD_SIZE_16BIT, pdata, (uint16_t)count,
                         VL53L1_I2C_TIMEOUT_MS) != HAL_OK)
    {
        return VL53L1_IO_ERROR;
    }
    return 0;
}

int8_t VL53L1_WriteMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count)
{
    if (HAL_I2C_Mem_Write(VL53L1_I2C_HANDLE, dev, index,
                          I2C_MEMADD_SIZE_16BIT, pdata, (uint16_t)count,
                          VL53L1_I2C_TIMEOUT_MS) != HAL_OK)
    {
        return VL53L1_IO_ERROR;
    }
    return 0;
}

int8_t VL53L1_RdByte(uint16_t dev, uint16_t index, uint8_t *pdata)
{
    return VL53L1_ReadMulti(dev, index, pdata, 1U);
}

int8_t VL53L1_WrByte(uint16_t dev, uint16_t index, uint8_t data)
{
    return VL53L1_WriteMulti(dev, index, &data, 1U);
}

int8_t VL53L1_RdWord(uint16_t dev, uint16_t index, uint16_t *pdata)
{
    uint8_t buf[2];

    if (VL53L1_ReadMulti(dev, index, buf, 2U) != 0)
    {
        return VL53L1_IO_ERROR;
    }
    *pdata = (uint16_t)(((uint16_t)buf[0] << 8) | (uint16_t)buf[1]);
    return 0;
}

int8_t VL53L1_WrWord(uint16_t dev, uint16_t index, uint16_t data)
{
    uint8_t buf[2];

    buf[0] = (uint8_t)(data >> 8);
    buf[1] = (uint8_t)(data & 0xFFU);
    return VL53L1_WriteMulti(dev, index, buf, 2U);
}

int8_t VL53L1_RdDWord(uint16_t dev, uint16_t index, uint32_t *pdata)
{
    uint8_t buf[4];

    if (VL53L1_ReadMulti(dev, index, buf, 4U) != 0)
    {
        return VL53L1_IO_ERROR;
    }
    *pdata = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) |
             ((uint32_t)buf[2] << 8) | (uint32_t)buf[3];
    return 0;
}

int8_t VL53L1_WrDWord(uint16_t dev, uint16_t index, uint32_t data)
{
    uint8_t buf[4];

    buf[0] = (uint8_t)(data >> 24);
    buf[1] = (uint8_t)(data >> 16);
    buf[2] = (uint8_t)(data >> 8);
    buf[3] = (uint8_t)(data & 0xFFU);
    return VL53L1_WriteMulti(dev, index, buf, 4U);
}
