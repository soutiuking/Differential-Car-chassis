/**
  ******************************************************************************
  * @file    nav_map.h
  * @brief   4 m × 4 m 矩形场地墙线模型。
  ******************************************************************************
  */
#ifndef NAV_MAP_H
#define NAV_MAP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/* 计算传感器射线到已知墙面的预测距离；射线打到角点或场外时返回 false。 */
bool nav_map_predict_range(float x, float y, float theta,
                           uint8_t sensor, float *range_m);

#ifdef __cplusplus
}
#endif

#endif /* NAV_MAP_H */
