/**
  ******************************************************************************
  * @file    param_store.h
  * @brief   速度/PID/ToF 标定/导航模式的 Flash 持久化。
  ******************************************************************************
  */
#ifndef PARAM_STORE_H
#define PARAM_STORE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#define PARAM_ID_MAX_SPEED        0U
#define PARAM_ID_MAX_OMEGA        1U
#define PARAM_ID_NAV_MODE         2U
#define PARAM_ID_TOF_FRONT_GAIN   3U
#define PARAM_ID_TOF_FRONT_OFFSET 4U
#define PARAM_ID_TOF_RIGHT_GAIN   5U
#define PARAM_ID_TOF_RIGHT_OFFSET 6U

void param_store_init(void);
bool param_store_load(void);
bool param_store_save(void);
bool param_store_reset_defaults(void);

bool param_store_get_value(uint8_t param_id, int32_t *value);
bool param_store_set_value(uint8_t param_id, int32_t value);

#ifdef __cplusplus
}
#endif

#endif /* PARAM_STORE_H */
