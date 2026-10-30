/**
  ******************************************************************************
  * @file    board_ui.h
  * @brief   板载 4 按键 + 4 共阳极 LED + OLED 页面/标定控制。
  ******************************************************************************
  */
#ifndef BOARD_UI_H
#define BOARD_UI_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void board_ui_init(void);
void board_ui_update(uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_UI_H */
