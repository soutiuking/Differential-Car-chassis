/**
  ******************************************************************************
  * @file    chassis_protocol.h
  * @brief   底盘串口帧协议（USART1，115200 8N1）。
  ******************************************************************************
  */
#ifndef CHASSIS_PROTOCOL_H
#define CHASSIS_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 启动 USART1 单字节中断接收并复位解析器。 */
void chassis_protocol_init(void);

/* 放在 main 的 while(1) 中，处理接收字节并按 10 Hz 回传状态。 */
void chassis_protocol_poll(uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* CHASSIS_PROTOCOL_H */
