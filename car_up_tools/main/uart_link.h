#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef void (*uart_frame_cb_t)(uint8_t type, const uint8_t *payload, uint8_t len);

void uart_link_init(uart_frame_cb_t callback);
bool uart_link_send(uint8_t type, const uint8_t *payload, uint8_t len);
