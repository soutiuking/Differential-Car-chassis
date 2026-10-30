#pragma once

#include <stddef.h>
#include <stdint.h>

#define PROTO_SOF1          0xAAU
#define PROTO_SOF2          0x55U
#define PROTO_MAX_PAYLOAD   48U
#define PROTO_MAX_FRAME     (5U + PROTO_MAX_PAYLOAD + 2U)

uint16_t proto_crc16(const uint8_t *data, size_t len);
size_t proto_build(uint8_t type, const uint8_t *payload, uint8_t len, uint8_t *out);
