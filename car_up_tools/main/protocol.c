#include "protocol.h"

uint16_t proto_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFFU;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8U; bit++) {
            crc = (crc & 1U) ? (uint16_t)((crc >> 1) ^ 0xA001U)
                             : (uint16_t)(crc >> 1);
        }
    }
    return crc;
}

size_t proto_build(uint8_t type, const uint8_t *payload, uint8_t len, uint8_t *out)
{
    if (len > PROTO_MAX_PAYLOAD) {
        return 0;
    }
    out[0] = PROTO_SOF1;
    out[1] = PROTO_SOF2;
    out[2] = len;
    out[3] = 0U;
    out[4] = type;
    if (len && payload) {
        for (uint8_t i = 0; i < len; i++) {
            out[5U + i] = payload[i];
        }
    }
    uint16_t crc = proto_crc16(&out[2], (size_t)(3U + len));
    out[5U + len] = (uint8_t)(crc & 0xFFU);
    out[6U + len] = (uint8_t)(crc >> 8);
    return (size_t)(7U + len);
}
