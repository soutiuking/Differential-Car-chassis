#include "uart_link.h"
#include "protocol.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include <string.h>

#define CAR_UART_PORT       UART_NUM_1
#define CAR_UART_TX_GPIO    17
#define CAR_UART_RX_GPIO    18
#define CAR_UART_BAUD       115200
#define CAR_UART_BUF_SIZE   4096

static const char *TAG = "uart_link";
static uart_frame_cb_t s_callback;
static SemaphoreHandle_t s_tx_mutex;

typedef enum {
    P_WAIT_SOF1 = 0,
    P_WAIT_SOF2,
    P_WAIT_LEN,
    P_WAIT_SEQ,
    P_WAIT_TYPE,
    P_WAIT_PAYLOAD,
    P_WAIT_CRC_L,
    P_WAIT_CRC_H
} parser_state_t;

static parser_state_t s_state;
static uint8_t s_len;
static uint8_t s_seq;
static uint8_t s_type;
static uint8_t s_payload[PROTO_MAX_PAYLOAD];
static uint8_t s_index;
static uint8_t s_crc_l;
static uint8_t s_crc_h;

static void parser_feed(uint8_t byte)
{
    switch (s_state) {
    case P_WAIT_SOF1:
        if (byte == PROTO_SOF1) s_state = P_WAIT_SOF2;
        break;
    case P_WAIT_SOF2:
        s_state = (byte == PROTO_SOF2) ? P_WAIT_LEN : P_WAIT_SOF1;
        break;
    case P_WAIT_LEN:
        if (byte > PROTO_MAX_PAYLOAD) {
            s_state = P_WAIT_SOF1;
        } else {
            s_len = byte;
            s_state = P_WAIT_SEQ;
        }
        break;
    case P_WAIT_SEQ:
        s_seq = byte;
        (void)s_seq;
        s_state = P_WAIT_TYPE;
        break;
    case P_WAIT_TYPE:
        s_type = byte;
        s_index = 0U;
        s_state = (s_len == 0U) ? P_WAIT_CRC_L : P_WAIT_PAYLOAD;
        break;
    case P_WAIT_PAYLOAD:
        s_payload[s_index++] = byte;
        if (s_index >= s_len) s_state = P_WAIT_CRC_L;
        break;
    case P_WAIT_CRC_L:
        s_crc_l = byte;
        s_state = P_WAIT_CRC_H;
        break;
    case P_WAIT_CRC_H: {
        uint8_t body[3U + PROTO_MAX_PAYLOAD];
        s_crc_h = byte;
        body[0] = s_len;
        body[1] = s_seq;
        body[2] = s_type;
        memcpy(&body[3], s_payload, s_len);
        uint16_t crc = proto_crc16(body, (size_t)(3U + s_len));
        if (crc == (uint16_t)(s_crc_l | ((uint16_t)s_crc_h << 8))) {
            if (s_callback) s_callback(s_type, s_payload, s_len);
        }
        s_state = P_WAIT_SOF1;
        break;
    }
    default:
        s_state = P_WAIT_SOF1;
        break;
    }
}

static void uart_rx_task(void *arg)
{
    (void)arg;
    uint8_t buf[256];
    for (;;) {
        int n = uart_read_bytes(CAR_UART_PORT, buf, sizeof(buf), pdMS_TO_TICKS(20));
        for (int i = 0; i < n; i++) {
            parser_feed(buf[i]);
        }
    }
}

void uart_link_init(uart_frame_cb_t callback)
{
    s_callback = callback;
    s_state = P_WAIT_SOF1;
    s_tx_mutex = xSemaphoreCreateMutex();

    uart_config_t cfg = {
        .baud_rate = CAR_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(CAR_UART_PORT, CAR_UART_BUF_SIZE, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(CAR_UART_PORT, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(CAR_UART_PORT, CAR_UART_TX_GPIO, CAR_UART_RX_GPIO,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    xTaskCreate(uart_rx_task, "uart_rx", 4096, NULL, 10, NULL);
    ESP_LOGI(TAG, "UART1 TX=%d RX=%d %d 8N1", CAR_UART_TX_GPIO, CAR_UART_RX_GPIO, CAR_UART_BAUD);
}

bool uart_link_send(uint8_t type, const uint8_t *payload, uint8_t len)
{
    uint8_t frame[PROTO_MAX_FRAME];
    size_t frame_len = proto_build(type, payload, len, frame);
    if (frame_len == 0U) return false;
    if (xSemaphoreTake(s_tx_mutex, pdMS_TO_TICKS(100)) != pdTRUE) return false;
    int written = uart_write_bytes(CAR_UART_PORT, (const char *)frame, frame_len);
    xSemaphoreGive(s_tx_mutex);
    return written == (int)frame_len;
}
