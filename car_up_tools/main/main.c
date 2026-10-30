#include "car_state.h"
#include "path_planner.h"
#include "uart_link.h"
#include "web_server.h"
#include "nvs_backup.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "main";

static void on_frame(uint8_t type, const uint8_t *payload, uint8_t len)
{
    state_update_frame(type, payload, len);
}

void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    state_init();
    planner_init();
    backup_init();
    uart_link_init(on_frame);
    web_server_start();
    xTaskCreate(planner_monitor_task, "path_monitor", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "ready: AP %s, http://192.168.4.1/", "Crawler-Car");
}
