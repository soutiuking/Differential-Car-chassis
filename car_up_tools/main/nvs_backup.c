#include "nvs_backup.h"
#include "car_state.h"
#include "uart_link.h"
#include "nvs.h"
#include "esp_log.h"
#include <string.h>

#define BACKUP_MAGIC   0x424B5031U
#define BACKUP_NS      "car_backup"
#define BACKUP_KEY     "params"

typedef struct {
    uint32_t magic;
    uint32_t mask;
    int32_t values[7];
} backup_blob_t;

static const char *TAG = "backup";

void backup_init(void)
{
    nvs_handle_t h;
    if (nvs_open(BACKUP_NS, NVS_READONLY, &h) == ESP_OK) {
        nvs_close(h);
    }
}

bool backup_has(void)
{
    nvs_handle_t h;
    size_t len = 0;
    bool ok = false;
    if (nvs_open(BACKUP_NS, NVS_READONLY, &h) == ESP_OK) {
        ok = (nvs_get_blob(h, BACKUP_KEY, NULL, &len) == ESP_OK) &&
             (len == sizeof(backup_blob_t));
        nvs_close(h);
    }
    return ok;
}

bool backup_save(void)
{
    backup_blob_t b;
    memset(&b, 0, sizeof(b));
    b.magic = BACKUP_MAGIC;

    state_lock();
    for (int i = 0; i < 7; i++) {
        if (g_state.params_valid[i]) {
            b.values[i] = g_state.params[i];
            b.mask |= (1U << i);
        }
    }
    state_unlock();

    if (b.mask == 0U) {
        return false;
    }

    nvs_handle_t h;
    if (nvs_open(BACKUP_NS, NVS_READWRITE, &h) != ESP_OK) return false;
    esp_err_t err = nvs_set_blob(h, BACKUP_KEY, &b, sizeof(b));
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);
    if (err == ESP_OK) ESP_LOGI(TAG, "saved mask=0x%lx", (unsigned long)b.mask);
    return err == ESP_OK;
}

bool backup_restore(void)
{
    backup_blob_t b;
    size_t len = sizeof(b);
    nvs_handle_t h;
    if (nvs_open(BACKUP_NS, NVS_READONLY, &h) != ESP_OK) return false;
    esp_err_t err = nvs_get_blob(h, BACKUP_KEY, &b, &len);
    nvs_close(h);
    if (err != ESP_OK || len != sizeof(b) || b.magic != BACKUP_MAGIC) return false;

    for (int i = 0; i < 7; i++) {
        if ((b.mask & (1U << i)) == 0U) continue;
        int32_t v = b.values[i];
        uint8_t p[6] = { 1U, (uint8_t)i,
                         (uint8_t)(v & 0xFF), (uint8_t)((v >> 8) & 0xFF),
                         (uint8_t)((v >> 16) & 0xFF), (uint8_t)((v >> 24) & 0xFF) };
        (void)uart_link_send(0x1AU, p, 6U);
    }
    ESP_LOGI(TAG, "restore sent mask=0x%lx", (unsigned long)b.mask);
    return true;
}
