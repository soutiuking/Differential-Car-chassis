#include "web_server.h"
#include "car_state.h"
#include "path_planner.h"
#include "uart_link.h"
#include "nvs_backup.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "cJSON.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define AP_SSID       "Crawler-Car"
#define AP_PASSWORD   "12345678"

static const char *TAG = "web";

extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[] asm("_binary_index_html_end");
extern const uint8_t app_js_start[] asm("_binary_app_js_start");
extern const uint8_t app_js_end[] asm("_binary_app_js_end");
extern const uint8_t style_css_start[] asm("_binary_style_css_start");
extern const uint8_t style_css_end[] asm("_binary_style_css_end");

static void add_pair(cJSON *a, float x, float y)
{
    cJSON *p = cJSON_CreateArray();
    cJSON_AddItemToArray(p, cJSON_CreateNumber(x));
    cJSON_AddItemToArray(p, cJSON_CreateNumber(y));
    cJSON_AddItemToArray(a, p);
}

static void send_json(httpd_req_t *req, cJSON *root)
{
    char *txt = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (!txt) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "json");
        return;
    }
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_send(req, txt, (ssize_t)strlen(txt));
    free(txt);
}

static esp_err_t api_state_get(httpd_req_t *req)
{
    car_state_t st;
    state_lock();
    st = g_state;
    state_unlock();

    cJSON *root = cJSON_CreateObject();

    cJSON *chassis = cJSON_AddObjectToObject(root, "chassis");
    cJSON_AddBoolToObject(chassis, "enabled", (st.flags & 0x01U) != 0U);
    cJSON_AddNumberToObject(chassis, "mode", (st.flags >> 1) & 0x03U);
    cJSON_AddNumberToObject(chassis, "faults", st.faults);
    cJSON_AddNumberToObject(chassis, "v", st.linear_mm_s / 1000.0f);
    cJSON_AddNumberToObject(chassis, "w", st.angular_mrad_s / 1000.0f);
    cJSON *rpm = cJSON_AddArrayToObject(chassis, "rpm");
    for (int i = 0; i < 4; i++) cJSON_AddItemToArray(rpm, cJSON_CreateNumber(st.measured_rpm[i] / 10.0f));

    cJSON *nav = cJSON_AddObjectToObject(root, "nav");
    cJSON_AddBoolToObject(nav, "ok", st.have_nav);
    cJSON_AddNumberToObject(nav, "mode", st.nav_mode);
    cJSON_AddNumberToObject(nav, "x", st.x_m);
    cJSON_AddNumberToObject(nav, "y", st.y_m);
    cJSON_AddNumberToObject(nav, "theta", st.theta_rad);
    cJSON_AddNumberToObject(nav, "v", st.v_mps);
    cJSON_AddNumberToObject(nav, "w", st.omega_radps);
    cJSON_AddNumberToObject(nav, "roll", st.roll_rad);
    cJSON_AddNumberToObject(nav, "pitch", st.pitch_rad);
    cJSON_AddNumberToObject(nav, "pxx", st.p_xx);
    cJSON_AddNumberToObject(nav, "pyy", st.p_yy);
    cJSON *tof = cJSON_AddArrayToObject(nav, "tof");
    cJSON *tv = cJSON_AddArrayToObject(nav, "tof_valid");
    cJSON *to = cJSON_AddArrayToObject(nav, "tof_obstacle");
    for (int i = 0; i < 2; i++) {
        cJSON_AddItemToArray(tof, cJSON_CreateNumber(st.tof_mm[i]));
        cJSON_AddItemToArray(tv, cJSON_CreateBool(st.tof_valid[i]));
        cJSON_AddItemToArray(to, cJSON_CreateBool(st.tof_obstacle[i]));
    }

    cJSON *motion = cJSON_AddObjectToObject(root, "motion");
    cJSON_AddNumberToObject(motion, "state", st.motion_state);
    cJSON_AddNumberToObject(motion, "active", st.motion_active);
    cJSON_AddNumberToObject(motion, "total", st.motion_total);
    cJSON_AddNumberToObject(motion, "completed", st.motion_completed);
    cJSON_AddNumberToObject(motion, "error", st.motion_error);
    cJSON_AddNumberToObject(motion, "cross", st.motion_cross_track);
    cJSON_AddNumberToObject(motion, "heading", st.motion_heading_error);
    cJSON_AddNumberToObject(motion, "max_v", st.motion_max_v);
    cJSON_AddNumberToObject(motion, "max_w", st.motion_max_w);

    cJSON *param = cJSON_AddObjectToObject(root, "param");
    cJSON_AddBoolToObject(param, "have", st.have_param);
    cJSON_AddNumberToObject(param, "id", st.param_id);
    cJSON_AddNumberToObject(param, "status", st.param_status);
    cJSON_AddNumberToObject(param, "value", st.param_value);
    cJSON *params = cJSON_AddArrayToObject(param, "values");
    for (int i = 0; i < 7; i++) {
        cJSON_AddItemToArray(params, st.params_valid[i]
            ? cJSON_CreateNumber(st.params[i])
            : cJSON_CreateNull());
    }

    cJSON *calib = cJSON_AddObjectToObject(root, "calib");
    cJSON_AddBoolToObject(calib, "have", st.have_calib);
    cJSON_AddNumberToObject(calib, "action", st.calib_action);
    cJSON_AddNumberToObject(calib, "sensor", st.calib_sensor);
    cJSON_AddNumberToObject(calib, "count", st.calib_count);
    cJSON_AddNumberToObject(calib, "status", st.calib_status);
    cJSON_AddNumberToObject(calib, "known", st.calib_known_mm);
    cJSON_AddNumberToObject(calib, "raw", st.calib_raw_mm);
    cJSON_AddNumberToObject(calib, "gain", st.calib_gain);
    cJSON_AddNumberToObject(calib, "offset", st.calib_offset);

    cJSON *plan = cJSON_AddArrayToObject(root, "plan");
    for (uint16_t i = 0; i < st.plan_count; i++) add_pair(plan, st.plan_x[i], st.plan_y[i]);
    cJSON *trail = cJSON_AddArrayToObject(root, "trail");
    for (uint16_t i = 0; i < st.trail_count; i++) add_pair(trail, st.trail_x[i], st.trail_y[i]);

    send_json(req, root);
    return ESP_OK;
}

static float jnum(const cJSON *o, const char *name, float def)
{
    const cJSON *v = cJSON_GetObjectItemCaseSensitive(o, name);
    return cJSON_IsNumber(v) ? (float)v->valuedouble : def;
}

static int jint(const cJSON *o, const char *name, int def)
{
    const cJSON *v = cJSON_GetObjectItemCaseSensitive(o, name);
    return cJSON_IsNumber(v) ? v->valueint : def;
}

static bool send_i16_pair(uint8_t type, int16_t a, int16_t b, uint16_t c)
{
    uint8_t p[6];
    p[0] = (uint8_t)(a & 0xFF); p[1] = (uint8_t)((a >> 8) & 0xFF);
    p[2] = (uint8_t)(b & 0xFF); p[3] = (uint8_t)((b >> 8) & 0xFF);
    p[4] = (uint8_t)(c & 0xFF); p[5] = (uint8_t)((c >> 8) & 0xFF);
    return uart_link_send(type, p, 6U);
}

static bool do_path_shape(const cJSON *root, const char *cmd)
{
    float speed = jnum(root, "speed", 0.25f);
    if (strcmp(cmd, "goto") == 0) {
        return planner_goto(jnum(root, "x", 0.5f), jnum(root, "y", 0.5f), speed);
    }
    if (strcmp(cmd, "line") == 0 || strcmp(cmd, "polyline") == 0) {
        const cJSON *arr = cJSON_GetObjectItemCaseSensitive(root, "points");
        if (!cJSON_IsArray(arr)) return false;
        int n = cJSON_GetArraySize(arr);
        if (n < 1 || n > 60) return false;
        float xs[60], ys[60];
        int i = 0;
        const cJSON *it;
        cJSON_ArrayForEach(it, arr) {
            if (!cJSON_IsObject(it)) return false;
            xs[i] = jnum(it, "x", 0.0f);
            ys[i] = jnum(it, "y", 0.0f);
            i++;
        }
        return planner_line(xs, ys, (uint16_t)i, speed);
    }
    if (strcmp(cmd, "rect") == 0) {
        return planner_rect(jnum(root, "x1", 0.5f), jnum(root, "y1", 0.5f),
                            jnum(root, "x2", 1.5f), jnum(root, "y2", 1.5f), speed);
    }
    if (strcmp(cmd, "circle") == 0) {
        bool ccw = jint(root, "ccw", 1) != 0;
        return planner_circle(jnum(root, "cx", 1.0f), jnum(root, "cy", 1.0f),
                              jnum(root, "r", 0.5f), ccw, speed);
    }
    return false;
}

static esp_err_t api_cmd_post(httpd_req_t *req)
{
    char body[2048];
    int len = httpd_req_recv(req, body, sizeof(body) - 1);
    if (len <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "body");
        return ESP_FAIL;
    }
    body[len] = '\0';

    cJSON *root = cJSON_Parse(body);
    if (!root) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad json");
        return ESP_FAIL;
    }
    const cJSON *cmd_obj = cJSON_GetObjectItemCaseSensitive(root, "cmd");
    const char *cmd = cJSON_GetStringValue(cmd_obj);
    bool ok = false;

    if (cmd) {
        if (strcmp(cmd, "mode") == 0) {
            uint8_t p[1] = { (uint8_t)jint(root, "mode", 0) };
            ok = uart_link_send(0x10U, p, 1U);
        } else if (strcmp(cmd, "enable") == 0) {
            uint8_t p[2] = { 1U, 1U };
            ok = uart_link_send(0x04U, p, 2U);
        } else if (strcmp(cmd, "disable") == 0) {
            uint8_t p[2] = { 0U, 0U };
            ok = uart_link_send(0x04U, p, 2U);
            (void)uart_link_send(0x18U, NULL, 0U);
            planner_clear();
        } else if (strcmp(cmd, "stop") == 0) {
            (void)uart_link_send(0x18U, NULL, 0U);
            (void)uart_link_send(0x14U, NULL, 0U);
            (void)uart_link_send(0x06U, NULL, 0U);
            planner_clear();
            ok = true;
        } else if (strcmp(cmd, "nav_stop") == 0) {
            (void)uart_link_send(0x18U, NULL, 0U);
            (void)uart_link_send(0x14U, NULL, 0U);
            planner_clear();
            ok = true;
        } else if (strcmp(cmd, "velocity") == 0) {
            int16_t v = (int16_t)(jnum(root, "v", 0.0f) * 1000.0f);
            int16_t w = (int16_t)(jnum(root, "w", 0.0f) * 1000.0f);
            ok = send_i16_pair(0x01U, v, w, 300U);
        } else if (strcmp(cmd, "turn") == 0) {
            state_lock();
            float target = g_state.theta_rad + jnum(root, "theta", 0.0f);
            state_unlock();
            int16_t theta = (int16_t)(target * 1000.0f);
            int16_t omega = 1000;
            uint8_t begin[2] = { 1U, 0U };
            uint8_t p[4] = { (uint8_t)(theta & 0xFF), (uint8_t)((theta >> 8) & 0xFF),
                             (uint8_t)(omega & 0xFF), (uint8_t)((omega >> 8) & 0xFF) };
            ok = uart_link_send(0x16U, begin, 2U) &&
                 uart_link_send(0x12U, p, 4U) &&
                 uart_link_send(0x17U, NULL, 0U);
        } else if (strcmp(cmd, "goto") == 0 || strcmp(cmd, "line") == 0 ||
                   strcmp(cmd, "polyline") == 0 || strcmp(cmd, "rect") == 0 ||
                   strcmp(cmd, "circle") == 0) {
            ok = do_path_shape(root, cmd);
        } else if (strcmp(cmd, "set_speed") == 0) {
            uint16_t maxv = (uint16_t)(jnum(root, "max_v", 0.6f) * 1000.0f);
            uint16_t maxw = (uint16_t)(jnum(root, "max_w", 2.0f) * 1000.0f);
            uint8_t p[4] = { (uint8_t)(maxv & 0xFF), (uint8_t)(maxv >> 8),
                             (uint8_t)(maxw & 0xFF), (uint8_t)(maxw >> 8) };
            ok = uart_link_send(0x15U, p, 4U);
        } else if (strcmp(cmd, "pid") == 0) {
            int wheel = jint(root, "wheel", 255);
            int16_t kp = (int16_t)(jnum(root, "kp", 1.2f) * 1000.0f);
            int16_t ki = (int16_t)(jnum(root, "ki", 0.6f) * 1000.0f);
            int16_t kd = (int16_t)(jnum(root, "kd", 0.02f) * 1000.0f);
            uint8_t p[8] = { (uint8_t)wheel, 0U,
                             (uint8_t)(kp & 0xFF), (uint8_t)(kp >> 8),
                             (uint8_t)(ki & 0xFF), (uint8_t)(ki >> 8),
                             (uint8_t)(kd & 0xFF), (uint8_t)(kd >> 8) };
            ok = uart_link_send(0x03U, p, 8U);
        } else if (strcmp(cmd, "calib") == 0) {
            uint16_t known = (uint16_t)jint(root, "known", 0);
            uint8_t p[6] = { (uint8_t)jint(root, "action", 0),
                             (uint8_t)jint(root, "sensor", 0),
                             (uint8_t)(known & 0xFF), (uint8_t)(known >> 8), 0U, 0U };
            ok = uart_link_send(0x19U, p, 6U);
        } else if (strcmp(cmd, "param") == 0) {
            int32_t val = (int32_t)jint(root, "value", 0);
            uint8_t p[6] = { (uint8_t)jint(root, "action", 0),
                             (uint8_t)jint(root, "id", 0),
                             (uint8_t)(val & 0xFF), (uint8_t)((val >> 8) & 0xFF),
                             (uint8_t)((val >> 16) & 0xFF), (uint8_t)((val >> 24) & 0xFF) };
            ok = uart_link_send(0x1AU, p, 6U);
        } else if (strcmp(cmd, "save") == 0) {
            uint8_t p[6] = { 2U, 0U, 0U, 0U, 0U, 0U };
            ok = uart_link_send(0x1AU, p, 6U);
            if (ok) (void)backup_save();
        } else if (strcmp(cmd, "load") == 0) {
            uint8_t p[6] = { 3U, 0U, 0U, 0U, 0U, 0U };
            ok = uart_link_send(0x1AU, p, 6U);
        } else if (strcmp(cmd, "reset") == 0) {
            uint8_t p[6] = { 4U, 0U, 0U, 0U, 0U, 0U };
            ok = uart_link_send(0x1AU, p, 6U);
        } else if (strcmp(cmd, "backup") == 0) {
            ok = backup_save();
        } else if (strcmp(cmd, "restore") == 0) {
            ok = backup_restore();
        }
    }

    cJSON_Delete(root);
    cJSON *resp = cJSON_CreateObject();
    cJSON_AddBoolToObject(resp, "ok", ok);
    send_json(req, resp);
    return ESP_OK;
}

static esp_err_t file_get(httpd_req_t *req, const uint8_t *start, const uint8_t *end, const char *type)
{
    httpd_resp_set_type(req, type);
    httpd_resp_set_hdr(req, "Cache-Control", "max-age=3600");
    size_t len = (size_t)(end - start);
    if (len > 0U) len--;
    httpd_resp_send(req, (const char *)start, (ssize_t)len);
    return ESP_OK;
}

static esp_err_t index_get(httpd_req_t *req)
{
    return file_get(req, index_html_start, index_html_end, "text/html");
}

static esp_err_t js_get(httpd_req_t *req)
{
    return file_get(req, app_js_start, app_js_end, "application/javascript");
}

static esp_err_t css_get(httpd_req_t *req)
{
    return file_get(req, style_css_start, style_css_end, "text/css");
}

static void wifi_ap_start(void)
{
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init));

    wifi_config_t cfg = {0};
    strncpy((char *)cfg.ap.ssid, AP_SSID, sizeof(cfg.ap.ssid));
    cfg.ap.ssid_len = strlen(AP_SSID);
    strncpy((char *)cfg.ap.password, AP_PASSWORD, sizeof(cfg.ap.password));
    cfg.ap.channel = 1;
    cfg.ap.max_connection = 4;
    cfg.ap.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &cfg));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "AP SSID=%s password=%s http://192.168.4.1/", AP_SSID, AP_PASSWORD);
}

void web_server_start(void)
{
    wifi_ap_start();

    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.max_uri_handlers = 8;
    cfg.stack_size = 8192;
    httpd_handle_t server = NULL;
    ESP_ERROR_CHECK(httpd_start(&server, &cfg));

    httpd_uri_t root = { .uri = "/", .method = HTTP_GET, .handler = index_get };
    httpd_uri_t js = { .uri = "/app.js", .method = HTTP_GET, .handler = js_get };
    httpd_uri_t css = { .uri = "/style.css", .method = HTTP_GET, .handler = css_get };
    httpd_uri_t state = { .uri = "/api/state", .method = HTTP_GET, .handler = api_state_get };
    httpd_uri_t cmd = { .uri = "/api/cmd", .method = HTTP_POST, .handler = api_cmd_post };

    httpd_register_uri_handler(server, &root);
    httpd_register_uri_handler(server, &js);
    httpd_register_uri_handler(server, &css);
    httpd_register_uri_handler(server, &state);
    httpd_register_uri_handler(server, &cmd);
}
