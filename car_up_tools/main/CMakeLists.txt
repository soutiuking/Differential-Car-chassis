idf_component_register(
    SRCS "web_server.c" "path_planner.c" "uart_link.c" "car_state.c" "protocol.c" "nvs_backup.c" "main.c"
    INCLUDE_DIRS "."
    REQUIRES esp_http_server esp_wifi nvs_flash json driver
    EMBED_TXTFILES "www/index.html" "www/app.js" "www/style.css")
