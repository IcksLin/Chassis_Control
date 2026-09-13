idf_component_register(SRCS "chassis_runtime.c" "imu_web_runtime.c" "motion_safety.c"
                       INCLUDE_DIRS "include"
                       REQUIRES algorithm device esp_timer freertos log esp_wifi esp_event
                                esp_netif nvs_flash esp_http_server)
