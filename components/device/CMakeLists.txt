idf_component_register(SRCS "motor_board_uart.c" "imu963ra.c" "board_rgb.c"
                       INCLUDE_DIRS "include" REQUIRES esp_driver_uart esp_driver_i2c esp_driver_gpio)
