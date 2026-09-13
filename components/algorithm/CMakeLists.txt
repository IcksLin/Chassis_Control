idf_component_register(
    SRCS
        "chassis_hal.c"
        "move_control.c"
        "imu963ra_attitude.c"
        "fusion/src/FusionAhrs.c"
        "fusion/src/FusionBias.c"
        "fusion/src/FusionCompass.c"
    INCLUDE_DIRS "include" "fusion/include"
    REQUIRES esp_timer freertos
)
