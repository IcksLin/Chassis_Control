# ESP32-S3 四轮底盘控制

这是一个基于 ESP-IDF 的 ESP32-S3 底盘控制固件，负责电机控制板 UART 通信、轮速/编码器抽象，以及 IMU963RA 姿态采集和 Wi-Fi 网页显示。

硬件接线请先阅读 [HARDWARE_WIRING.md](HARDWARE_WIRING.md)。

## Source layout

- `components/device`: UART driver and motor-board wire protocol
- `components/algorithm`: logical wheel mapping, sign normalization and public speed HAL
- `components/system`: FreeRTOS tasks, 50 Hz forwarding, watchdog and test console
- `main`: minimal application entry point

Application code should normally call `chassis_hal_set_wheel_speeds()` and
`chassis_hal_get_encoder_delta()` from `chassis_hal.h`. It should not depend on
the motor board's UART frame format.

- Motor-board UART: UART1，115200 8N1
- ESP32-S3 GPIO17 (TX) -> motor-board RX2
- ESP32-S3 GPIO18 (RX) <- motor-board TX2
- Common GND is required
- IMU963RA I²C: I²C0，SDA GPIO8，SCL GPIO9，400 kHz，地址 0x6A/0x6B
- Logical wheels: 1 right-rear/M1, 2 left-rear/M2, 3 right-front/M3, 4 left-front/M4
- Speed forwarding: 50 Hz
- Command watchdog: 500 ms, then all zero

Build and flash with ESP-IDF 6.1:

```sh
idf.py set-target esp32s3
idf.py build
idf.py -p /dev/ttyACM0 flash monitor
```

工程默认目标为 `esp32s3`。首次使用可执行 `idf.py set-target esp32s3`；Wi-Fi SSID 和密码需在 `components/system/private_config.h` 中配置（可参考同目录的 `.example` 文件）。启动后网页地址会打印在日志中。

The firmware does not move on boot. Lift the chassis off the floor and enter `t`
in the monitor to run the one-shot test. Each wheel runs at +120 mm/s for 800 ms,
with a 500 ms stopped interval. Enter any character during the test to abort.

If a logical wheel must reverse for chassis-forward motion, change its entry in
`s_hal.direction` from `1` to `-1`. The same sign is applied to encoder feedback,
so positive logical speed and positive logical encoder delta remain consistent.
