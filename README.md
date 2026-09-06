# ESP32-S3 four-wheel chassis test

## Source layout

- `components/device`: UART driver and motor-board wire protocol
- `components/algorithm`: logical wheel mapping, sign normalization and public speed HAL
- `components/system`: FreeRTOS tasks, 50 Hz forwarding, watchdog and test console
- `main`: minimal application entry point

Application code should normally call `chassis_hal_set_wheel_speeds()` and
`chassis_hal_get_encoder_delta()` from `chassis_hal.h`. It should not depend on
the motor board's UART frame format.

- Motor-board UART: 115200 8N1
- ESP32-S3 GPIO17 (TX) -> motor-board RX2
- ESP32-S3 GPIO18 (RX) <- motor-board TX2
- Common GND is required
- Logical wheels: 1 right-rear/M1, 2 right-front/M2, 3 left-rear/M3, 4 left-front/M4
- Speed forwarding: 50 Hz
- Command watchdog: 500 ms, then all zero

Build and flash with ESP-IDF 5.4:

```sh
idf.py set-target esp32s3
idf.py build
idf.py -p /dev/ttyACM0 flash monitor
```

The firmware does not move on boot. Lift the chassis off the floor and enter `t`
in the monitor to run the one-shot test. Each wheel runs at +120 mm/s for 800 ms,
with a 500 ms stopped interval. Enter any character during the test to abort.

If a logical wheel must reverse for chassis-forward motion, change its entry in
`s_hal.direction` from `1` to `-1`. The same sign is applied to encoder feedback,
so positive logical speed and positive logical encoder delta remain consistent.
