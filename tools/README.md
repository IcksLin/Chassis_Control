# IMU 统计工具

`imu_stats.py` 从 ESP32 的 `/api/attitude` 接口采样，输出 CSV，并统计磁力计三轴、磁场模长及姿态角的均值、标准差、最小值、最大值和峰峰值。

```bash
python3 tools/imu_stats.py --url http://192.168.43.19 --duration 30 --label motor_off
```

建议分别使用 `motor_off`、`driver_on`、`wheel_1_low` 等标签采集，每次采集结束后保存生成的 CSV，比较 `mx/my/mz/norm` 的 `p2p` 和 `std`。
