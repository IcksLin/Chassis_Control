# 硬件接线说明

本文档对应当前固件中的固定引脚定义（ESP32-S3）。除电源和地线外，信号线应按下表连接。

## 接线表

| 外设 | ESP32-S3 引脚 | 外设端 | 电气参数/说明 |
|---|---:|---|---|
| 电机控制板 UART1 TX | GPIO17 | 控制板 RX2 | 115200，8 数据位、无校验、1 停止位 |
| 电机控制板 UART1 RX | GPIO18 | 控制板 TX2 | 与 TX 交叉连接 |
| IMU963RA I²C SDA | GPIO8 | SDA | I²C 主机，400 kHz |
| IMU963RA I²C SCL | GPIO9 | SCL | I²C 主机，400 kHz |
| 电机控制板/IMU 地 | GND | GND | 必须共地 |
| 电机控制板/IMU 电源 | 按模块额定电压 | VCC | 使用与模块匹配的稳压电源，禁止直接猜测电压 |

## 电机控制板

GPIO17 接控制板 `RX2`，GPIO18 接控制板 `TX2`，不要将 TX 对 TX 或 RX 对 RX 直连。固件通过 `$spd:v1,v2,v3,v4#` 发送四路速度，并接收 `MAll`、`MTEP`、`MSPD` 帧。

逻辑轮编号与电机通道为：1=`right_rear/M1`、2=`left_rear/M2`、3=`right_front/M3`、4=`left_front/M4`。方向正负由 `components/algorithm/chassis_hal.c` 中的方向表统一处理。

## IMU963RA

IMU 接在 I²C0 总线上：GPIO8 为 SDA，GPIO9 为 SCL。固件会扫描 7 位地址 `0x6A` 和 `0x6B`，并要求 WHO_AM_I 为 `0x6B`。驱动启用 ESP32-S3 内部上拉；若线长较长或总线上挂载多个器件，应根据模块情况补充外部上拉。

当前未使用 IMU 的 INT、CS、SA0 等引脚，CS/SA0 应按模块手册配置为 I²C 模式和目标地址。

## 调试与供电

USB 串口/JTAG 用于下载和日志输出，不属于上述电机 UART。首次调试时先断开电机电源或抬起底盘；固件启动不会自动驱动车轮。电机控制板、IMU 和 ESP32-S3 必须共地，信号电平需满足各模块的 3.3 V 逻辑要求。
