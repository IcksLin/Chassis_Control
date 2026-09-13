# 全向移动航向补偿规划

## 目标

在操作者只请求平移时，保持开始移动时的车体 yaw，抑制地面坡度、轮径差异和打滑引起的非指令偏转；不试图仅凭 IMU 解决位置横向误差。

## 控制结构

网页托盘输出车体坐标系中的 `vx`（前进）、`vy`（右移）和 `w`（旋转）。当 `w` 在死区内，控制层锁存当前 yaw 为参考值，并计算：

`w_comp = clamp(Kp * wrap(yaw_ref - yaw), -0.35, 0.35)`

随后将 `(vx, vy, w_comp)` 送入四个 45 度全向轮逆运动学。当前 `Kp=0.025`，需要在平整地面与典型坡面上分别调试。

## 传感器策略

磁力计会受电机及驱动电流影响。电机运动时不得依赖未经干扰检测的磁航向修正；航向保持优先使用陀螺仪融合得到的短时 yaw。长期位置或航向精度还需要编码器-IMU 融合及外部定位约束。

## 局限

roll/pitch 不能直接转换为轮速补偿来保证全局直线：坡面、轮胎打滑和电机闭环误差都会造成横向误差。当前实现先解决可测且可控的 yaw 偏差；后续应基于编码器速度与 IMU yaw 建立速度闭环。

## 参考

- [Fahmizal & Kuo, Mecanum robot trajectory and heading tracking with IMU](https://scholars.lib.ntu.edu.tw/entities/publication/c6048581-5b6f-4a92-9e76-d00d74f8e059)
- [Diegel et al., Geometry and kinematics of the Mecanum wheel](https://www.sciencedirect.com/science/article/pii/S0167839608000770)
