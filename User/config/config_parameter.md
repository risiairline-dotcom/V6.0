# 智能搬运车参数说明

> 适用版本：V1.0整车跑图稳定版  
> 原则：调整参数前先保存当前稳定值，每次只修改一类参数并单独实车验证。

## 1. 修改路线距离在哪里？

文件：`User/config/mission_config.h`

```c
MISSION_ROUTE_SIDE_DISTANCE_MM
MISSION_ROUTE_FORWARD1_MM
MISSION_ROUTE_FORWARD2_MM
MISSION_ROUTE_BACKWARD1_MM
MISSION_ROUTE_BACKWARD2_MM
MISSION_ROUTE_BACKWARD3_MM
MISSION_ROUTE_BACKWARD4_MM
```

这些宏只保存距离幅值。实际前进、后退、左移和右移由`mission.c`调用`Chassis_MoveDistance()`时参数所在轴及正负号决定。

## 2. 修改运行速度在哪里？

Mission路线速度位于`User/config/mission_config.h`：

```c
MISSION_SIDE_SPEED_MM_S
MISSION_MOVE_SPEED_MM_S
```

底盘长距离兼容回退速度参数位于`User/config/chassis_config.h`：

```c
DISTANCE_PROFILE_START_MM
DISTANCE_MIN_SPEED_MM_S
```

Mission速度决定一次动作允许的最大速度；当前1000 mm主路径使用`trajectory`梯形速度规划，上述参数只供规划失败时的平方根减速兼容回退。

## 3. 修改加速度在哪里？

Mission动作加速度位于`User/config/mission_config.h`：

```c
MISSION_SIDE_ACCEL_MM_S2
MISSION_MOVE_ACCEL_MM_S2
```

电机加速度档位换算位于`User/config/chassis_config.h`：

```c
MOTOR_ACCEL_LEVEL_MIN
MOTOR_ACCEL_LEVEL_MAX
MOTOR_ACCEL_MM_S2_PER_LEVEL
```

转向加速度位于同一文件：

```c
TURN_ACCELERATION_MM_S2
```

## 4. 修改转向速度在哪里？

文件：`User/config/chassis_config.h`

```c
TURN_MAX_SPEED_DEG_S
TURN_MIN_SPEED_DEG_S
TURN_KP
TURN_ACCELERATION_MM_S2
TURN_STOP_TOLERANCE_DEG
TURN_STOP_GYRO_DEG_S
TURN_STABLE_TIME_MS
```

路线中的左转和右转目标角度位于`User/config/mission_config.h`：

```c
MISSION_ROUTE_TURN_DEG
MISSION_ROUTE_TURN_RIGHT_DEG
```

## 5. 修改舵机角度在哪里？

舵机PWM范围和最大角度：`User/config/servo_config.h`

```c
SERVO_1_MIN_PULSE_US
SERVO_1_MAX_PULSE_US
SERVO_1_MAX_ANGLE
SERVO_3_MIN_PULSE_US
SERVO_3_MAX_PULSE_US
SERVO_3_MAX_ANGLE
```

转盘三个工位角度：`User/config/turntable_config.h`

```c
TURNTABLE_POS_1_ANGLE
TURNTABLE_POS_2_ANGLE
TURNTABLE_POS_3_ANGLE
TURNTABLE_MOVE_TIME_MS
```

## 6. 修改底盘尺寸在哪里？

文件：`User/config/chassis_config.h`

```c
CHASSIS_LENGTH
CHASSIS_WIDTH
CHASSIS_WHEEL_DIAMETER
ROTATE_FACTOR
```

这些数值参与麦克纳姆运动学和速度/距离换算，已经实车验证，不应在普通路线调试中修改。

## 7. 修改电机方向在哪里？

文件：`User/config/chassis_config.h`

```c
MOTOR_M1_SIGN
MOTOR_M2_SIGN
MOTOR_M3_SIGN
MOTOR_M4_SIGN
CHASSIS_FORWARD_SIGN
CHASSIS_LATERAL_SIGN
```

电机CAN地址映射也位于该文件：

```c
MOTOR_M1_ID
MOTOR_M2_ID
MOTOR_M3_ID
MOTOR_M4_ID
```

以上方向和地址均属于实车锁定参数，代码整理阶段禁止修改。

## 8. 距离标定和停止参数

文件：`User/config/chassis_config.h`

```c
DISTANCE_X_CALIBRATION
DISTANCE_Y_CALIBRATION
DISTANCE_PROFILE_START_MM
DISTANCE_MIN_SPEED_MM_S
DISTANCE_STOP_TOLERANCE_MM
DISTANCE_CONTROL_PERIOD_MS
DISTANCE_MAX_VALID_DT_MS
```

## 9. 参数审计结果

### 9.1 重复定义

当前`User/config/`中的业务参数没有发现同名重复定义。

### 9.2 已定义但当前未使用

以下宏当前尚未进入控制判断，本阶段保留不删除：

```c
DISTANCE_STOP_TOLERANCE_MM
DISTANCE_MAX_VALID_DT_MS
```

### 9.3 位于`.c`文件、后续可考虑迁入config的参数

以下项目只列出，不在本阶段迁移：

- `User/input/button.c`
  - `BUTTON_DEBOUNCE_MS`
  - `BUTTON_PRESSED_LEVEL`
- `User/servo/servo.c`
  - `SERVO_2_MIN_TEST_PULSE_US`
  - `SERVO_2_MAX_TEST_PULSE_US`
- `User/chassis/chassis.c`
  - 四轮位置到达容差、连续确认次数、轮询周期、同步超时和稳定等待时间。

其中Chassis属于已实车验证锁定模块，未经专项授权不得迁移或修改。HWT101帧格式、EMM42命令字、圆周率等内部实现常量不属于现场调试参数，应继续留在各自模块内部。

## 10. 修改参数时的检查顺序

1. 确认只修改目标宏，不顺带格式化底层源码。
2. 检查宏值单位：mm、mm/s、mm/s²、degree或degree/s。
3. 编译并确认无错误和警告。
4. 先做单动作低速测试。
5. 再执行完整Mission。
6. 将最终实车值和测试日期记录到版本说明中。
