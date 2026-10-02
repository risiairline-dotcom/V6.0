#ifndef CHASSIS_CONFIG_H
#define CHASSIS_CONFIG_H

/* ==================== 麦轮底盘几何参数 ==================== */
/* 底盘尺寸和轮径，单位均为 mm；ROTATE_FACTOR 用于旋转运动学。 */
#define CHASSIS_LENGTH                 188.0f
#define CHASSIS_WIDTH                  220.0f
#define CHASSIS_WHEEL_DIAMETER         100.0f
#define ROTATE_FACTOR                  ((CHASSIS_LENGTH + CHASSIS_WIDTH) / 2.0f)

/* ==================== 车体坐标方向标定 ==================== */
/* 已验证：软件 +X 乘以 -1 后对应实际车头方向。 */
#define CHASSIS_FORWARD_SIGN           (-1)

/* 实车标定：软件 +Y 乘以 -1 后对应车体左移。 */
#define CHASSIS_LATERAL_SIGN           (-1)

/* Path/TIME/Fast 平移速度的四方向实车标定；仅在最终速度下发前使用。 */
#define MOVE_X_POS_CALIBRATION         1.0135f
#define MOVE_X_NEG_CALIBRATION         1.015f
#define MOVE_Y_POS_CALIBRATION         1.033f
#define MOVE_Y_NEG_CALIBRATION         1.020f

/* ==================== 电机基础参数 ==================== */
/* X42S 在16细分时一圈对应3200个脉冲；最大转速单位为 RPM。 */
#define MOTOR_PULSES_PER_REVOLUTION    3200.0f
#define MOTOR_MAX_RPM                  3000
#define CHASSIS_MIN_COMMAND_RPM        1

/* Mission 默认速度统一放在底盘配置；PathPoint 和 MoveVelocity 仍由调用者指定速度。
 * 三者最终都受 MOTOR_MAX_RPM 对应的轮速上限限制。 */
#define CHASSIS_MISSION_MOVE_X_SPEED_MM_S   500.0f
#define CHASSIS_MISSION_MOVE_Y_SPEED_MM_S   500.0f
#define CHASSIS_MISSION_ROTATE_SPEED_DEG_S  30.0f

/* ==================== 电机地址映射 ==================== */
/* 软件编号保持标准轮位，通过 ID 映射到实车 CAN 地址：
 * M1 左前 -> CAN 地址2
 * M2 右前 -> CAN 地址1
 * M3 左后 -> CAN 地址3
 * M4 右后 -> CAN 地址4
 */
#define MOTOR_M1_ID                    2
#define MOTOR_M2_ID                    1
#define MOTOR_M3_ID                    3
#define MOTOR_M4_ID                    4

/* ==================== 电机安装方向 ==================== */
/* 已验证的四个标准轮位安装方向。 */
#define MOTOR_M1_SIGN                  (-1)
#define MOTOR_M2_SIGN                  1
#define MOTOR_M3_SIGN                  (-1)
#define MOTOR_M4_SIGN                  1

/* ==================== 电机加速度换算 ==================== */
/* X42S 协议加速度是 0~255 档而非 mm/s^2，此处给出可现场标定的换算比例。 */
#define MOTOR_ACCEL_LEVEL_MIN          1
#define MOTOR_ACCEL_LEVEL_MAX          255
#define MOTOR_ACCEL_MM_S2_PER_LEVEL    20.0f

/* ==================== 距离移动速度规划 ==================== */
/* 长距离X/Y移动的距离比例速度参数。 */
#define DISTANCE_PROFILE_START_MM       500
#define DISTANCE_MIN_SPEED_MM_S         30
#define DISTANCE_CONTROL_PERIOD_MS      20

/* 电机停止命令按原间隔依次发送。 */
#define CHASSIS_MOTOR_STOP_INTERVAL_MS  10U

/* PathPoint_t.accel_time 为多段路径实际正弦过渡时间，单位 ms。
 * MoveX/MoveY/Rotate 包装接口按运行时间的四分之一生成正弦加减速时间。 */
#define CHASSIS_PATH_DURATION_MIN_MS    100U
#define CHASSIS_PATH_DURATION_MAX_MS    65535U
#define CHASSIS_PATH_ACCEL_TIME_DIVISOR 4U

/* 路径航向计算至少间隔 2 ms；速度 CAN 指令仍按原周期发送。 */
#define PATH_CONTROL_PERIOD_MS          2
#define PATH_MAX_POINTS                 16
/* Emm 速度模式加速度为 0 时直接响应目标速度；平移斜坡由软件正弦规划。 */
#define PATH_MOTOR_ACCEL_LEVEL          0U
/* 仅 MoveX/MoveY 单段路径使用的最低指令速度，多段 Path 不使用此下限。 */
#define PATH_MIN_COMMAND_SPEED_MM_S     30.0f

/* 仅为 MoveDistance 的旧 Trajectory 参数校验和规划保留。
 * MoveX/MoveY 的实际速度由 Path 正弦时间决定，不再调此值来调加速度。 */
#define CHASSIS_LEGACY_TRAJECTORY_ACCEL_MM_S2 300.0f

/* X、Y方向分别使用独立的实车距离标定系数。 */
#define DISTANCE_X_CALIBRATION          1.00
#define DISTANCE_Y_CALIBRATION          1.00

/* ==================== 直线航向保持 ==================== */
/* 移动段和旧距离接口共用比例项、陀螺仪角速度阻尼、死区及限幅。 */
#define HEADING_DEADBAND_DEG            0.2f
#define PATH_HEADING_KP                 6.0f
#define HEADING_GYRO_DAMPING_GAIN       0.3f
#define HEADING_MAX_CORRECTION           10.0f
#define HEADING_CORRECTION_SIGN         1

/* ==================== HWT101闭环转向 ==================== */
/* HWT101 航向闭环转向：角度误差比例调速，并限制最大/最小角速度。 */
#define TURN_DIRECTION_SIGN             1
#define TURN_MAX_SPEED_DEG_S            200
#define TURN_MIN_SPEED_DEG_S            5
#define TURN_KP                         1.6f
#define TURN_ACCELERATION_MM_S2         800
#define TURN_STOP_TOLERANCE_DEG         1.5
#define TURN_STOP_GYRO_DEG_S            2.0
#define TURN_STABLE_TIME_MS             200

/* 单次 Rotate 的 GongXun 航向 PID，输出单位为旋转轮速 RPM。 */
#define TURN_GX_KP                       4.0f
#define TURN_GX_KI                       0.0f
#define TURN_GX_KD                       3.0f
#define TURN_GX_OUTPUT_LIMIT_RPM         100.0f
#define TURN_GX_DEADBAND_DEG             0.5f
#define TURN_ANGLE_CALIBRATION           1.01411f

/* ==================== 小距离位置模式微调 ==================== */
/* 小距离位置模式微调参数，与长距离“速度模式+位置反馈判断”逻辑独立。 */
#define FINE_FORWARD_CALIBRATION        1.00
#define FINE_LATERAL_CALIBRATION        1.00
#define FINE_MOVE_MAX_DISTANCE_MM       150
#define FINE_HEADING_TOLERANCE_DEG      1.0
#define FINE_HEADING_SETTLE_MS          150
#define FINE_HEADING_MAX_CORRECTION_DEG 5.0

/* 四轮相对位置模式的到位判断与反馈查询。 */
#define CHASSIS_POSITION_UNITS_PER_REVOLUTION 65536.0f
#define CHASSIS_ARRIVAL_TOLERANCE       220
#define CHASSIS_ARRIVAL_CONFIRM_COUNT   3
#define CHASSIS_POSITION_POLL_INTERVAL_MS 10
#define CHASSIS_POSITION_SYNC_TIMEOUT_MS 100
#define CHASSIS_MOTOR_STABLE_DELAY_MS   100

#endif /* CHASSIS_CONFIG_H */
