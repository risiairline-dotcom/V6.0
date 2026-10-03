#ifndef CHASSIS_CONFIG_H
#define CHASSIS_CONFIG_H

/* Existing wheel geometry and installation mapping. */
#define CHASSIS_LENGTH                 188.0f
#define CHASSIS_WIDTH                  220.0f
#define CHASSIS_WHEEL_DIAMETER         100.0f
#define ROTATE_FACTOR                  ((CHASSIS_LENGTH + CHASSIS_WIDTH) / 2.0f)
#define CHASSIS_X_DISTANCE_SCALE       1.000f
#define CHASSIS_Y_DISTANCE_SCALE       1.000f
#define CHASSIS_FORWARD_SIGN           (-1)
#define CHASSIS_LATERAL_SIGN           (-1)

/* Motor protocol limits shared with the mechanism axes. */
#define MOTOR_PULSES_PER_REVOLUTION    3200.0f
#define MOTOR_MAX_RPM                  3000
#define MOTOR_ACCEL_LEVEL_MIN          1
#define CHASSIS_MIN_COMMAND_RPM        1

#define MOTOR_M1_ID                    2
#define MOTOR_M2_ID                    1
#define MOTOR_M3_ID                    3
#define MOTOR_M4_ID                    4
#define MOTOR_M1_SIGN                  (-1)
#define MOTOR_M2_SIGN                  1
#define MOTOR_M3_SIGN                  (-1)
#define MOTOR_M4_SIGN                  1

#define CHASSIS_MOTOR_STOP_INTERVAL_MS 10U
#define CHASSIS_MOTOR_STABLE_DELAY_MS  10U

/* X/Y 方向移动默认参数；调用时传入 0 表示使用对应方向默认值。 */
#define CHASSIS_MOVE_X_SPEED_DEFAULT   150U
#define CHASSIS_MOVE_X_ACCEL_DEFAULT   100U
#define CHASSIS_MOVE_Y_SPEED_DEFAULT   200U
#define CHASSIS_MOVE_Y_ACCEL_DEFAULT   100U

/* 移动航向保持参数。 */
#define MOVE_HEADING_KP                1.5f
#define MOVE_HEADING_KD                0.08f
#define MOVE_HEADING_DEADZONE          0.5f
#define MOVE_HEADING_MAX_WZ            10.0f
#define CHASSIS_MOVE_HEADING_KP        1.1f
#define CHASSIS_MOVE_HEADING_KI        0.01f
#define CHASSIS_MOVE_HEADING_KD        0.15f

/* 移动控制周期。 */
#define MOVE_CONTROL_PERIOD_MS         50U
#define MOVE_POSITION_SYNC_TIMEOUT_MS  150U
/* 速度规划参数，单位分别为 mm/s^2 和 mm/s^3。 */
#define MOVE_ACCEL_LIMIT               800.0f
#define MOVE_DECEL_LIMIT               500.0f
#define MOVE_JERK_LIMIT                4000.0f
#define MOVE_STOP_DISTANCE_MM          78.52f
#define MOVE_FINAL_STOP_SPEED_MM_S     20.0f

/* 位置误差速度补偿参数，速度单位为 mm/s。 */
#define CHASSIS_POSITION_KP            0.07f
#define CHASSIS_POSITION_SPEED_LIMIT   100U

/* EMM42 位置反馈每转的计数；用于换算轮面距离。 */
#define CHASSIS_FEEDBACK_UNITS_PER_REVOLUTION 65536.0f
#define CHASSIS_POSITION_TIME_FACTOR  2.0f
#define CHASSIS_POSITION_SETTLE_MARGIN_MS 1000U

/* 独立旋转航向 PID 参数。调车时只需修改本配置文件。 */
#define HEADING_KP                    3.5f
#define HEADING_KI                    0.0f
#define HEADING_KD                    2.0f
#define HEADING_MAX_INTEGRAL          7.0f
#define HEADING_MAX_OUTPUT            100.0f
#define HEADING_CONTROL_PERIOD_MS     50U
#define HEADING_STABLE_ERROR_DEG      1.5f
#define HEADING_STABLE_GYRO_DPS      3.0f
#define HEADING_STABLE_COUNT          5U

#endif /* CHASSIS_CONFIG_H */
