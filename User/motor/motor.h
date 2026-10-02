#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

typedef enum
{
  MOTOR_OK = 0,
  MOTOR_ERROR_ARGUMENT,
  MOTOR_ERROR_CAN
} MotorStatus_t;

/* [DEBUG] 位置命令调试快照，可在调试器 Watch 窗口中直接查看。 */
typedef struct
{
  uint8_t id;
  float position_deg;
  uint32_t pulse;
  uint16_t speed_rpm;
  uint8_t acc;
  uint8_t dir;
  uint8_t sync;
} MotorPositionDebug_t;

extern volatile MotorPositionDebug_t motor_position_debug;

/* [DEBUG] 每个CAN地址的速度、停止命令调用次数，供Keil Watch观察。 */
extern volatile uint32_t motor_velocity_send_count[5];
extern volatile uint32_t motor_stop_send_count[5];

/* 每个CAN地址最后一次速度、停止命令的系统时刻，供Keil Watch观察。 */
extern volatile uint32_t motor_velocity_last_tick[5];
extern volatile uint32_t motor_stop_last_tick[5];

/* 初始化电机应用层；CAN 外设应先在 main 中启动。 */
void Motor_Init(void);

/* 使能、停止指定地址的 X42S 电机。 */
MotorStatus_t Motor_Enable(uint8_t id);
MotorStatus_t Motor_Disable(uint8_t id);
MotorStatus_t Motor_Stop(uint8_t id);

/* 速度控制：signed_rpm 正负号决定方向，加速度使用 X42S 的 0~255 档。 */
MotorStatus_t Motor_Move_Velocity(uint8_t id, int16_t signed_rpm, uint8_t acceleration_level, uint8_t sync);

/* 相对位置控制：position_deg 为电机轴角度，速度单位 RPM。 */
MotorStatus_t Motor_Move_Position(uint8_t id, float position_deg, uint16_t speed_rpm, uint8_t acceleration_level, uint8_t sync);

/* 同步启动所有已经缓存的电机命令。 */
MotorStatus_t Motor_Sync_Start(void);

#endif /* MOTOR_H */
