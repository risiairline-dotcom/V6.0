#include "motor.h"
#include "emm42.h"
#include "chassis_config.h"
#include "stm32f4xx_hal.h"

/* [DEBUG] 以下快照和计数不参与电机控制结果。 */
volatile MotorPositionDebug_t motor_position_debug = {0};
volatile uint32_t motor_velocity_send_count[5] = {0};
volatile uint32_t motor_stop_send_count[5] = {0};
volatile uint32_t motor_velocity_last_tick[5] = {0};
volatile uint32_t motor_stop_last_tick[5] = {0};

static uint16_t Motor_ClampRpm(uint16_t rpm)
{
  return (rpm > MOTOR_MAX_RPM) ? MOTOR_MAX_RPM : rpm;
}

static uint8_t Motor_ClampAcceleration(uint8_t level)
{
  return (level < MOTOR_ACCEL_LEVEL_MIN) ? MOTOR_ACCEL_LEVEL_MIN : level;
}

void Motor_Init(void)
{
  /* 当前底层驱动无额外的软件状态，保留此入口方便后续增加反馈与故障管理。 */
}

MotorStatus_t Motor_Enable(uint8_t id)
{
  EMM42_Enable(id, true, EMM42_SYNC_NOW);
  return MOTOR_OK;
}

MotorStatus_t Motor_Disable(uint8_t id)
{
  EMM42_Enable(id, false, EMM42_SYNC_NOW);
  return MOTOR_OK;
}

MotorStatus_t Motor_Stop(uint8_t id)
{
  /* 数组下标1至4对应四个CAN电机地址，下标0保留。 */
  if (id < 5)
    motor_stop_send_count[id]++;
  EMM42_Stop(id);
  if (id < 5)
    motor_stop_last_tick[id] = HAL_GetTick();
  return MOTOR_OK;
}

MotorStatus_t Motor_Move_Velocity(uint8_t id, int16_t signed_rpm,
                                  uint8_t acceleration_level, uint8_t sync)
{
  uint16_t rpm;
  uint8_t direction;

  /* 记录本次速度命令调用，不改变原有速度换算和发送参数。 */
  if (id < 5)
    motor_velocity_send_count[id]++;

  direction = (signed_rpm < 0) ? EMM42_DIR_CCW : EMM42_DIR_CW;
  rpm = (signed_rpm < 0) ? (uint16_t)(-(int32_t)signed_rpm) : (uint16_t)signed_rpm;
  /* 速度模式允许透传协议0档；位置模式仍使用原有最小加速度限制。 */
  EMM42_VelocityControl(id, direction, Motor_ClampRpm(rpm),
                        acceleration_level, sync != 0);
  if (id < 5)
    motor_velocity_last_tick[id] = HAL_GetTick();
  return MOTOR_OK;
}

MotorStatus_t Motor_Move_Position(uint8_t id, float position_deg, uint16_t speed_rpm, uint8_t acceleration_level, uint8_t sync)
{
  float absolute_degree;
  float pulse_float;
  uint32_t pulse;
  uint8_t direction;

  direction = (position_deg < 0.0f) ? EMM42_DIR_CCW : EMM42_DIR_CW;
  absolute_degree = (position_deg < 0.0f) ? -position_deg : position_deg;
  pulse_float = absolute_degree * MOTOR_PULSES_PER_REVOLUTION / 360.0f;
  if (pulse_float > 4294967295.0f)
  {
    return MOTOR_ERROR_ARGUMENT;
  }
  pulse = (uint32_t)(pulse_float + 0.5f);
  speed_rpm = Motor_ClampRpm(speed_rpm);
  acceleration_level = Motor_ClampAcceleration(acceleration_level);

  /*
   * 调试器 Watch 示例：Motor_Move_Position(1, 1000, 100, 10, 0)
   * id=1，position_deg=1000，pulse=8889，speed_rpm=100，acc=10，dir=CW，sync=0。
   */
  motor_position_debug.id = id;
  motor_position_debug.position_deg = position_deg;
  motor_position_debug.pulse = pulse;
  motor_position_debug.speed_rpm = speed_rpm;
  motor_position_debug.acc = acceleration_level;
  motor_position_debug.dir = direction;
  motor_position_debug.sync = (uint8_t)(sync != 0);

  EMM42_PositionControl(id, direction, speed_rpm, acceleration_level, pulse, EMM42_POS_RELATIVE, sync != 0);
  return MOTOR_OK;
}

MotorStatus_t Motor_Sync_Start(void)
{
  EMM42_Multi_Motor_Cmd(EMM42_SYNC_NOW);
  return MOTOR_OK;
}
