#include "lift.h"
#include "motor.h"
#include "emm42.h"
#include "stm32f4xx_hal.h"
#include <math.h>

#define LIFT_MOTOR_ID            5
#define LIFT_MOVE_SPEED_RPM      30
#define LIFT_MOVE_ACCEL_LEVEL    1
/* 机械标定阶段单次相对运动上限，仍禁止超过 5000°。 */
#define LIFT_MAX_STEP_ANGLE      5000
#define LIFT_FEEDBACK_TIMEOUT_MS 50
#define LIFT_POLL_INTERVAL_MS    50
#define LIFT_POSITION_UNITS_REV  65536
#define LIFT_ARRIVAL_TOLERANCE   200
/* 实测 100° 电机轴角度对应约 12 mm 升降位移。 */
#define LIFT_CALIBRATION_DEG     100.0f
#define LIFT_CALIBRATION_MM      12.0f
#define LIFT_MAX_RELATIVE_MM     50.0f

static int32_t lift_start_position;
static uint32_t lift_target_travel;
static uint32_t lift_last_poll_tick;
static uint8_t lift_ready;
static uint8_t lift_moving;
static uint8_t lift_arrived;
static int32_t lift_zero_position;
static int8_t lift_feedback_sign;
static int8_t lift_move_sign;
static uint8_t lift_zero_valid;

static void Lift_UpdateFeedbackSign(int32_t current_position)
{
  int64_t delta;

  if ((lift_moving == 0) || (lift_feedback_sign != 0))
    return;

  delta = (int64_t)current_position - lift_start_position;
  if (delta >= LIFT_ARRIVAL_TOLERANCE)
    lift_feedback_sign = lift_move_sign;
  else if (delta <= -LIFT_ARRIVAL_TOLERANCE)
    lift_feedback_sign = -lift_move_sign;
}

uint8_t Lift_GetPosition(int32_t *position)
{
  int32_t current_position;

  if ((position == 0) ||
      !EMM42_Read_Position_Confirmed(LIFT_MOTOR_ID, &current_position,
                                    LIFT_FEEDBACK_TIMEOUT_MS))
    return 0;

  *position = current_position;
  return 1;
}

uint8_t Lift_Init(void)
{
  int32_t current_position;

  lift_ready = 0;
  lift_moving = 0;
  lift_arrived = 0;
  lift_zero_valid = 0;
  lift_feedback_sign = 0;
  lift_move_sign = 0;
  if (Motor_Enable(LIFT_MOTOR_ID) != MOTOR_OK)
    return 0;
  if (Lift_GetPosition(&current_position) == 0)
    return 0;

  lift_start_position = current_position;
  lift_ready = 1;
  lift_arrived = 1;
  return 1;
}

uint8_t Lift_MoveRelative(int32_t angle)
{
  int32_t current_position;
  uint32_t angle_magnitude;

  /* 尚无机械零点和限位开关，单次命令必须受标定上限约束。 */
  if ((lift_ready == 0) || (lift_moving != 0) || (angle == 0) ||
      (angle > LIFT_MAX_STEP_ANGLE) || (angle < -LIFT_MAX_STEP_ANGLE))
    return 0;
  if (Lift_GetPosition(&current_position) == 0)
    return 0;

  angle_magnitude = (angle < 0) ? (uint32_t)(-angle) : (uint32_t)angle;
  if (Motor_Move_Position(LIFT_MOTOR_ID, (float)angle, LIFT_MOVE_SPEED_RPM,
                          LIFT_MOVE_ACCEL_LEVEL, 0) != MOTOR_OK)
    return 0;

  lift_start_position = current_position;
  /* 0x36 反馈为每圈 65536 单位，与命令的 3200 脉冲/圈不同。 */
  lift_target_travel = (angle_magnitude * LIFT_POSITION_UNITS_REV + 180) / 360;
  lift_last_poll_tick = HAL_GetTick();
  lift_arrived = 0;
  lift_move_sign = (angle > 0) ? 1 : -1;
  lift_moving = 1;
  return 1;
}

uint8_t Lift_MoveRelativeMm(float distance_mm)
{
  int32_t angle;

  /* 尚无机械零点和限位，毫米接口单次不得超过 50 mm。 */
  if (!(distance_mm >= -LIFT_MAX_RELATIVE_MM &&
        distance_mm <= LIFT_MAX_RELATIVE_MM))
    return 0;

  /* 向零取整，避免角度取整后超出请求的毫米位移。 */
  angle = (int32_t)(distance_mm * LIFT_CALIBRATION_DEG /
                    LIFT_CALIBRATION_MM);
  return Lift_MoveRelative(angle);
}

uint8_t Lift_SetZero(void)
{
  int32_t current_position;

  /* 调试零点仅来自当前反馈，运动中不重设。 */
  if ((lift_ready == 0) || (lift_moving != 0) ||
      (Lift_GetPosition(&current_position) == 0))
    return 0;

  lift_zero_position = current_position;
  lift_zero_valid = 1;
  return 1;
}

float Lift_GetHeight(void)
{
  int32_t current_position;
  int64_t delta;

  if ((lift_zero_valid == 0) || (Lift_GetPosition(&current_position) == 0))
    return NAN;

  Lift_UpdateFeedbackSign(current_position);
  delta = (int64_t)current_position - lift_zero_position;
  if (delta == 0)
    return 0.0f;
  if (lift_feedback_sign == 0)
    return NAN;

  /* 先由反馈单位换算电机轴角度，再取反为向上为正的高度。 */
  return -(float)delta * lift_feedback_sign * 360.0f /
         LIFT_POSITION_UNITS_REV * LIFT_CALIBRATION_MM /
         LIFT_CALIBRATION_DEG;
}

uint8_t Lift_MoveHeight(float height_mm)
{
  float current_height;
  float distance_mm;

  if (lift_moving != 0)
    return 0;
  current_height = Lift_GetHeight();
  if (!(current_height == current_height && height_mm == height_mm))
    return 0;

  /* 高度向上为正；相对毫米命令仍以向下为正。 */
  distance_mm = current_height - height_mm;
  if (distance_mm == 0.0f)
    return 1;
  return Lift_MoveRelativeMm(distance_mm);
}

void Lift_Stop(void)
{
  (void)Motor_Stop(LIFT_MOTOR_ID);
  lift_moving = 0;
  lift_arrived = 0;
}

uint8_t Lift_IsArrived(void)
{
  int32_t current_position;
  int64_t travel;

  if (lift_moving == 0)
    return lift_arrived;
  if ((uint32_t)(HAL_GetTick() - lift_last_poll_tick) < LIFT_POLL_INTERVAL_MS)
    return 0;

  lift_last_poll_tick = HAL_GetTick();
  if (Lift_GetPosition(&current_position) == 0)
    return 0;

  Lift_UpdateFeedbackSign(current_position);
  /* 反馈数值的符号不参与到位判定，比较本次原始反馈位移绝对值。 */
  travel = (int64_t)current_position - lift_start_position;
  if (travel < 0)
    travel = -travel;
  if ((uint64_t)travel + LIFT_ARRIVAL_TOLERANCE >= lift_target_travel)
  {
    lift_moving = 0;
    lift_arrived = 1;
  }
  return lift_arrived;
}
