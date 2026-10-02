#include "extend.h"
#include "motor.h"
#include "emm42.h"

#define EXTEND_MOTOR_ID            6
#define EXTEND_MOVE_SPEED_RPM      30
#define EXTEND_MOVE_ACCEL_LEVEL    1
#define EXTEND_FEEDBACK_TIMEOUT_MS 50
#define EXTEND_ARRIVAL_TOLERANCE   200

static uint8_t extend_ready;
static uint8_t extend_command_accepted;

uint8_t Extend_IsReady(void)
{
  return extend_ready;
}

uint8_t Extend_CommandAccepted(void)
{
  return extend_command_accepted;
}

int32_t Extend_GetPosition(void)
{
  int32_t position;

  if (!EMM42_Read_Position_Confirmed(EXTEND_MOTOR_ID, &position,
                                    EXTEND_FEEDBACK_TIMEOUT_MS))
    return INT32_MIN;
  return position;
}

void Extend_Init(void)
{
  extend_ready = 0;
  extend_command_accepted = 0;
  if (Motor_Enable(EXTEND_MOTOR_ID) != MOTOR_OK)
    return;
  if (Extend_GetPosition() != INT32_MIN)
    extend_ready = 1;
}

void Extend_MoveRelative(int32_t angle)
{
  extend_command_accepted = 0;
  /* 没有机械零点和限位；反馈未确认时不发运动命令。 */
  if ((extend_ready == 0) || (angle == 0) ||
      (Extend_GetPosition() == INT32_MIN))
    return;

  if (Motor_Move_Position(EXTEND_MOTOR_ID, (float)angle,
                          EXTEND_MOVE_SPEED_RPM,
                          EXTEND_MOVE_ACCEL_LEVEL, 0) == MOTOR_OK)
    extend_command_accepted = 1;
}

void Extend_Stop(void)
{
  (void)Motor_Stop(EXTEND_MOTOR_ID);
  extend_command_accepted = 0;
}

uint8_t Extend_IsArrived(int32_t target)
{
  int32_t position;
  int64_t error;

  position = Extend_GetPosition();
  if (position == INT32_MIN)
    return 0;

  error = (int64_t)position - target;
  if (error < 0)
    error = -error;
  return (error <= EXTEND_ARRIVAL_TOLERANCE) ? 1 : 0;
}
