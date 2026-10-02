#include "mission_action.h"
#include "chassis.h"
#include "heading_pid.h"
#include "stm32f4xx_hal.h"

#define MOVE_ACTION_SETTLE_MS    300U
#define ROTATE_ACTION_SETTLE_MS  300U

static uint32_t action_wait_start_tick;
static uint8_t action_done;

void Mission_Action_Reset(void)
{
  action_wait_start_tick = 0U;
  action_done = 0U;
}

uint8_t Mission_Action_Start(const MissionAction *action,
                             float rotate_target_deg)
{
  if (action == 0)
  {
    return 0U;
  }

  action_done = 0U;
  switch (action->type)
  {
    case ACTION_MOVE_X:
      Chassis_MoveX(action->value, action->speed_rpm, action->accel);
      /* 位移完成后停止并等待机械稳定。 */
      Chassis_Stop();
      HAL_Delay(MOVE_ACTION_SETTLE_MS);
      action_done = 1U;
      return 1U;

    case ACTION_MOVE_Y:
      Chassis_MoveY(action->value, action->speed_rpm, action->accel);
      /* 位移完成后停止并等待机械稳定。 */
      Chassis_Stop();
      HAL_Delay(MOVE_ACTION_SETTLE_MS);
      action_done = 1U;
      return 1U;

    case ACTION_ROTATE:
      (void)rotate_target_deg;
      Heading_RotateRelative(action->value);
      /* 旋转完成后停止并等待机械稳定。 */
      Chassis_Stop();
      HAL_Delay(ROTATE_ACTION_SETTLE_MS);
      action_done = 1U;
      return 1U;

    case ACTION_WAIT:
      action_wait_start_tick = HAL_GetTick();
      return 1U;

    default:
      return 0U;
  }
}

uint8_t Mission_Action_IsDone(const MissionAction *action)
{
  if (action == 0U)
  {
    return 0U;
  }
  if (action->type == ACTION_WAIT)
  {
    return ((HAL_GetTick() - action_wait_start_tick) >=
            (uint32_t)action->value) ? 1U : 0U;
  }
  return action_done;
}

