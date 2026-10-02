#include "mission_action_runner.h"
#include "mission_fast_block.h"
#include "task_code.h"
#include "stm32f4xx_hal.h"

static uint32_t action_wait_start_tick;

void Mission_ActionRunner_Reset(void)
{
  action_wait_start_tick = 0;
}

uint8_t Mission_ActionRunner_NeedsImu(const MissionAction_t *action)
{
  ActionType_t type;

  if (action == 0)
    return 0;

  type = action->type;
  return ((type == ACTION_MOVE) ||
          (type == ACTION_FAST_MOVE) ||
          (type == ACTION_ROTATE)) ? 1 : 0;
}

ChassisStatus_t Mission_ActionRunner_Start(const MissionAction_t *action,
                                           float *planned_yaw)
{
  ChassisStatus_t status;
  Path_t path;
  uint16_t block_index;

  if ((action == 0) || (planned_yaw == 0))
    return CHASSIS_ERROR_ARGUMENT;

  switch (action->type)
  {
    case ACTION_MOVE:
      if ((action->param1 != 0.0f) && (action->param2 == 0.0f))
      {
        Chassis_SetMoveHeadingTarget(*planned_yaw);
        return Chassis_MoveX(action->param1, action->speed);
      }
      if ((action->param2 != 0.0f) && (action->param1 == 0.0f))
      {
        Chassis_SetMoveHeadingTarget(*planned_yaw);
        return Chassis_MoveY(action->param2, action->speed);
      }
      return CHASSIS_ERROR_ARGUMENT;

    case ACTION_ROTATE:
      status = Chassis_Rotate(action->param1, action->speed);
      if (status == CHASSIS_OK)
        *planned_yaw += action->param1;
      return status;

    case ACTION_WAIT:
      Chassis_Stop();
      action_wait_start_tick = HAL_GetTick();
      return CHASSIS_OK;

    case ACTION_SCAN:
      Chassis_Stop();
      action_wait_start_tick = HAL_GetTick();
      return CHASSIS_OK;

    case ACTION_FAST_MOVE:
      if (!(action->param1 >= 0.0f && action->param1 <= 65535.0f))
        return CHASSIS_ERROR_ARGUMENT;
      block_index = (uint16_t)action->param1;
      if (((float)block_index != action->param1) ||
          (Mission_FastBlock_GetPath(block_index, &path) == 0U))
        return CHASSIS_ERROR_ARGUMENT;
      return Chassis_MovePath(&path);

    case ACTION_GRAB:
    case ACTION_PLACE:
    case ACTION_FINISH:
      /* 仅预留接口；当前正式路线不包含这些动作。 */
      return CHASSIS_ERROR_ARGUMENT;

    default:
      return CHASSIS_ERROR_ARGUMENT;
  }
}

uint8_t Mission_ActionRunner_IsDone(const MissionAction_t *action)
{
  if (action == 0)
    return 0;

  switch (action->type)
  {
    case ACTION_MOVE:
      return Chassis_IsArrived();

    case ACTION_ROTATE:
      return Chassis_IsTurnDone();

    case ACTION_WAIT:
      return ((HAL_GetTick() - action_wait_start_tick) >= action->timeout_ms) ? 1 : 0;

    case ACTION_SCAN:
      return TaskCode_IsReady();

    case ACTION_FAST_MOVE:
      return Chassis_IsPathDone();

    case ACTION_GRAB:
    case ACTION_PLACE:
    case ACTION_FINISH:
      return 0;

    default:
      return 0;
  }
}

uint8_t Mission_ActionRunner_IsTimedOut(const MissionAction_t *action)
{
  if ((action == 0) || (action->type != ACTION_SCAN) ||
      (action->timeout_ms == 0U) || (TaskCode_IsReady() != 0U))
  {
    return 0U;
  }

  return ((HAL_GetTick() - action_wait_start_tick) >= action->timeout_ms) ? 1U : 0U;
}
