#include "mission_action.h"
#include "chassis.h"
#include "chassis_task.h"
#include "chassis_config.h"
#include "heading_pid.h"
#include "stm32f4xx_hal.h"

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
      {
        float speed_mm_s = (float)action->speed_rpm *
                           3.14159265358979323846f *
                           CHASSIS_WHEEL_DIAMETER / 60.0f;
        /* 启动非阻塞距离任务，并保持 Mission 规划航向。 */
        Chassis_Move_Start(speed_mm_s, 0.0f,
                           action->value, rotate_target_deg);
      }
      return 1U;

    case ACTION_MOVE_Y:
      {
        float speed_mm_s = (float)action->speed_rpm *
                           3.14159265358979323846f *
                           CHASSIS_WHEEL_DIAMETER / 60.0f;
        /* 启动非阻塞距离任务，并保持 Mission 规划航向。 */
        Chassis_Move_Start(0.0f, speed_mm_s,
                           action->value, rotate_target_deg);
      }
      return 1U;

    case ACTION_ROTATE:
      /* Mission 已将路线增量累计为绝对目标航向，直接交给绝对角控制器。 */
      Heading_RotateTo(rotate_target_deg);
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

/*
 * 功能：检查当前动作是否完成。
 * 参数：action，当前路线动作。
 * 返回：1 表示完成，0 表示仍在执行或参数无效。
 */
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
  if ((action->type == ACTION_MOVE_X) || (action->type == ACTION_MOVE_Y))
  {
    /* 当前仍以 Chassis_IsFinished() 判定完成；剩余距离接口供后续提前衔接使用。 */
    if ((action_done == 0U) && (Chassis_IsFinished() != 0U))
    {
      action_done = 1U;
    }
  }
  return action_done;
}

