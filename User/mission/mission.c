#include "mission.h"
#include "mission_action.h"
#include "mission_route.h"
#include "chassis.h"
#include "hwt101.h"

volatile MissionState_t mission_state = MISSION_IDLE;
volatile uint8_t mission_started = 0U;
volatile uint8_t mission_finished = 0U;

static uint8_t mission_action_index;
static float mission_target_yaw;

static float Mission_WrapYaw(float yaw)
{
  while (yaw > 180.0f)
  {
    yaw -= 360.0f;
  }
  while (yaw < -180.0f)
  {
    yaw += 360.0f;
  }
  return yaw;
}

void Mission_Init(void)
{
  mission_state = MISSION_IDLE;
  mission_started = 0U;
  mission_finished = 0U;
  mission_action_index = 0U;
  Mission_Action_Reset();
}

void Mission_Start(void)
{
  if ((MissionRoute_IsReady() == 0U) ||
      (MissionRoute_GetCount() == 0U))
  {
    Chassis_Stop();
    mission_state = MISSION_ERROR;
    mission_started = 0U;
    mission_finished = 0U;
    return;
  }

  /* 正式路线开始前，将当前车头方向设为0°。 */
  IMU_ZeroYaw();
  mission_target_yaw = 0.0f;
  mission_action_index = 0U;
  mission_started = 1U;
  mission_finished = 0U;
  mission_state = MISSION_ACTION_START;
  Mission_Action_Reset();
}

void Mission_Stop(void)
{
  Chassis_Stop();
  mission_state = MISSION_IDLE;
  mission_started = 0U;
  mission_finished = 0U;
  Mission_Action_Reset();
}

/*
 * 功能：推进一次 Mission 状态机。
 * 参数：无；当前动作由内部路线索引取得。
 * 返回：无；动作完成后切换到下一动作或结束状态。
 */
void Mission_Run(void)
{
  const MissionAction *action;

  if (mission_started == 0U)
  {
    return;
  }

  switch (mission_state)
  {
    case MISSION_ACTION_START:
      action = MissionRoute_GetAction(mission_action_index);
      if (action != 0 && action->type == ACTION_ROTATE)
      {
        mission_target_yaw = Mission_WrapYaw(mission_target_yaw +
                                              action->value);
      }
      if (Mission_Action_Start(action, mission_target_yaw) == 0U)
      {
        Chassis_Stop();
        mission_state = MISSION_ERROR;
        mission_started = 0U;
        return;
      }
      mission_state = MISSION_ACTION_WAIT;
      break;

    case MISSION_ACTION_WAIT:
      action = MissionRoute_GetAction(mission_action_index);
      if (Mission_Action_IsDone(action) != 0U)
      {
        ++mission_action_index;
        if (mission_action_index >= MissionRoute_GetCount())
        {
          mission_state = MISSION_FINISH;
        }
        else
        {
          mission_state = MISSION_ACTION_START;
        }
      }
      break;

    case MISSION_FINISH:
      Chassis_Stop();
      mission_started = 0U;
      mission_finished = 1U;
      break;

    default:
      Chassis_Stop();
      mission_state = MISSION_ERROR;
      mission_started = 0U;
      break;
  }
}

uint8_t Mission_IsRunning(void)
{
  return mission_started;
}

uint8_t Mission_IsFinished(void)
{
  return mission_finished;
}

