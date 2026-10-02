#include "mission_formal.h"
#include "mission.h"
#include "mission_route.h"
#include "mission_action_runner.h"
#include "chassis.h"
#include "hwt101.h"
#include "task_code.h"

/* 保留既有 Watch 符号和状态枚举；Formal 状态由本模块推进。 */
volatile MissionState_t mission_state = MISSION_IDLE;

static uint8_t formal_finish_stop_done;
static const MissionAction_t *formal_action_sequence;
static uint16_t formal_action_count;
static uint16_t formal_action_index;
static float formal_planned_yaw;

static const MissionAction_t *Mission_Formal_CurrentAction(void)
{
  if ((formal_action_sequence == 0) ||
      (formal_action_index >= formal_action_count))
    return 0;
  return &formal_action_sequence[formal_action_index];
}

static void Mission_Formal_EnterError(void)
{
  Chassis_Stop();
  mission_state = MISSION_ERROR;
}

void Mission_Formal_Init(void)
{
  mission_state = MISSION_IDLE;
  formal_finish_stop_done = 0;
  formal_action_sequence = 0;
  formal_action_count = 0;
  formal_action_index = 0;
  formal_planned_yaw = 0.0f;
  Mission_ActionRunner_Reset();
}

void Mission_Formal_Start(void)
{
  TaskCode_ClearReady();
  formal_planned_yaw = imu_continuous_yaw;
  formal_action_sequence = formal_route;
  formal_action_count = formal_route_count;
  formal_action_index = 0;
  formal_finish_stop_done = 0;
  Mission_ActionRunner_Reset();
  mission_state = MISSION_FORMAL_ROUTE_START;
}

void Mission_Formal_Run(void)
{
  const MissionAction_t *action;

  switch (mission_state)
  {
    case MISSION_IDLE:
      break;

    case MISSION_FORMAL_ROUTE_START:
      if ((formal_action_sequence == 0) || (formal_action_count == 0))
        Mission_Formal_EnterError();
      else
        mission_state = MISSION_ACTION_START;
      break;

    case MISSION_ACTION_START:
      action = Mission_Formal_CurrentAction();
      if ((Mission_ActionRunner_NeedsImu(action) != 0) &&
          (imu_zero_done == 0))
        break;
      if (Mission_ActionRunner_Start(action, &formal_planned_yaw) == CHASSIS_OK)
        mission_state = MISSION_ACTION_WAIT;
      else
        Mission_Formal_EnterError();
      break;

    case MISSION_ACTION_WAIT:
      action = Mission_Formal_CurrentAction();
      if (Mission_ActionRunner_IsDone(action) != 0)
        mission_state = MISSION_ACTION_FINISH;
      else if (Mission_ActionRunner_IsTimedOut(action) != 0)
        Mission_Formal_EnterError();
      break;

    case MISSION_ACTION_FINISH:
      Chassis_Stop();
      formal_action_index++;
      if (formal_action_index >= formal_action_count)
        mission_state = MISSION_FINISH;
      else
        mission_state = MISSION_ACTION_START;
      break;

    case MISSION_FINISH:
      if (formal_finish_stop_done == 0)
      {
        Chassis_Stop();
        formal_finish_stop_done = 1;
      }
      break;

    case MISSION_ERROR:
    default:
      break;
  }
}

void Mission_Formal_Stop(void)
{
  Chassis_Stop();
  mission_state = MISSION_IDLE;
  formal_finish_stop_done = 0;
  formal_action_sequence = 0;
  formal_action_count = 0;
  formal_action_index = 0;
  Mission_ActionRunner_Reset();
}

uint8_t Mission_Formal_IsFinished(void)
{
  return ((mission_state == MISSION_FINISH) &&
          (formal_finish_stop_done != 0)) ? 1U : 0U;
}
