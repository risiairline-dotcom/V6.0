#include "mission_test_fast_block.h"
#include "mission.h"
#include "mission_action_runner.h"
#include "chassis.h"
#include "hwt101.h"

/* 测试动作只引用 Block 0；正式动作表保持原样。 */
static const MissionAction_t fast_block_test_action =
{
  ACTION_FAST_MOVE, 0.0f, 0.0f, 0.0f, 0.0f, 0U
};

typedef enum
{
  FAST_BLOCK_TEST_IDLE = 0,
  FAST_BLOCK_TEST_WAIT_IMU,
  FAST_BLOCK_TEST_START,
  FAST_BLOCK_TEST_RUN,
  FAST_BLOCK_TEST_FINISH
} FastBlockTestState_t;

volatile uint8_t fast_block_test_path_started = 0U;
volatile uint8_t fast_block_test_done = 0U;
volatile uint8_t fast_block_test_error = 0U;

static FastBlockTestState_t fast_block_test_state;
static float fast_block_test_planned_yaw;

void Mission_TestFastBlock_Init(void)
{
  fast_block_test_state = FAST_BLOCK_TEST_IDLE;
  fast_block_test_path_started = 0U;
  fast_block_test_done = 0U;
  fast_block_test_error = 0U;
}

void Mission_TestFastBlock_Start(void)
{
  if ((fast_block_test_state != FAST_BLOCK_TEST_IDLE) &&
      (fast_block_test_state != FAST_BLOCK_TEST_FINISH))
    return;

  Mission_ActionRunner_Reset();
  fast_block_test_path_started = 0U;
  fast_block_test_done = 0U;
  fast_block_test_error = 0U;
  fast_block_test_planned_yaw = imu_continuous_yaw;
  fast_block_test_state = (imu_zero_done != 0) ?
      FAST_BLOCK_TEST_START : FAST_BLOCK_TEST_WAIT_IMU;
}

void Mission_TestFastBlock_Stop(void)
{
  Chassis_Stop();
  fast_block_test_state = FAST_BLOCK_TEST_IDLE;
}

void Mission_TestFastBlock_Run(void)
{
  switch (fast_block_test_state)
  {
    case FAST_BLOCK_TEST_WAIT_IMU:
      if (imu_zero_done != 0)
        fast_block_test_state = FAST_BLOCK_TEST_START;
      break;

    case FAST_BLOCK_TEST_START:
      if (Mission_ActionRunner_NeedsImu(&fast_block_test_action) != 0 &&
          imu_zero_done == 0)
        break;
      if (Mission_ActionRunner_Start(&fast_block_test_action,
                                     &fast_block_test_planned_yaw) == CHASSIS_OK)
      {
        fast_block_test_path_started = 1U;
        fast_block_test_state = FAST_BLOCK_TEST_RUN;
      }
      else
      {
        Chassis_Stop();
        fast_block_test_error = 1U;
        fast_block_test_state = FAST_BLOCK_TEST_FINISH;
      }
      break;

    case FAST_BLOCK_TEST_RUN:
      if (Mission_ActionRunner_IsDone(&fast_block_test_action) != 0)
      {
        fast_block_test_done = 1U;
        fast_block_test_state = FAST_BLOCK_TEST_FINISH;
      }
      break;

    case FAST_BLOCK_TEST_IDLE:
    case FAST_BLOCK_TEST_FINISH:
    default:
      break;
  }
}

uint8_t Mission_TestFastBlock_IsFinished(void)
{
  return (fast_block_test_state == FAST_BLOCK_TEST_FINISH) ? 1U : 0U;
}
