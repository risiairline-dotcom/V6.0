#include "mission.h"
#include "mission_config.h"
#include "mission_formal.h"
#include "mission_test_path.h"
#include "mission_test_odometry.h"
#include "mission_test_rectangle.h"
#include "mission_test_fast_block.h"
#include "chassis.h"

volatile uint8_t mission_started = 0;
volatile uint8_t mission_finished = 0;

static void Mission_EnterError(void)
{
  Chassis_Stop();
  mission_state = MISSION_ERROR;
  mission_started = 0;
  mission_finished = 0;
}

void Mission_Init(void)
{
  Mission_Formal_Init();
  mission_started = 0;
  mission_finished = 0;
  Mission_TestPath_Init();
  Mission_TestOdometry_Init();
  Mission_TestRectangle_Init();
  Mission_TestFastBlock_Init();
}

void Mission_Start(void)
{
  if ((mission_started != 0) && (mission_finished == 0))
    return;

  mission_state = MISSION_IDLE;
  mission_started = 1;
  mission_finished = 0;
  switch (MISSION_MODE)
  {
    case MISSION_MODE_RECTANGLE_TEST:
      Mission_TestRectangle_Start();
      break;
    case MISSION_MODE_ODOMETRY_TEST:
      Mission_TestOdometry_Start();
      break;
    case MISSION_MODE_PATH_TEST:
      Mission_TestPath_Start();
      break;
    case MISSION_MODE_FAST_BLOCK_TEST:
      Mission_TestFastBlock_Start();
      break;
    case MISSION_MODE_FORMAL:
      Mission_Formal_Start();
      break;
    default:
      Mission_EnterError();
      break;
  }
}

void Mission_Stop(void)
{
  switch (MISSION_MODE)
  {
    case MISSION_MODE_RECTANGLE_TEST:
      Mission_TestRectangle_Stop();
      break;
    case MISSION_MODE_ODOMETRY_TEST:
      Mission_TestOdometry_Stop();
      break;
    case MISSION_MODE_PATH_TEST:
      Mission_TestPath_Stop();
      break;
    case MISSION_MODE_FAST_BLOCK_TEST:
      Mission_TestFastBlock_Stop();
      break;
    case MISSION_MODE_FORMAL:
      Mission_Formal_Stop();
      break;
    default:
      Chassis_Stop();
      break;
  }
  mission_state = MISSION_IDLE;
  mission_started = 0;
  mission_finished = 0;
}

void Mission_Run(void)
{
  if (mission_started == 0)
    return;

  Chassis_Run();

  switch (MISSION_MODE)
  {
    case MISSION_MODE_RECTANGLE_TEST:
      Mission_TestRectangle_Run();
      if (Mission_TestRectangle_IsFinished() != 0)
      {
        mission_finished = 1;
        mission_started = 0;
      }
      return;
    case MISSION_MODE_ODOMETRY_TEST:
      Mission_TestOdometry_Run();
      if (Mission_TestOdometry_IsFinished() != 0)
      {
        mission_finished = 1;
        mission_started = 0;
      }
      return;
    case MISSION_MODE_PATH_TEST:
      Mission_TestPath_Run();
      if (Mission_TestPath_IsFinished() != 0)
      {
        mission_finished = 1;
        mission_started = 0;
      }
      return;
    case MISSION_MODE_FAST_BLOCK_TEST:
      Mission_TestFastBlock_Run();
      if (Mission_TestFastBlock_IsFinished() != 0)
      {
        mission_finished = 1;
        mission_started = 0;
      }
      return;
    case MISSION_MODE_FORMAL:
      Mission_Formal_Run();
      if (Mission_Formal_IsFinished() != 0)
        mission_finished = 1;
      if (mission_state == MISSION_ERROR)
      {
        mission_started = 0;
        mission_finished = 0;
      }
      return;
    default:
      Mission_EnterError();
      return;
  }

}

uint8_t Mission_IsFinished(void)
{
  return mission_finished;
}

uint8_t Mission_IsRunning(void)
{
  return (mission_started != 0) &&
         (mission_finished == 0) &&
         (mission_state != MISSION_ERROR);
}
