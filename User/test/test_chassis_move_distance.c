#include "test_chassis_move_distance.h"
#include "button.h"
#include "chassis.h"
#include "hwt101.h"

static uint8_t chassis_move_distance_test_done;

void Chassis_MoveDistance_Test_Init(void)
{
  chassis_move_distance_test_done = 0U;
  (void)HWT101_Init();
  Chassis_Init();
}

void Chassis_MoveDistance_Test_Run(void)
{
  if ((chassis_move_distance_test_done != 0U) ||
      (Button_GetStartEvent() == 0U))
  {
    return;
  }

  /* X方向移动1000 mm，并保持开始时的航向。 */
  Chassis_MoveDistance(500.0f, 0.0f, 1000.0f, IMU_GetYaw());
  chassis_move_distance_test_done = 1U;
}
