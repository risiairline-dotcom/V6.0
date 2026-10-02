#include "test_chassis_control.h"
#include "button.h"
#include "chassis.h"
#include "stm32f4xx_hal.h"

#define CHASSIS_CONTROL_TEST_STOP_DELAY_MS 100U
#define CHASSIS_CONTROL_TEST_STEP_COUNT    6U

static uint8_t chassis_control_test_step;

void Chassis_Control_Test_Init(void)
{
  chassis_control_test_step = 0U;
  Chassis_Init();
}

void Chassis_Control_Test_Run(void)
{
  if (Button_GetStartEvent() == 0U)
  {
    return;
  }

  Chassis_Stop();

  if (chassis_control_test_step >= CHASSIS_CONTROL_TEST_STEP_COUNT)
  {
    return;
  }

  HAL_Delay(CHASSIS_CONTROL_TEST_STOP_DELAY_MS);

  switch (chassis_control_test_step)
  {
    case 0U:
      Chassis_Control( 100.0f,    0.0f,   0.0f);
      break;
    case 1U:
      Chassis_Control(-100.0f,    0.0f,   0.0f);
      break;
    case 2U:
      Chassis_Control(   0.0f,  100.0f,   0.0f);
      break;
    case 3U:
      Chassis_Control(   0.0f, -100.0f,   0.0f);
      break;
    case 4U:
      Chassis_Control(   0.0f,    0.0f,  30.0f);
      break;
    case 5U:
      Chassis_Control(   0.0f,    0.0f, -30.0f);
      break;
    default:
      break;
  }

  ++chassis_control_test_step;
}
