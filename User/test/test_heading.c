#include "test_heading.h"
#include "button.h"
#include "chassis.h"
#include "heading_pid.h"
#include "hwt101.h"
#include "stm32f4xx_hal.h"

static uint8_t heading_test_step;

/* ==================== 航向位置测试 ==================== */

void Heading_Test_Init(void)
{
  heading_test_step = 0U;
  Heading_Init();
  Chassis_Init();
}

void Heading_Test_Run(void)
{
  if (Button_GetStartEvent() == 0U)
  {
    return;
  }

  switch (heading_test_step)
  {
    case 0U:
      /* 第1次：建立当前方向为0°，原地左转90° */
      Chassis_Stop();
      IMU_ZeroYaw();
      HAL_Delay(10U);
      Heading_RotateTo(90.0f);
      heading_test_step = 1U;
      break;
    case 1U:
      /* 第2次：原地右转回到0° */
      Heading_RotateTo(0.0f);
      heading_test_step = 2U;
      break;
    case 2U:
      /* 第3次：原地左转到180° */
      Heading_RotateTo(180.0f);
      heading_test_step = 3U;
      break;
    case 3U:
      /* 第4次：原地右转回到0° */
      Heading_RotateTo(0.0f);
      heading_test_step = 4U;
      break;
    default:
      /* 第5次及以后：停止 */
      Chassis_Stop();
      heading_test_step = 5U;
      break;
  }
}
