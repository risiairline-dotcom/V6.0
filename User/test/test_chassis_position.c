#include "test_chassis_position.h"
#include "button.h"
#include "chassis.h"

static uint8_t chassis_position_test_step;

/* ==================== Y方向位置测试 ==================== */

void Chassis_Position_Test_Init(void)
{
  chassis_position_test_step = 0U;
  Chassis_Init();
}

void Chassis_Position_Test_Run(void)
{
  if (Button_GetStartEvent() == 0U)
  {
    return;
  }

  switch (chassis_position_test_step)
  {
    case 0U:
      /* 第1次：左移1000 mm */
      Chassis_MoveY( 1000.0f, 100U, 80U);
      chassis_position_test_step = 1U;
      break;
    case 1U:
      /* 第2次：右移1000 mm */
      Chassis_MoveY(-1000.0f, 100U, 80U);
      chassis_position_test_step = 2U;
      break;
    case 2U:
      /* 第3次：左移1000 mm */
      Chassis_MoveY( 1000.0f, 100U, 80U);
      chassis_position_test_step = 3U;
      break;
    case 3U:
      /* 第4次：右移1000 mm */
      Chassis_MoveY(-1000.0f, 100U, 80U);
      chassis_position_test_step = 4U;
      break;
    case 4U:
      /* 第5次：左移1000 mm */
      Chassis_MoveY( 1000.0f, 100U, 80U);
      chassis_position_test_step = 5U;
      break;
    case 5U:
      /* 第6次：右移1000 mm */
      Chassis_MoveY(-1000.0f, 100U, 80U);
      chassis_position_test_step = 6U;
      break;
    default:
      /* 第7次：停止 */
      Chassis_Stop();
      chassis_position_test_step = 7U;
      break;
  }
}
