#include "test_lift.h"
#include "lift.h"
#include "button.h"

/* 相对调试零点向上的目标高度。 */
#define LIFT_TEST_HEIGHT_MM  50

/* 供 Keil Watch 查看新反馈读取结果与每次动作后的位置。 */
volatile int32_t lift_test_position;
volatile uint8_t lift_test_feedback_valid;
volatile uint8_t lift_test_ready;
volatile uint8_t lift_test_last_command_ok;
volatile float lift_test_height_mm;

static uint8_t lift_test_step_state;
static uint8_t lift_test_busy;

static void Lift_Test_ReadPosition(void)
{
  int32_t position;

  lift_test_feedback_valid = Lift_GetPosition(&position);
  if (lift_test_feedback_valid != 0)
    lift_test_position = position;
}

void Lift_Test_Init(void)
{
  lift_test_ready = Lift_Init();
  lift_test_feedback_valid = 0;
  lift_test_last_command_ok = 0;
  lift_test_busy = 0;
  lift_test_step_state = 0;
  lift_test_height_mm = 0.0f;

  if (lift_test_ready != 0)
    Lift_Test_ReadPosition();
}

void Lift_Test_Run(void)
{
  Button_Update();

  if (lift_test_busy != 0)
  {
    if (Lift_IsArrived() != 0)
    {
      Lift_Test_ReadPosition();
      lift_test_height_mm = Lift_GetHeight();
      lift_test_busy = 0;
      if (lift_test_step_state == 3)
        lift_test_step_state = 0;
    }
  }

  if (Button_GetStartEvent() == 0)
    return;
  if (lift_test_ready == 0)
    return;

  if (lift_test_busy != 0)
    return;

  if (lift_test_step_state == 0)
  {
    /* 第一次按键以当前反馈位置建立调试零点，不驱动电机。 */
    lift_test_last_command_ok = Lift_SetZero();
    Lift_Test_ReadPosition();
    if (lift_test_last_command_ok != 0)
    {
      lift_test_height_mm = 0.0f;
      lift_test_step_state = 1;
    }
    return;
  }

  /* 第二次上升到目标高度；第三次返回调试零点。 */
  lift_test_last_command_ok = Lift_MoveHeight(
      (lift_test_step_state == 1) ? LIFT_TEST_HEIGHT_MM : 0.0f);
  Lift_Test_ReadPosition();
  if (lift_test_last_command_ok == 0)
    return;

  lift_test_step_state++;
  lift_test_busy = 1;
}
