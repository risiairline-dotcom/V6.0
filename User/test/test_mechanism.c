#include "test_mechanism.h"
#include "turntable.h"
#include "arm_rotate.h"
#include "gripper.h"
#include "lift.h"
#include "servo.h"
#include "button.h"

/* 执行机构综合测试参数，按键逐步执行，不自动连续运行。 */
#define MECHANISM_TEST_ARM_ANGLE   90
#define MECHANISM_TEST_LIFT_HEIGHT 20

/* 供 Keil Watch 查看当前步骤和 ID5 初始化、到位状态。 */
volatile uint8_t mechanism_test_step;
volatile uint8_t mechanism_test_lift_ready;
volatile uint8_t mechanism_test_lift_arrived;

void Mechanism_Test_Init(void)
{
  /* Gripper_Init 初始化 TIM1 并启动 CH3；随后启动 CH1、CH2、CH3。 */
  Gripper_Init();
  Turntable_Init();
  ArmRotate_Init();
  Servo_Init();

  /* 上电前须人工放到最低安全位置；此处只记录当前位置为调试零点。 */
  mechanism_test_lift_ready = Lift_Init();
  if (mechanism_test_lift_ready != 0)
    mechanism_test_lift_ready = Lift_SetZero();

  mechanism_test_lift_arrived = 0;
  mechanism_test_step = 0;
}

void Mechanism_Test_Run(void)
{
  Turntable_Run();
  if (mechanism_test_step == 4)
    mechanism_test_lift_arrived = Lift_IsArrived();

  Button_Update();
  if (Button_GetStartEvent() == 0)
    return;

  switch (mechanism_test_step)
  {
    case 0:
      if (Turntable_IsDone() == 0)
        return;
      Turntable_Goto_Station2();
      mechanism_test_step = 1;
      break;

    case 1:
      if (Turntable_IsDone() == 0)
        return;
      ArmRotate_MoveTo(MECHANISM_TEST_ARM_ANGLE);
      mechanism_test_step = 2;
      break;

    case 2:
      Gripper_Close();
      mechanism_test_step = 3;
      break;

    case 3:
      if ((mechanism_test_lift_ready == 0) ||
          (Lift_MoveHeight(MECHANISM_TEST_LIFT_HEIGHT) == 0))
        return;
      mechanism_test_step = 4;
      break;

    case 4:
      /* 电机停止，三个舵机继续保持 PWM 和当前位置。 */
      Lift_Stop();
      mechanism_test_step = 5;
      break;

    default:
      break;
  }
}
