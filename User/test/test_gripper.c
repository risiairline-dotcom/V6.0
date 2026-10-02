#include "test_gripper.h"
#include "servo.h"
#include "button.h"
#include "tim.h"

/* 当前仅借用 SERVO_1 逻辑通道测试 PE9 上的夹爪，实际比赛功能不改变。 */
/* 按键测试角度集中在此修改。 */
#define GRIPPER_INITIAL_ANGLE  0
#define GRIPPER_TEST_ANGLE_A   50
#define GRIPPER_TEST_ANGLE_B   170

static uint8_t next_angle_is_b;

void Gripper_Test_Init(void)
{
  MX_TIM1_Init();

  /* 先设置起始角度，再仅启动 PE9 对应的 TIM1_CH1 PWM。 */
  Servo_SetAngle(SERVO_1, GRIPPER_INITIAL_ANGLE);
  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  next_angle_is_b = 1;
}

void Gripper_Test_Run(void)
{
  Button_Update();
  if (Button_GetStartEvent() == 0)
    return;

  if (next_angle_is_b != 0)
    Servo_SetAngle(SERVO_1, GRIPPER_TEST_ANGLE_B);
  else
    Servo_SetAngle(SERVO_1, GRIPPER_TEST_ANGLE_A);

  next_angle_is_b = (next_angle_is_b == 0) ? 1 : 0;
}
