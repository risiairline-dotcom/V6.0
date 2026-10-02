#include "gripper.h"
#include "servo.h"
#include "tim.h"

/* SERVO_3 为 PE13 / TIM1_CH3 上的夹爪舵机。 */
/* 张开角暂沿用测试值，待正式接线实测微调。 */
#define GRIPPER_OPEN_ANGLE   120
/* 175° 为当前实测夹紧角，更大角度可能触发机械限位并造成堵转。 */
#define GRIPPER_CLOSE_ANGLE  175

void Gripper_Init(void)
{
  MX_TIM1_Init();

  /* 先设置张开角度，再只启动夹爪所在的 CH3 PWM。 */
  Gripper_Open();
  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
}

void Gripper_Open(void)
{
  Servo_SetAngle(SERVO_3, GRIPPER_OPEN_ANGLE);
}

void Gripper_Close(void)
{
  Servo_SetAngle(SERVO_3, GRIPPER_CLOSE_ANGLE);
}
