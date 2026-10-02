#include "test_servo1.h"
#include "servo.h"
#include "button.h"
#include "tim.h"

/* SERVO_1：PE9 / TIM1_CH1，按键依次测试三个角度。 */
#define SERVO1_TEST_ANGLE_1  0
#define SERVO1_TEST_ANGLE_2  120
#define SERVO1_TEST_ANGLE_3  240

static uint8_t servo1_next_angle;

void Servo1_Test_Init(void)
{
  MX_TIM1_Init();
  Servo_SetAngle(SERVO_1, SERVO1_TEST_ANGLE_1);
  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  servo1_next_angle = 2;
}

void Servo1_Test_Run(void)
{
  Button_Update();
  if (Button_GetStartEvent() == 0)
    return;

  if (servo1_next_angle == 2)
  {
    Servo_SetAngle(SERVO_1, SERVO1_TEST_ANGLE_2);
    servo1_next_angle = 3;
  }
  else if (servo1_next_angle == 3)
  {
    Servo_SetAngle(SERVO_1, SERVO1_TEST_ANGLE_3);
    servo1_next_angle = 1;
  }
  else
  {
    Servo_SetAngle(SERVO_1, SERVO1_TEST_ANGLE_1);
    servo1_next_angle = 2;
  }
}
