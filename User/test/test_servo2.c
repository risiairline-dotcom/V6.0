#include "test_servo2.h"
#include "servo.h"
#include "button.h"
#include "tim.h"

/* SERVO_2：PE11 / TIM1_CH2，有限角度位置舵机。 */
#define SERVO2_TEST_ANGLE_0    0
#define SERVO2_TEST_ANGLE_90   15
#define SERVO2_TEST_ANGLE_180  180
#define SERVO2_TEST_ANGLE_270  270

static uint8_t servo2_next_angle;

void Servo2_Test_Init(void)
{
  MX_TIM1_Init();
  /* 仅启动 CH2，第一次按键才请求 0°。 */
  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  servo2_next_angle = 0;
}

void Servo2_Test_Run(void)
{
  Button_Update();
  if (Button_GetStartEvent() == 0)
    return;

  if (servo2_next_angle == 0)
  {
    Servo_SetAngle(SERVO_2, SERVO2_TEST_ANGLE_0);
    servo2_next_angle = 1;
  }
  else if (servo2_next_angle == 1)
  {
    Servo_SetAngle(SERVO_2, SERVO2_TEST_ANGLE_90);
    servo2_next_angle = 2;
  }
  else if (servo2_next_angle == 2)
  {
    Servo_SetAngle(SERVO_2, SERVO2_TEST_ANGLE_180);
    servo2_next_angle = 3;
  }
  else
  {
    Servo_SetAngle(SERVO_2, SERVO2_TEST_ANGLE_270);
    servo2_next_angle = 0;
  }
}
