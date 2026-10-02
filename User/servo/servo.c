#include "servo.h"
#include "servo_config.h"
#include "tim.h"

volatile HAL_StatusTypeDef servo_pwm_start_status = HAL_ERROR;

#define SERVO_2_MIN_TEST_PULSE_US  1000U
#define SERVO_2_MAX_TEST_PULSE_US  2000U
#define SERVO_2_MAX_ANGLE         359

void Servo_Init(void)
{
  servo_pwm_start_status = HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  if (servo_pwm_start_status != HAL_OK)
  {
    Error_Handler();
  }

  servo_pwm_start_status = HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  if (servo_pwm_start_status != HAL_OK)
  {
    Error_Handler();
  }

  servo_pwm_start_status = HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  if (servo_pwm_start_status != HAL_OK)
  {
    Error_Handler();
  }
}

void Servo_SetPulse(uint8_t id, uint16_t pulse_us)
{
  uint32_t channel;

  if (id == SERVO_1)
  {
    channel = TIM_CHANNEL_1;
  }
  else if (id == SERVO_2)
  {
    channel = TIM_CHANNEL_2;

    if (pulse_us < SERVO_2_MIN_TEST_PULSE_US)
    {
      pulse_us = SERVO_2_MIN_TEST_PULSE_US;
    }
    else if (pulse_us > SERVO_2_MAX_TEST_PULSE_US)
    {
      pulse_us = SERVO_2_MAX_TEST_PULSE_US;
    }
  }
  else if (id == SERVO_3)
  {
    channel = TIM_CHANNEL_3;
  }
  else
  {
    return;
  }

  __HAL_TIM_SET_COMPARE(&htim1, channel, pulse_us);
}

void Servo_SetAngle(uint8_t id, float angle)
{
  uint32_t channel;
  float min_pulse_us;
  float max_pulse_us;
  float max_angle;
  float pulse_us;

  if (id == SERVO_1)
  {
    channel = TIM_CHANNEL_1;
    min_pulse_us = SERVO_1_MIN_PULSE_US;
    max_pulse_us = SERVO_1_MAX_PULSE_US;
    max_angle = SERVO_1_MAX_ANGLE;
  }
  else if (id == SERVO_2)
  {
    /* PE11 / TIM1_CH2：359°有限角度位置舵机。 */
    channel = TIM_CHANNEL_2;
    min_pulse_us = SERVO_2_MIN_TEST_PULSE_US;
    max_pulse_us = SERVO_2_MAX_TEST_PULSE_US;
    max_angle = SERVO_2_MAX_ANGLE;
  }
  else if (id == SERVO_3)
  {
    channel = TIM_CHANNEL_3;
    min_pulse_us = SERVO_3_MIN_PULSE_US;
    max_pulse_us = SERVO_3_MAX_PULSE_US;
    max_angle = SERVO_3_MAX_ANGLE;
  }
  else
  {
    return;
  }

  if (angle < 0)
    angle = 0;
  else if (angle > max_angle)
    angle = max_angle;

  pulse_us = min_pulse_us + angle * (max_pulse_us - min_pulse_us) / max_angle;
  __HAL_TIM_SET_COMPARE(&htim1, channel, (uint32_t)(pulse_us + 0.5f));
}
