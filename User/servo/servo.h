#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

typedef enum
{
  SERVO_1 = 0,
  SERVO_2,
  SERVO_3
} ServoId_t;

/* [DEBUG/DORMANT] 阶段1未初始化TIM1和舵机，保留状态供后续调试。 */
extern volatile HAL_StatusTypeDef servo_pwm_start_status;

void Servo_Init(void);
void Servo_SetPulse(uint8_t id, uint16_t pulse_us);
/* SERVO_2 为 0~359° 有限角度位置舵机，对应 TIM1_CH2 / PE11。 */
void Servo_SetAngle(uint8_t id, float angle);

#endif /* SERVO_H */
