#include "turntable.h"
#include "turntable_config.h"
#include "servo.h"
#include "stm32f4xx_hal.h"

static TurntableState_t turntable_state;
static uint32_t move_start_tick;

void Turntable_Init(void)
{
  Servo_SetAngle(SERVO_1, TURNTABLE_STATION1_ANGLE);
  move_start_tick = HAL_GetTick();
  turntable_state = TURNTABLE_MOVING;
}

void Turntable_Run(void)
{
  if ((turntable_state == TURNTABLE_MOVING) &&
      ((HAL_GetTick() - move_start_tick) >= TURNTABLE_MOVE_TIME_MS))
  {
    turntable_state = TURNTABLE_READY;
  }
}

void Turntable_MoveTo(float angle)
{
  if (!(angle >= 0.0f && angle <= 270.0f) ||
      (turntable_state == TURNTABLE_MOVING))
    return;

  Servo_SetAngle(SERVO_1, angle);
  move_start_tick = HAL_GetTick();
  turntable_state = TURNTABLE_MOVING;
}

void Turntable_Goto_Station1(void)
{
  Turntable_MoveTo(TURNTABLE_STATION1_ANGLE);
}

void Turntable_Goto_Station2(void)
{
  Turntable_MoveTo(TURNTABLE_STATION2_ANGLE);
}

void Turntable_Goto_Station3(void)
{
  Turntable_MoveTo(TURNTABLE_STATION3_ANGLE);
}

uint8_t Turntable_IsDone(void)
{
  return (turntable_state == TURNTABLE_READY) ? 1 : 0;
}
