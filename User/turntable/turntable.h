#ifndef TURNTABLE_H
#define TURNTABLE_H

#include <stdint.h>

typedef enum
{
  TURNTABLE_READY = 0,
  TURNTABLE_MOVING
} TurntableState_t;

/* SERVO_1 转盘，工位角为 0°、120°、240°。 */
void Turntable_Init(void);
void Turntable_Run(void);
void Turntable_MoveTo(float angle);
void Turntable_Goto_Station1(void);
void Turntable_Goto_Station2(void);
void Turntable_Goto_Station3(void);
uint8_t Turntable_IsDone(void);

#endif /* TURNTABLE_H */
