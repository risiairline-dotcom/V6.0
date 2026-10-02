#ifndef MISSION_H
#define MISSION_H

#include <stdint.h>

typedef enum
{
  ACTION_MOVE_X = 0,
  ACTION_MOVE_Y,
  ACTION_ROTATE,
  ACTION_WAIT
} ActionType;

typedef struct
{
  ActionType type;
  float value;
  uint16_t speed_rpm;
  uint8_t accel;
} MissionAction;

typedef enum
{
  MISSION_IDLE = 0,
  MISSION_ACTION_START,
  MISSION_ACTION_WAIT,
  MISSION_FINISH,
  MISSION_ERROR
} MissionState_t;

extern volatile MissionState_t mission_state;
extern volatile uint8_t mission_started;
extern volatile uint8_t mission_finished;

void Mission_Init(void);
void Mission_Start(void);
void Mission_Stop(void);
void Mission_Run(void);
uint8_t Mission_IsRunning(void);
uint8_t Mission_IsFinished(void);

#endif /* MISSION_H */

