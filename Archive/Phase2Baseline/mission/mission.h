#ifndef MISSION_H
#define MISSION_H

#include <stdint.h>

typedef enum
{
  ACTION_MOVE = 0,
  ACTION_FAST_MOVE,
  ACTION_ROTATE,
  ACTION_WAIT,
  ACTION_GRAB,
  ACTION_PLACE,
  ACTION_FINISH,
  ACTION_SCAN
} ActionType_t;

typedef struct
{
  ActionType_t type;
  float param1;
  float param2;
  float param3;
  float speed;
  uint32_t timeout_ms;
} MissionAction_t;

typedef enum
{
  MISSION_IDLE = 0,
  MISSION_ACTION_START,
  MISSION_ACTION_WAIT,
  MISSION_ACTION_FINISH,
  MISSION_FORMAL_ROUTE_START,
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
