#ifndef MISSION_ROUTE_H
#define MISSION_ROUTE_H

#include "mission.h"
#include <stdint.h>

uint8_t MissionRoute_IsReady(void);
uint8_t MissionRoute_GetCount(void);
const MissionAction *MissionRoute_GetAction(uint8_t index);

#endif /* MISSION_ROUTE_H */

