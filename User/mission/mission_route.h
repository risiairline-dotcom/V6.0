#ifndef MISSION_ROUTE_H
#define MISSION_ROUTE_H

#include "mission.h"
#include <stdint.h>

uint8_t MissionRoute_IsReady(void);
/* 返回当前正式路线动作数量。 */
uint8_t MissionRoute_GetCount(void);
/* 按索引取得动作；索引越界返回空指针。 */
const MissionAction *MissionRoute_GetAction(uint8_t index);

#endif /* MISSION_ROUTE_H */

