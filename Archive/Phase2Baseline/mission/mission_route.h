#ifndef MISSION_ROUTE_H
#define MISSION_ROUTE_H

#include <stdint.h>
#include "mission.h"

/* 正式动作表。MOVE: param1=X mm、param2=Y mm；ROTATE: param1=deg；
 * WAIT: timeout_ms=等待时长。当前路线不使用预留动作类型。 */
extern const MissionAction_t formal_route[];
extern const uint16_t formal_route_count;

#endif /* MISSION_ROUTE_H */
