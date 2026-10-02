#ifndef MISSION_FAST_ROUTE_H
#define MISSION_FAST_ROUTE_H

#include <stdint.h>
#include "path.h"

/* 正式比赛 Fast Block，索引从 0 开始；不包含测试 Block。 */
uint8_t Mission_FastRoute_GetPath(uint16_t index, Path_t *path);

#endif /* MISSION_FAST_ROUTE_H */
