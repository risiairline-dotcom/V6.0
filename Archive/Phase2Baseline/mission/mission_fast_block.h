#ifndef MISSION_FAST_BLOCK_H
#define MISSION_FAST_BLOCK_H

#include <stdint.h>
#include "path.h"

/* 一个连续运动块由现有 PathPoint 数组、点数和 Fast 航向开关组成。 */
typedef struct
{
  PathPoint_t *points;
  uint8_t point_count;
  uint8_t fast_heading;
} MissionFastBlock_t;

/* 按零起始索引读取 Block：Formal 模式使用正式表，其余模式使用测试表。 */
uint8_t Mission_FastBlock_GetPath(uint16_t index, Path_t *path);

#endif /* MISSION_FAST_BLOCK_H */
