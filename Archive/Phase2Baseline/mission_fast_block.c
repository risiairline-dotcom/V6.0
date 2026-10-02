#include "mission_fast_block.h"
#include "mission_fast_route.h"

uint8_t Mission_FastBlock_GetPath(uint16_t index, Path_t *path)
{
  return Mission_FastRoute_GetPath(index, path);
}
