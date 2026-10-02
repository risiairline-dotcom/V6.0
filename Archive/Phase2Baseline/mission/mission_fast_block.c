#include "mission_fast_block.h"
#include "mission_config.h"
#include "mission_fast_route.h"

/* Block 0：仅供 ACTION_FAST_MOVE 接口验证，不属于正式比赛路线。
 * 固定路径方向始终 X+，第二段增加 +90° 航向目标。 */
static PathPoint_t fast_block_test_points[3] =
{
  { .vx = 500.0f, .vy = 0.0f, .delta_yaw = 0.0f,
    .run_time = 1400U, .accel_time = 600U, .target_distance_mm = 0.0f,
    .mode = PATH_MODE_TIME, .decel_time = 0U },
  { .vx = 500.0f, .vy = 0.0f, .delta_yaw = 90.0f,
    .run_time = 1500U, .accel_time = 900U, .target_distance_mm = 0.0f,
    .mode = PATH_MODE_TIME, .decel_time = 0U },
  { .vx = 500.0f, .vy = 0.0f, .delta_yaw = 0.0f,
    .run_time = 1500U, .accel_time = 600U, .target_distance_mm = 0.0f,
    .mode = PATH_MODE_TIME, .decel_time = 600U }
};

static const MissionFastBlock_t mission_fast_blocks[1] =
{
  {fast_block_test_points, 3U, 1U}
};
static const uint16_t mission_fast_block_count = 1U;

uint8_t Mission_FastBlock_GetPath(uint16_t index, Path_t *path)
{
  const MissionFastBlock_t *block;

  if (MISSION_MODE == MISSION_MODE_FORMAL)
    return Mission_FastRoute_GetPath(index, path);

  if ((path == 0) || (index >= mission_fast_block_count))
    return 0U;

  block = &mission_fast_blocks[index];
  if ((block->points == 0) || (block->point_count == 0U))
    return 0U;

  path->path = block->points;
  path->size = block->point_count;
  path->fast_heading = block->fast_heading;
  return 1U;
}
