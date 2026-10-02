#include "mission_fast_route.h"
#include "mission_fast_block.h"

#define FORMAL_FAST_SPEED_MM_S              500.0f

/* 正式 Fast 路线沿用已验证的 TIME 参数。 */
#define FORMAL_FAST_B0_Y_START_TIME_MS      120U
#define FORMAL_FAST_B0_X_FORWARD_TIME_MS    2600U
#define FORMAL_FAST_B0_Y_FORWARD_TIME_MS    1850U
#define FORMAL_FAST_B0_X_RETURN_TIME_MS     1880U
#define FORMAL_FAST_B0_ROTATE_TIME_MS       1800U
#define FORMAL_FAST_B0_SHORT_ACCEL_MS       100U
#define FORMAL_FAST_B0_MOVE_ACCEL_MS        500U
#define FORMAL_FAST_B0_ROTATE_ACCEL_MS      600U

/* Block 0：起点 → 二维码点。X+1050 为末段，按原移动过渡时间平滑减速。 */
static PathPoint_t formal_fast_block0_points[2] =
{
  { .vx = 0.0f, .vy = -FORMAL_FAST_SPEED_MM_S, .delta_yaw = 0.0f,
    .run_time = FORMAL_FAST_B0_Y_START_TIME_MS,
    .accel_time = FORMAL_FAST_B0_SHORT_ACCEL_MS,
    .target_distance_mm = 0.0f, .mode = PATH_MODE_TIME, .decel_time = 0U },
  { .vx = FORMAL_FAST_SPEED_MM_S, .vy = 0.0f, .delta_yaw = 0.0f,
    .run_time = FORMAL_FAST_B0_X_FORWARD_TIME_MS,
    .accel_time = FORMAL_FAST_B0_MOVE_ACCEL_MS,
    .target_distance_mm = 0.0f, .mode = PATH_MODE_TIME,
    .decel_time = FORMAL_FAST_B0_MOVE_ACCEL_MS }
};

/* Block 1：扫码点 → Y-990 → X-922 → 航向+90°。首段从静止缓启动。 */
static PathPoint_t formal_fast_block1_points[3] =
{
  { .vx = 0.0f, .vy = -FORMAL_FAST_SPEED_MM_S, .delta_yaw = 0.0f,
    .run_time = FORMAL_FAST_B0_Y_FORWARD_TIME_MS,
    .accel_time = FORMAL_FAST_B0_MOVE_ACCEL_MS,
    .target_distance_mm = 0.0f, .mode = PATH_MODE_TIME, .decel_time = 0U },
  { .vx = -FORMAL_FAST_SPEED_MM_S, .vy = 0.0f, .delta_yaw = 0.0f,
    .run_time = FORMAL_FAST_B0_X_RETURN_TIME_MS,
    .accel_time = FORMAL_FAST_B0_MOVE_ACCEL_MS,
    .target_distance_mm = 0.0f, .mode = PATH_MODE_TIME, .decel_time = 0U },
  { .vx = 0.0f, .vy = 0.0f, .delta_yaw = 90.0f,
    .run_time = FORMAL_FAST_B0_ROTATE_TIME_MS,
    .accel_time = FORMAL_FAST_B0_ROTATE_ACCEL_MS,
    .target_distance_mm = 0.0f, .mode = PATH_MODE_TIME,
    .decel_time = FORMAL_FAST_B0_ROTATE_ACCEL_MS }
};

static const MissionFastBlock_t formal_fast_blocks[2] =
{
  {formal_fast_block0_points, 2U, 1U},
  {formal_fast_block1_points, 3U, 1U}
};

uint8_t Mission_FastRoute_GetPath(uint16_t index, Path_t *path)
{
  const MissionFastBlock_t *block;

  if ((path == 0) || (index >= 2U))
    return 0U;

  block = &formal_fast_blocks[index];
  path->path = block->points;
  path->size = block->point_count;
  path->fast_heading = block->fast_heading;
  return 1U;
}
