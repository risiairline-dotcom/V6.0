#include "mission_route.h"
#include "mission_config.h"

#define FORMAL_MOVE_X(distance_mm) \
  {ACTION_MOVE, (distance_mm), 0.0f, 0.0f, MISSION_FORMAL_MOVE_X_SPEED_MM_S, 0U}
#define FORMAL_MOVE_Y(distance_mm) \
  {ACTION_MOVE, 0.0f, (distance_mm), 0.0f, MISSION_FORMAL_MOVE_Y_SPEED_MM_S, 0U}
#define FORMAL_ROTATE(angle_deg) \
  {ACTION_ROTATE, (angle_deg), 0.0f, 0.0f, MISSION_FORMAL_ROTATE_SPEED_DEG_S, 0U}

/*
 * 第一版正式路线：距离单位mm，角度单位degree。
 * 当前执行两段 Fast 移动，并在二维码位置等待有效任务码。
 */
const MissionAction_t formal_route[] =
{
  /* 起点 → 二维码点 → 扫码 → 前半段剩余路线。 */
  {ACTION_FAST_MOVE, 0.0f, 0.0f, 0.0f, 0.0f, 0U},
  {ACTION_SCAN,      0.0f, 0.0f, 0.0f, 0.0f, 0U},
  {ACTION_FAST_MOVE, 1.0f, 0.0f, 0.0f, 0.0f, 0U},
	
  /*FORMAL_MOVE_Y(-100.0f),
  FORMAL_MOVE_X(1050.0f),
  FORMAL_MOVE_Y(-960.0f),
  FORMAL_MOVE_X(-922.0f),
  FORMAL_ROTATE(90.0f),
  FORMAL_ROTATE(-90.0f),
  FORMAL_MOVE_X(1730.0f),
  FORMAL_ROTATE(-90.0f),
  FORMAL_MOVE_X(-852.0f),
  FORMAL_MOVE_Y(860.0f),
  FORMAL_ROTATE(90.0f),
  FORMAL_MOVE_X(-922.0f),
  FORMAL_MOVE_Y(852.0f),
  FORMAL_ROTATE(90.0f),
  FORMAL_ROTATE(-90.0f),
  FORMAL_MOVE_X(1750.0f),
  FORMAL_ROTATE(-90.0f),
  FORMAL_MOVE_X(-852.0f),
  FORMAL_MOVE_Y(860.0f),
  FORMAL_ROTATE(90.0f),
  FORMAL_MOVE_X(-922.0f),
  FORMAL_ROTATE(90.0f),
  FORMAL_MOVE_X(-1960.0f),
  FORMAL_MOVE_Y(-120.0f),*/
};

const uint16_t formal_route_count =
    (uint16_t)(sizeof(formal_route) / sizeof(formal_route[0]));
