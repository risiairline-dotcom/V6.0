#include "mission_route.h"

/* ==================== 测试路线参数 ==================== */

#define MISSION_TEST_SPEED_RPM  100U
#define MISSION_TEST_ACCEL       80U

static const MissionAction mission_route[] =
{
  /* 启动区1 → 原料区 */
	
    /*矩形测试*/
	/*{ ACTION_MOVE_X, 1000.0f, 100, 80 },
    { ACTION_MOVE_Y, 1000.0f, 100, 80 },
	{ ACTION_MOVE_X, -1000.0f, 100, 80 },
    { ACTION_MOVE_Y, -1000.0f, 100, 80 },*/
    /* 横移调整 */
    { ACTION_MOVE_Y, -100.0f, 100, 80 },

    /* 前往原料区 */
    { ACTION_MOVE_X, 1050.0f, 100, 80 },
    { ACTION_MOVE_Y, -990.0f, 100, 80 },
    { ACTION_MOVE_X, -922.0f, 100, 80 },
    /* 模拟抓取等待 */
    { ACTION_WAIT, 1.0f, 0, 0 },
    
	/* 原料区 → 粗加工区 */
    { ACTION_ROTATE, -90.0f, 0, 0 },
    { ACTION_ROTATE, 90.0f, 0, 0 },
    { ACTION_MOVE_X, 1782.0f, 100, 80 },

    /* 粗加工区 → 暂存区 */
    //{ ACTION_ROTATE, 90.0f, 0, 0 },
    //{ ACTION_MOVE_X, -852.0f, 100, 80 },
    //{ ACTION_MOVE_Y, 860.0f, 100, 80 },

    /* 模拟放置等待 */
    { ACTION_WAIT, 1.0f, 0, 0 },
};

uint8_t MissionRoute_IsReady(void)
{
  return 1U;
}

uint8_t MissionRoute_GetCount(void)
{
  return (uint8_t)(sizeof(mission_route) / sizeof(mission_route[0]));
}

const MissionAction *MissionRoute_GetAction(uint8_t index)
{
  if (index >= MissionRoute_GetCount())
  {
    return 0;
  }
  return &mission_route[index];
}

