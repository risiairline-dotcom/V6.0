#include "test_extend.h"
#include "extend.h"
#include "button.h"
#include "stm32f4xx_hal.h"

/* 电机轴相对角度；5000° 约为 13.9 圈，实车先确认齿条可用行程。 */
#define EXTEND_TEST_STEP            5000
#define EXTEND_TEST_POLL_MS         100
#define EXTEND_POSITION_UNITS_REV   65536
#define EXTEND_ARRIVAL_TOLERANCE    200

/* 供 Keil Watch 查看每次新位置反馈与测试状态。 */
volatile int32_t extend_test_position;
volatile uint8_t extend_test_feedback_valid;
volatile uint8_t extend_test_ready;

static int32_t extend_test_start_position;
static uint32_t extend_test_target_travel;
static uint32_t extend_test_last_poll_tick;
static uint8_t extend_test_next_positive;
static uint8_t extend_test_busy;

void Extend_Test_Init(void)
{
  int32_t position;

  Extend_Init();
  position = Extend_GetPosition();
  extend_test_feedback_valid = (position != INT32_MIN) ? 1 : 0;
  extend_test_ready = (Extend_IsReady() != 0) && (extend_test_feedback_valid != 0);
  if (extend_test_feedback_valid != 0)
    extend_test_position = position;
  extend_test_next_positive = 1;
  extend_test_busy = 0;
}

void Extend_Test_Run(void)
{
  int32_t position;
  int32_t angle;
  int64_t travel;

  Button_Update();

  if ((extend_test_busy != 0) &&
      ((uint32_t)(HAL_GetTick() - extend_test_last_poll_tick) >= EXTEND_TEST_POLL_MS))
  {
    extend_test_last_poll_tick = HAL_GetTick();
    position = Extend_GetPosition();
    extend_test_feedback_valid = (position != INT32_MIN) ? 1 : 0;
    if (extend_test_feedback_valid != 0)
    {
      extend_test_position = position;
      /* 反馈正负方向尚待标定，先比较本次位移的绝对值。 */
      travel = (int64_t)position - extend_test_start_position;
      if (travel < 0)
        travel = -travel;
      if ((uint64_t)travel + EXTEND_ARRIVAL_TOLERANCE >= extend_test_target_travel)
        extend_test_busy = 0;
    }
  }

  if (Button_GetStartEvent() == 0)
    return;
  if ((extend_test_ready == 0) || (extend_test_busy != 0))
    return;

  position = Extend_GetPosition();
  extend_test_feedback_valid = (position != INT32_MIN) ? 1 : 0;
  if (extend_test_feedback_valid == 0)
    return;
  extend_test_position = position;

  angle = (extend_test_next_positive != 0) ? EXTEND_TEST_STEP : -EXTEND_TEST_STEP;
  Extend_MoveRelative(angle);
  if (Extend_CommandAccepted() == 0)
    return;
  extend_test_start_position = position;
  extend_test_target_travel =
      ((uint32_t)EXTEND_TEST_STEP * EXTEND_POSITION_UNITS_REV + 180) / 360;
  extend_test_last_poll_tick = HAL_GetTick();
  extend_test_busy = 1;
  extend_test_next_positive = (extend_test_next_positive == 0) ? 1 : 0;

  /* 发令后读取一次 ID6 新反馈，后续位置由周期查询更新。 */
  position = Extend_GetPosition();
  extend_test_feedback_valid = (position != INT32_MIN) ? 1 : 0;
  if (extend_test_feedback_valid != 0)
    extend_test_position = position;
}
