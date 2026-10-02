#include "mission_test_path.h"
#include "chassis.h"
#include "hwt101.h"

/* Fast 对称测试：固定路径坐标中由 X+ 过渡到 Y-，同时累计 -90° 航向。 */
static PathPoint_t test_points[2] =
{
  { .vx = 500.0f, .vy = 0.0f, .delta_yaw = 0.0f,
    .run_time = 1400U, .accel_time = 600U, .target_distance_mm = 0.0f,
    .mode = PATH_MODE_TIME, .decel_time = 0U },
  { .vx = 0.0f, .vy = -500.0f, .delta_yaw = -90.0f,
    .run_time = 1500U, .accel_time = 900U, .target_distance_mm = 0.0f,
    .mode = PATH_MODE_TIME, .decel_time = 600U }
};

static Path_t test_path = { test_points, 2U, 1U };

typedef enum
{
  TEST_PATH_IDLE = 0,
  TEST_PATH_WAIT_IMU,
  TEST_PATH_START,
  TEST_PATH_RUN,
  TEST_PATH_FINISH
} TestPathState_t;

static TestPathState_t test_path_state;

void Mission_TestPath_Init(void)
{
  test_path_state = TEST_PATH_IDLE;
}

void Mission_TestPath_Start(void)
{
  if ((test_path_state != TEST_PATH_IDLE) &&
      (test_path_state != TEST_PATH_FINISH))
    return;
  test_path_state = (imu_zero_done != 0) ? TEST_PATH_START : TEST_PATH_WAIT_IMU;
}

void Mission_TestPath_Stop(void)
{
  Chassis_Stop();
  test_path_state = TEST_PATH_IDLE;
}

void Mission_TestPath_Run(void)
{
  switch (test_path_state)
  {
    case TEST_PATH_WAIT_IMU:
      if (imu_zero_done != 0)
        test_path_state = TEST_PATH_START;
      break;

    case TEST_PATH_START:
      if (Chassis_MovePath(&test_path) == CHASSIS_OK)
        test_path_state = TEST_PATH_RUN;
      else
        test_path_state = TEST_PATH_FINISH;
      break;

    case TEST_PATH_RUN:
      if (Chassis_IsPathDone() != 0)
        test_path_state = TEST_PATH_FINISH;
      break;

    case TEST_PATH_IDLE:
    case TEST_PATH_FINISH:
    default:
      break;
  }
}

uint8_t Mission_TestPath_IsFinished(void)
{
  return (test_path_state == TEST_PATH_FINISH) ? 1U : 0U;
}
