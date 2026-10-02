#include "mission_test_odometry.h"
#include "chassis.h"
#include "hwt101.h"
#include "odometry.h"
#include "stm32f4xx_hal.h"

/* Y+ 测试：保持已验证的 500 mm/s、2500 ms TIME 段及前后各 500 ms
 * 正弦过渡，标称积分位移 1000 mm。里程反馈只用于测试输出。 */
static PathPoint_t odom_test_point =
{
  .vx = 500.0f, .vy = 0.0f, .delta_yaw = 0.0f,
  .run_time = 2500U, .accel_time = 500U, .target_distance_mm = 0.0f,
  .mode = PATH_MODE_TIME, .decel_time = 500U
};
static Path_t odom_test_path = { &odom_test_point, 1U, 1U };

typedef enum
{
  ODOM_TEST_IDLE = 0,
  ODOM_TEST_WAIT_IMU,
  ODOM_TEST_RESET,
  ODOM_TEST_RUN,
  ODOM_TEST_SETTLE,
  ODOM_TEST_FINISH
} OdomTestState_t;

volatile float odom_test_start_x = 0.0f;
volatile float odom_test_end_x = 0.0f;
volatile float odom_test_distance_x = 0.0f;
volatile float odom_test_start_y = 0.0f;
volatile float odom_test_end_y = 0.0f;
volatile float odom_test_distance_y = 0.0f;
volatile float odom_test_start_yaw = 0.0f;
volatile float odom_test_end_yaw = 0.0f;
volatile float odom_test_yaw_change = 0.0f;
volatile uint8_t odom_test_done = 0U;
volatile uint8_t odom_test_error = 0U;

static OdomTestState_t odom_test_state;
static uint32_t odom_test_stop_tick;

void Mission_TestOdometry_Init(void)
{
  odom_test_state = ODOM_TEST_IDLE;
  odom_test_done = 0U;
  odom_test_error = 0U;
}

void Mission_TestOdometry_Start(void)
{
  if ((odom_test_state != ODOM_TEST_IDLE) &&
      (odom_test_state != ODOM_TEST_FINISH))
    return;

  odom_test_done = 0U;
  odom_test_error = 0U;
  odom_test_start_x = odom_test_end_x = odom_test_distance_x = 0.0f;
  odom_test_start_y = odom_test_end_y = odom_test_distance_y = 0.0f;
  odom_test_start_yaw = odom_test_end_yaw = odom_test_yaw_change = 0.0f;
  odom_test_state = (imu_zero_done != 0) ? ODOM_TEST_RESET : ODOM_TEST_WAIT_IMU;
}

void Mission_TestOdometry_Stop(void)
{
  Chassis_Stop();
  odom_test_state = ODOM_TEST_IDLE;
}

void Mission_TestOdometry_Run(void)
{
  switch (odom_test_state)
  {
    case ODOM_TEST_WAIT_IMU:
      if (imu_zero_done != 0)
        odom_test_state = ODOM_TEST_RESET;
      break;

    case ODOM_TEST_RESET:
      if (Odometry_Reset() == 0U)
      {
        odom_test_error = 1U;
        odom_test_state = ODOM_TEST_FINISH;
        break;
      }
      odom_test_start_x = odom_x_mm;
      odom_test_start_y = odom_y_mm;
      odom_test_start_yaw = odom_yaw;
      if (Chassis_MovePath(&odom_test_path) != CHASSIS_OK)
      {
        odom_test_error = 2U;
        odom_test_state = ODOM_TEST_FINISH;
        break;
      }
      odom_test_state = ODOM_TEST_RUN;
      break;

    case ODOM_TEST_RUN:
      if (Chassis_IsPathDone() != 0U)
      {
        Chassis_Stop();
        odom_test_stop_tick = HAL_GetTick();
        odom_test_state = ODOM_TEST_SETTLE;
      }
      break;

    case ODOM_TEST_SETTLE:
      /* 停止后等待四轮反馈完成若干轮查询，再固定最终读数。 */
      if ((HAL_GetTick() - odom_test_stop_tick) >= 150U)
      {
        odom_test_end_x = odom_x_mm;
        odom_test_end_y = odom_y_mm;
        odom_test_end_yaw = odom_yaw;
        odom_test_distance_x = odom_test_end_x - odom_test_start_x;
        odom_test_distance_y = odom_test_end_y - odom_test_start_y;
        odom_test_yaw_change = odom_test_end_yaw - odom_test_start_yaw;
        odom_test_done = 1U;
        odom_test_state = ODOM_TEST_FINISH;
      }
      break;

    case ODOM_TEST_IDLE:
    case ODOM_TEST_FINISH:
    default:
      break;
  }
}

uint8_t Mission_TestOdometry_IsFinished(void)
{
  return (odom_test_state == ODOM_TEST_FINISH) ? 1U : 0U;
}
