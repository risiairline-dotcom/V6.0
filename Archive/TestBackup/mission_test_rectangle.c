#include "mission_test_rectangle.h"
#include "chassis.h"
#include "chassis_config.h"
#include "hwt101.h"
#include "odometry.h"
#include "stm32f4xx_hal.h"

/* 单次 Fast/TIME Path，段间由已有正弦过渡连续连接，不调用分段停止。
 * 速度取自已验证的 Mission 配置，过渡时间沿用 500 ms。
 * TIME 段时长计入过渡分配：前 3 段 2250 ms，末段加上
 * 500 ms 末端减速后为 2750 ms，使标称路径积分回到起点。
 * 四个方向的实际位移及闭合误差由 Odometry 在结束后测量。 */
static PathPoint_t rectangle_points[4] =
{
  { .vx = CHASSIS_MISSION_MOVE_X_SPEED_MM_S, .vy = 0.0f,
    .delta_yaw = 0.0f, .run_time = 2250U, .accel_time = 500U,
    .target_distance_mm = 0.0f, .mode = PATH_MODE_TIME, .decel_time = 0U },
  { .vx = 0.0f, .vy = CHASSIS_MISSION_MOVE_Y_SPEED_MM_S,
    .delta_yaw = 0.0f, .run_time = 2250U, .accel_time = 500U,
    .target_distance_mm = 0.0f, .mode = PATH_MODE_TIME, .decel_time = 0U },
  { .vx = -CHASSIS_MISSION_MOVE_X_SPEED_MM_S, .vy = 0.0f,
    .delta_yaw = 0.0f, .run_time = 2250U, .accel_time = 500U,
    .target_distance_mm = 0.0f, .mode = PATH_MODE_TIME, .decel_time = 0U },
  { .vx = 0.0f, .vy = -CHASSIS_MISSION_MOVE_Y_SPEED_MM_S,
    .delta_yaw = 0.0f, .run_time = 2750U, .accel_time = 500U,
    .target_distance_mm = 0.0f, .mode = PATH_MODE_TIME, .decel_time = 500U }
};
static Path_t rectangle_path = { rectangle_points, 4U, 1U };

typedef enum
{
  RECTANGLE_IDLE = 0,
  RECTANGLE_WAIT_IMU,
  RECTANGLE_RESET,
  RECTANGLE_RUN,
  RECTANGLE_SETTLE,
  RECTANGLE_FINISH
} RectangleState_t;

volatile uint8_t rectangle_test_done = 0U;
volatile uint8_t rectangle_test_error = 0U;
volatile float rectangle_final_x = 0.0f;
volatile float rectangle_final_y = 0.0f;
volatile float rectangle_final_yaw = 0.0f;

static RectangleState_t rectangle_state;
static uint32_t rectangle_stop_tick;

void Mission_TestRectangle_Init(void)
{
  rectangle_state = RECTANGLE_IDLE;
  rectangle_test_done = 0U;
  rectangle_test_error = 0U;
}

void Mission_TestRectangle_Start(void)
{
  if ((rectangle_state != RECTANGLE_IDLE) &&
      (rectangle_state != RECTANGLE_FINISH))
    return;

  rectangle_test_done = 0U;
  rectangle_test_error = 0U;
  rectangle_final_x = 0.0f;
  rectangle_final_y = 0.0f;
  rectangle_final_yaw = 0.0f;
  rectangle_state = (imu_zero_done != 0) ? RECTANGLE_RESET : RECTANGLE_WAIT_IMU;
}

void Mission_TestRectangle_Stop(void)
{
  Chassis_Stop();
  rectangle_state = RECTANGLE_IDLE;
}

void Mission_TestRectangle_Run(void)
{
  switch (rectangle_state)
  {
    case RECTANGLE_WAIT_IMU:
      if (imu_zero_done != 0)
        rectangle_state = RECTANGLE_RESET;
      break;

    case RECTANGLE_RESET:
      if (Odometry_Reset() == 0U)
      {
        rectangle_test_error = 1U;
        rectangle_state = RECTANGLE_FINISH;
        break;
      }
      if (Chassis_MovePath(&rectangle_path) != CHASSIS_OK)
      {
        rectangle_test_error = 2U;
        rectangle_state = RECTANGLE_FINISH;
        break;
      }
      rectangle_state = RECTANGLE_RUN;
      break;

    case RECTANGLE_RUN:
      if (Chassis_IsPathDone() != 0U)
      {
        Chassis_Stop();
        rectangle_stop_tick = HAL_GetTick();
        rectangle_state = RECTANGLE_SETTLE;
      }
      break;

    case RECTANGLE_SETTLE:
      if ((HAL_GetTick() - rectangle_stop_tick) >= 150U)
      {
        rectangle_final_x = odom_x_mm;
        rectangle_final_y = odom_y_mm;
        rectangle_final_yaw = odom_yaw;
        rectangle_test_done = 1U;
        rectangle_state = RECTANGLE_FINISH;
      }
      break;

    case RECTANGLE_IDLE:
    case RECTANGLE_FINISH:
    default:
      break;
  }
}

uint8_t Mission_TestRectangle_IsFinished(void)
{
  return (rectangle_state == RECTANGLE_FINISH) ? 1U : 0U;
}
