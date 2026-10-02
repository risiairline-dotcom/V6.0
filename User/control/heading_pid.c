#include "heading_pid.h"
#include "chassis.h"
#include "hwt101.h"
#include "stm32f4xx_hal.h"

/* PID 参数沿用参考工程 PID_move 的 pid_choose == 1 分支。 */
#define HEADING_KP                 3.5f
#define HEADING_KI                 0.0f
#define HEADING_KD                 2.0f
#define HEADING_MAX_INTEGRAL       7.0f
/* 参考工程输出量级 30 与当前 Chassis_Control 的 deg/s 单位不同，初次适配上限为 100 deg/s。 */
#define HEADING_MAX_OUTPUT         120.0f

/* 保持参考工程的 50 ms 控制周期和固定时间结束方式。 */
#define HEADING_CONTROL_PERIOD_MS  50U
/* 时间估算使用独立标定速度，不使用 PID 峰值输出。 */
#define HEADING_TIME_SPEED         80.0f
/* 固定时间裕量；运行中不根据误差提前停止。 */
#define HEADING_TIME_MARGIN_MS     800U

typedef struct
{
  float kp;
  float ki;
  float kd;
  float error;
  float last_error;
  float integral;
  float max_integral;
  float output;
  float max_output;
} PID;

static PID heading_pid;

static void PID_Init(PID *pid,
                     float kp,
                     float ki,
                     float kd,
                     float max_integral,
                     float max_output)
{
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->error = 0.0f;
  pid->last_error = 0.0f;
  pid->integral = 0.0f;
  pid->max_integral = max_integral;
  pid->output = 0.0f;
  pid->max_output = max_output;
}

static float PID_WrapError(float error)
{
  if (error < -180.0f)
  {
    error += 360.0f;
  }
  else if (error > 180.0f)
  {
    error -= 360.0f;
  }
  return error;
}

static void PID_Calc(PID *pid, float target, float feedback)
{
  float proportional;
  float derivative;

  pid->last_error = pid->error;
  pid->error = PID_WrapError(target - feedback);

  derivative = (pid->error - pid->last_error) * pid->kd;
  proportional = pid->error * pid->kp;
  pid->integral += pid->error * pid->ki;

  if (pid->integral > pid->max_integral)
  {
    pid->integral = pid->max_integral;
  }
  else if (pid->integral < -pid->max_integral)
  {
    pid->integral = -pid->max_integral;
  }

  pid->output = proportional + derivative + pid->integral;

  if (pid->output > pid->max_output)
  {
    pid->output = pid->max_output;
  }
  else if (pid->output < -pid->max_output)
  {
    pid->output = -pid->max_output;
  }
}

void Heading_Init(void)
{
  PID_Init(&heading_pid,
           HEADING_KP,
           HEADING_KI,
           HEADING_KD,
           HEADING_MAX_INTEGRAL,
           HEADING_MAX_OUTPUT);
}

void Heading_RotateTo(float target_deg)
{
  uint32_t start_tick;
  uint32_t runtime_ms;
  float turn_deg;

  Heading_Init();
  /* 每次开始转向前完全清除上一次 PID 的动态状态。 */
  heading_pid.error = 0.0f;
  heading_pid.last_error = 0.0f;
  heading_pid.integral = 0.0f;
  heading_pid.output = 0.0f;
  turn_deg = PID_WrapError(target_deg - IMU_GetYaw());
  if (turn_deg < 0.0f)
  {
    turn_deg = -turn_deg;
  }
  runtime_ms = (uint32_t)(turn_deg * 1000.0f / HEADING_TIME_SPEED) +
               HEADING_TIME_MARGIN_MS;
  start_tick = HAL_GetTick();

  while ((uint32_t)(HAL_GetTick() - start_tick) < runtime_ms)
  {
    PID_Calc(&heading_pid, target_deg, IMU_GetYaw());
    Chassis_Control(0.0f, 0.0f, heading_pid.output);
    HAL_Delay(HEADING_CONTROL_PERIOD_MS);
  }

  Chassis_Control(0.0f, 0.0f, 0.0f);
}

/* 保留现有接口符号；输入值仍直接作为绝对目标角。 */
void Heading_RotateRelative(float delta_deg)
{
  Heading_RotateTo(delta_deg);
}



