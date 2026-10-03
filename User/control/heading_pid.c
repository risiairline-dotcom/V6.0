#include "heading_pid.h"
#include "chassis.h"
#include "chassis_config.h"
#include "hwt101.h"
#include "stm32f4xx_hal.h"
#include <math.h>

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

/*
 * 功能：按 50 ms 周期执行航向 PID 旋转。
 * 参数：target_deg，绝对目标航向角，单位为度。
 * 返回：无；到达目标且角速度稳定后返回。
 */
void Heading_RotateTo(float target_deg)
{
  uint8_t stable_count = 0U;

  Heading_Init();
  /* 每次开始转向前完全清除上一次 PID 的动态状态。 */
  heading_pid.error = 0.0f;
  heading_pid.last_error = 0.0f;
  heading_pid.integral = 0.0f;
  heading_pid.output = 0.0f;

  while (1)
  {
    PID_Calc(&heading_pid, target_deg, IMU_GetYaw());
    Chassis_Control(0.0f, 0.0f, heading_pid.output);

    /* 角度和角速度同时稳定约 250 ms 后结束旋转。 */
    if ((fabsf(heading_pid.error) < HEADING_STABLE_ERROR_DEG) &&
        (fabsf(IMU_GetGyroZ()) < HEADING_STABLE_GYRO_DPS))
    {
      stable_count++;
      if (stable_count >= HEADING_STABLE_COUNT)
      {
        break;
      }
    }
    else
    {
      stable_count = 0U;
    }

    HAL_Delay(HEADING_CONTROL_PERIOD_MS);
  }

  Chassis_Control(0.0f, 0.0f, 0.0f);
}

/* 保留现有接口符号；输入值仍直接作为绝对目标角。 */
void Heading_RotateRelative(float delta_deg)
{
  Heading_RotateTo(delta_deg);
}



