#include "chassis_task.h"
#include "chassis.h"
#include "chassis_config.h"
#include "chassis_profile.h"
#include "emm42.h"
#include "hwt101.h"
#include "motor.h"
#include <math.h>

#define CHASSIS_TASK_PI 3.14159265358979323846f

typedef enum
{
  CHASSIS_TASK_IDLE = 0,
  CHASSIS_TASK_RUNNING,
  CHASSIS_TASK_FINISHED
} ChassisTaskState;

static ChassisTaskState task_state;
static float task_vx;
static float task_vy;
static float task_target_distance;
static float task_current_distance;
static float task_target_yaw;
static float task_direction_sign;
static int32_t task_start_m1;
static int32_t task_start_m2;
static int32_t task_start_m3;
static int32_t task_start_m4;
static float task_heading_integral;

static float Task_Abs(float value)
{
  return (value < 0.0f) ? -value : value;
}

static float Task_WrapAngle(float angle)
{
  while (angle > 180.0f) angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}

static void Task_ReadPosition(int32_t *m1, int32_t *m2,
                              int32_t *m3, int32_t *m4)
{
  *m1 = EMM42_Read_Position(MOTOR_M1_ID);
  *m2 = EMM42_Read_Position(MOTOR_M2_ID);
  *m3 = EMM42_Read_Position(MOTOR_M3_ID);
  *m4 = EMM42_Read_Position(MOTOR_M4_ID);
}

static float Task_ReadDistance(void)
{
  float wheel1 = ((float)EMM42_Read_Position(MOTOR_M1_ID) -
                  (float)task_start_m1) / CHASSIS_FEEDBACK_UNITS_PER_REVOLUTION *
                 (CHASSIS_TASK_PI * CHASSIS_WHEEL_DIAMETER) * MOTOR_M1_SIGN;
  float wheel2 = ((float)EMM42_Read_Position(MOTOR_M2_ID) -
                  (float)task_start_m2) / CHASSIS_FEEDBACK_UNITS_PER_REVOLUTION *
                 (CHASSIS_TASK_PI * CHASSIS_WHEEL_DIAMETER) * MOTOR_M2_SIGN;
  float wheel3 = ((float)EMM42_Read_Position(MOTOR_M3_ID) -
                  (float)task_start_m3) / CHASSIS_FEEDBACK_UNITS_PER_REVOLUTION *
                 (CHASSIS_TASK_PI * CHASSIS_WHEEL_DIAMETER) * MOTOR_M3_SIGN;
  float wheel4 = ((float)EMM42_Read_Position(MOTOR_M4_ID) -
                  (float)task_start_m4) / CHASSIS_FEEDBACK_UNITS_PER_REVOLUTION *
                 (CHASSIS_TASK_PI * CHASSIS_WHEEL_DIAMETER) * MOTOR_M4_SIGN;
  float axis_x = (wheel1 + wheel2 + wheel3 + wheel4) * 0.25f *
                 CHASSIS_FORWARD_SIGN;
  float axis_y = (-wheel1 + wheel2 + wheel3 - wheel4) * 0.25f *
                 CHASSIS_LATERAL_SIGN;
  float direction = sqrtf(task_vx * task_vx + task_vy * task_vy);

  if (direction <= 0.0f) return 0.0f;
  return (axis_x * task_vx * task_direction_sign +
          axis_y * task_vy * task_direction_sign) / direction;
}

void Chassis_Task_Init(void)
{
  task_state = CHASSIS_TASK_IDLE;
  task_vx = 0.0f;
  task_vy = 0.0f;
  task_target_distance = 0.0f;
  task_current_distance = 0.0f;
  task_heading_integral = 0.0f;
}

/*
 * 功能：启动一次非阻塞底盘直线移动。
 * 参数：vx、vy 为移动方向速度，distance_mm 为目标距离，target_yaw 为保持航向。
 * 返回：无；这里只保存参数并记录电机起点，不等待运动完成。
 */
void Chassis_Move_Start(float vx, float vy, float distance_mm,
                        float target_yaw)
{
  float speed_magnitude = sqrtf(vx * vx + vy * vy);

  if ((speed_magnitude <= 0.0f) || (Task_Abs(distance_mm) <= 0.0f))
  {
    Chassis_Stop();
    task_state = CHASSIS_TASK_FINISHED;
    return;
  }

  Task_ReadPosition(&task_start_m1, &task_start_m2,
                    &task_start_m3, &task_start_m4);
  task_vx = vx;
  task_vy = vy;
  task_target_distance = Task_Abs(distance_mm);
  task_current_distance = 0.0f;
  task_target_yaw = target_yaw;
  task_direction_sign = (distance_mm < 0.0f) ? -1.0f : 1.0f;
  task_heading_integral = 0.0f;
  Chassis_ProfileInit();
  task_state = CHASSIS_TASK_RUNNING;
}

/*
 * 功能：执行一次底盘周期任务。
 * 参数：无；由主循环周期调用。
 * 返回：无；到达目标距离并完成减速后停止。
 */
void Chassis_Task(void)
{
  float current_distance;
  float remaining_distance;
  float speed_magnitude;
  float current_speed;
  float speed_scale;
  float error;
  float wz;
  const float period_s = (float)MOVE_CONTROL_PERIOD_MS / 1000.0f;

  if (task_state != CHASSIS_TASK_RUNNING) return;

  speed_magnitude = sqrtf(task_vx * task_vx + task_vy * task_vy);
  current_distance = Task_ReadDistance();
  task_current_distance = current_distance;
  remaining_distance = task_target_distance - current_distance;
  current_speed = Chassis_ProfileUpdate(
      (remaining_distance <= MOVE_STOP_DISTANCE_MM) ?
      0.0f : speed_magnitude,
      (remaining_distance <= MOVE_STOP_DISTANCE_MM) ?
      MOVE_STOP_DISTANCE_MM : remaining_distance, period_s);

  current_speed += remaining_distance * CHASSIS_POSITION_KP;
  if (current_speed > speed_magnitude) current_speed = speed_magnitude;
  if (current_speed < 0.0f) current_speed = 0.0f;

  if ((remaining_distance <= MOVE_STOP_DISTANCE_MM) &&
      (current_speed <= MOVE_FINAL_STOP_SPEED_MM_S))
  {
    Chassis_Stop();
    task_state = CHASSIS_TASK_FINISHED;
    return;
  }

  speed_scale = current_speed / speed_magnitude;
  error = Task_WrapAngle(task_target_yaw - IMU_GetYaw());
  if (Task_Abs(error) < MOVE_HEADING_DEADZONE)
  {
    task_heading_integral = 0.0f;
    wz = 0.0f;
  }
  else
  {
    task_heading_integral += error * period_s;
    if (task_heading_integral > 100.0f) task_heading_integral = 100.0f;
    if (task_heading_integral < -100.0f) task_heading_integral = -100.0f;
    wz = CHASSIS_MOVE_HEADING_KP * error +
         CHASSIS_MOVE_HEADING_KI * task_heading_integral -
         CHASSIS_MOVE_HEADING_KD * IMU_GetGyroZ();
  }
  if (wz > MOVE_HEADING_MAX_WZ) wz = MOVE_HEADING_MAX_WZ;
  if (wz < -MOVE_HEADING_MAX_WZ) wz = -MOVE_HEADING_MAX_WZ;

  Chassis_Control(task_vx * task_direction_sign * speed_scale,
                  task_vy * task_direction_sign * speed_scale, wz);
}

uint8_t Chassis_IsFinished(void)
{
  return (task_state == CHASSIS_TASK_FINISHED) ? 1U : 0U;
}

float Chassis_GetRemainDistance(void)
{
  float remain_distance;

  /* 该接口只读取任务状态，不参与速度或电机控制。 */
  if (task_state != CHASSIS_TASK_RUNNING)
  {
    return 0.0f;
  }

  remain_distance = task_target_distance - task_current_distance;
  if (remain_distance < 0.0f)
  {
    remain_distance = 0.0f;
  }
  return remain_distance;
}
