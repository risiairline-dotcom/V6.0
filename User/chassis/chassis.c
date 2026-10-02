#include "chassis.h"
#include "chassis_config.h"
#include "mecanum.h"
#include "motor.h"
#include "emm42.h"
#include "hwt101.h"
#include "stm32f4xx_hal.h"
#include <math.h>

#define CHASSIS_PI 3.14159265358979323846f
#define CHASSIS_POSITION_TIME_FACTOR      2.0f
#define CHASSIS_POSITION_SETTLE_MARGIN_MS 1000U

static float Chassis_Abs(float value)
{
  return (value < 0.0f) ? -value : value;
}

static float Chassis_WrapAngle(float angle)
{
  while (angle > 180.0f) angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}

static float Chassis_ReadDistance(float start_m1, float start_m2,
                                  float start_m3, float start_m4,
                                  float vx, float vy)
{
  float wheel1 = ((float)EMM42_Read_Position(MOTOR_M1_ID) - start_m1) /
                 CHASSIS_FEEDBACK_UNITS_PER_REVOLUTION *
                 (CHASSIS_PI * CHASSIS_WHEEL_DIAMETER) * MOTOR_M1_SIGN;
  float wheel2 = ((float)EMM42_Read_Position(MOTOR_M2_ID) - start_m2) /
                 CHASSIS_FEEDBACK_UNITS_PER_REVOLUTION *
                 (CHASSIS_PI * CHASSIS_WHEEL_DIAMETER) * MOTOR_M2_SIGN;
  float wheel3 = ((float)EMM42_Read_Position(MOTOR_M3_ID) - start_m3) /
                 CHASSIS_FEEDBACK_UNITS_PER_REVOLUTION *
                 (CHASSIS_PI * CHASSIS_WHEEL_DIAMETER) * MOTOR_M3_SIGN;
  float wheel4 = ((float)EMM42_Read_Position(MOTOR_M4_ID) - start_m4) /
                 CHASSIS_FEEDBACK_UNITS_PER_REVOLUTION *
                 (CHASSIS_PI * CHASSIS_WHEEL_DIAMETER) * MOTOR_M4_SIGN;
  float axis_x = (wheel1 + wheel2 + wheel3 + wheel4) * 0.25f *
                 CHASSIS_FORWARD_SIGN;
  float axis_y = (-wheel1 + wheel2 + wheel3 - wheel4) * 0.25f *
                 CHASSIS_LATERAL_SIGN;
  float direction = sqrtf(vx * vx + vy * vy);

  if (direction <= 0.0f) return 0.0f;
  return (axis_x * vx + axis_y * vy) / direction;
}

static void Chassis_ReadPosition(int32_t *m1, int32_t *m2,
                                 int32_t *m3, int32_t *m4)
{
  /* 读取四个电机的当前位置，作为运动起点。 */
  if (!EMM42_Read_Position_Confirmed(MOTOR_M1_ID, m1,
                                     MOVE_POSITION_SYNC_TIMEOUT_MS))
    *m1 = EMM42_Read_Position(MOTOR_M1_ID);
  if (!EMM42_Read_Position_Confirmed(MOTOR_M2_ID, m2,
                                     MOVE_POSITION_SYNC_TIMEOUT_MS))
    *m2 = EMM42_Read_Position(MOTOR_M2_ID);
  if (!EMM42_Read_Position_Confirmed(MOTOR_M3_ID, m3,
                                     MOVE_POSITION_SYNC_TIMEOUT_MS))
    *m3 = EMM42_Read_Position(MOTOR_M3_ID);
  if (!EMM42_Read_Position_Confirmed(MOTOR_M4_ID, m4,
                                     MOVE_POSITION_SYNC_TIMEOUT_MS))
    *m4 = EMM42_Read_Position(MOTOR_M4_ID);
}

static float Chassis_MaxWheelMagnitude(const MecanumWheels_t *wheels)
{
  float maximum = Chassis_Abs(wheels->m1);
  if (Chassis_Abs(wheels->m2) > maximum) maximum = Chassis_Abs(wheels->m2);
  if (Chassis_Abs(wheels->m3) > maximum) maximum = Chassis_Abs(wheels->m3);
  if (Chassis_Abs(wheels->m4) > maximum) maximum = Chassis_Abs(wheels->m4);
  return maximum;
}

static int16_t Chassis_WheelSpeedToRpm(float speed_mm_s, int8_t installation_sign)
{
  float rpm = Chassis_Abs(speed_mm_s) * 60.0f /
              (CHASSIS_PI * CHASSIS_WHEEL_DIAMETER);
  int16_t signed_rpm;

  if (rpm > (float)MOTOR_MAX_RPM) rpm = (float)MOTOR_MAX_RPM;
  if ((rpm > 0.0f) && (rpm < (float)CHASSIS_MIN_COMMAND_RPM))
    rpm = (float)CHASSIS_MIN_COMMAND_RPM;
  signed_rpm = (int16_t)(rpm + 0.5f);
  if (speed_mm_s < 0.0f) signed_rpm = (int16_t)-signed_rpm;
  return (int16_t)(signed_rpm * installation_sign);
}

void Chassis_Init(void)
{
  Motor_Init();
  (void)Motor_Enable(MOTOR_M1_ID);
  (void)Motor_Enable(MOTOR_M2_ID);
  (void)Motor_Enable(MOTOR_M3_ID);
  (void)Motor_Enable(MOTOR_M4_ID);
  HAL_Delay(CHASSIS_MOTOR_STABLE_DELAY_MS);
  Chassis_Stop();
}

void Chassis_Stop(void)
{
  (void)Motor_Stop(MOTOR_M1_ID);
  HAL_Delay(CHASSIS_MOTOR_STOP_INTERVAL_MS);
  (void)Motor_Stop(MOTOR_M2_ID);
  HAL_Delay(CHASSIS_MOTOR_STOP_INTERVAL_MS);
  (void)Motor_Stop(MOTOR_M3_ID);
  HAL_Delay(CHASSIS_MOTOR_STOP_INTERVAL_MS);
  (void)Motor_Stop(MOTOR_M4_ID);
}

void Chassis_Control(float vx, float vy, float wz)
{
  MecanumWheels_t wheels;
  float maximum;
  float scale = 1.0f;
  const float max_wheel_speed = (float)MOTOR_MAX_RPM * CHASSIS_PI *
                                 CHASSIS_WHEEL_DIAMETER / 60.0f;

  Mecanum_Inverse(vx * CHASSIS_FORWARD_SIGN,
                  vy * CHASSIS_LATERAL_SIGN,
                   -wz * CHASSIS_PI / 180.0f, &wheels);
  maximum = Chassis_MaxWheelMagnitude(&wheels);
  if (maximum > max_wheel_speed) scale = max_wheel_speed / maximum;

  (void)Motor_Move_Velocity(MOTOR_M1_ID,
      Chassis_WheelSpeedToRpm(wheels.m1 * scale, MOTOR_M1_SIGN), 0U, 1U);
  (void)Motor_Move_Velocity(MOTOR_M2_ID,
      Chassis_WheelSpeedToRpm(wheels.m2 * scale, MOTOR_M2_SIGN), 0U, 1U);
  (void)Motor_Move_Velocity(MOTOR_M3_ID,
      Chassis_WheelSpeedToRpm(wheels.m3 * scale, MOTOR_M3_SIGN), 0U, 1U);
  (void)Motor_Move_Velocity(MOTOR_M4_ID,
      Chassis_WheelSpeedToRpm(wheels.m4 * scale, MOTOR_M4_SIGN), 0U, 1U);
  (void)Motor_Sync_Start();
}

static uint32_t Chassis_EstimateMoveTimeMs(float maximum_angle_deg,
                                           uint16_t speed_rpm,
                                           uint8_t accel)
{
  float cruise_time_ms;
  uint16_t effective_rpm = speed_rpm;

  /* 加速度档位暂无物理加速度换算，使用保守倍率覆盖起停过程。 */
  (void)accel;
  if (effective_rpm == 0U)
  {
    return CHASSIS_POSITION_SETTLE_MARGIN_MS;
  }
  if (effective_rpm > MOTOR_MAX_RPM)
  {
    effective_rpm = MOTOR_MAX_RPM;
  }

  cruise_time_ms = Chassis_Abs(maximum_angle_deg) * 1000.0f /
                   ((float)effective_rpm * 6.0f);
  return (uint32_t)(cruise_time_ms * CHASSIS_POSITION_TIME_FACTOR +
                    (float)CHASSIS_POSITION_SETTLE_MARGIN_MS + 0.5f);
}

static void Chassis_MoveWheelsPosition(const MecanumWheels_t *wheels,
                                       float distance_scale,
                                       uint16_t speed_rpm,
                                       uint8_t accel)
{
  float angle1;
  float angle2;
  float angle3;
  float angle4;
  float maximum_angle;

  angle1 = distance_to_motor_angle(wheels->m1 * distance_scale) * MOTOR_M1_SIGN;
  angle2 = distance_to_motor_angle(wheels->m2 * distance_scale) * MOTOR_M2_SIGN;
  angle3 = distance_to_motor_angle(wheels->m3 * distance_scale) * MOTOR_M3_SIGN;
  angle4 = distance_to_motor_angle(wheels->m4 * distance_scale) * MOTOR_M4_SIGN;

  maximum_angle = Chassis_Abs(angle1);
  if (Chassis_Abs(angle2) > maximum_angle) maximum_angle = Chassis_Abs(angle2);
  if (Chassis_Abs(angle3) > maximum_angle) maximum_angle = Chassis_Abs(angle3);
  if (Chassis_Abs(angle4) > maximum_angle) maximum_angle = Chassis_Abs(angle4);

  (void)Motor_Move_Position(MOTOR_M1_ID, angle1, speed_rpm, accel, 1U);
  (void)Motor_Move_Position(MOTOR_M2_ID, angle2, speed_rpm, accel, 1U);
  (void)Motor_Move_Position(MOTOR_M3_ID, angle3, speed_rpm, accel, 1U);
  (void)Motor_Move_Position(MOTOR_M4_ID, angle4, speed_rpm, accel, 1U);
  (void)Motor_Sync_Start();

  HAL_Delay(Chassis_EstimateMoveTimeMs(maximum_angle, speed_rpm, accel));
}

void Chassis_MoveX(float distance_mm, uint16_t speed_rpm, uint8_t accel)
{
  MecanumWheels_t wheels;

  Mecanum_Inverse(distance_mm * CHASSIS_X_DISTANCE_SCALE *
                  CHASSIS_FORWARD_SIGN,
                  0.0f, 0.0f, &wheels);
  Chassis_MoveWheelsPosition(&wheels, 1.0f,
                             speed_rpm, accel);
}

void Chassis_MoveY(float distance_mm, uint16_t speed_rpm, uint8_t accel)
{
  MecanumWheels_t wheels;

  Mecanum_Inverse(0.0f,
                  distance_mm * CHASSIS_Y_DISTANCE_SCALE *
                  CHASSIS_LATERAL_SIGN,
                  0.0f, &wheels);
  Chassis_MoveWheelsPosition(&wheels, 1.0f,
                             speed_rpm, accel);
}

void Chassis_MoveDistance(float vx, float vy, float distance_mm,
                          float target_yaw)
{
  int32_t start_m1;
  int32_t start_m2;
  int32_t start_m3;
  int32_t start_m4;
  float direction_sign;
  float target_distance;
  float previous_error = 0.0f;
  float speed_magnitude = sqrtf(vx * vx + vy * vy);

  if ((speed_magnitude <= 0.0f) || (Chassis_Abs(distance_mm) <= 0.0f))
  {
    Chassis_Stop();
    return;
  }

  /* 记录运动起点位置。 */
  Chassis_ReadPosition(&start_m1, &start_m2, &start_m3, &start_m4);
  target_distance = Chassis_Abs(distance_mm);
  direction_sign = (distance_mm < 0.0f) ? -1.0f : 1.0f;

  while (1)
  {
    float current_distance;
    float remaining_distance;
    float speed_scale = 1.0f;
    float command_vx;
    float command_vy;
    float current_yaw;
    float error;
    float derivative;
    float wz;

    /* 读取当前移动距离。 */
    current_distance = Chassis_ReadDistance((float)start_m1,
                                             (float)start_m2,
                                             (float)start_m3,
                                             (float)start_m4,
                                             vx * direction_sign,
                                             vy * direction_sign);
    if (current_distance >= target_distance)
    {
      break;
    }

    remaining_distance = target_distance - current_distance;
    if (remaining_distance <= MOVE_STOP_DISTANCE_MM)
    {
      break;
    }
    if (remaining_distance <= MOVE_DECEL_START_DISTANCE_MM)
    {
      /* 剩余距离越小，平移速度越低。 */
      speed_scale = remaining_distance / MOVE_DECEL_START_DISTANCE_MM;
      if ((MOVE_MIN_SPEED_MM_S < speed_magnitude) &&
          (speed_scale < MOVE_MIN_SPEED_MM_S / speed_magnitude))
      {
        speed_scale = MOVE_MIN_SPEED_MM_S / speed_magnitude;
      }
    }
    command_vx = vx * direction_sign * speed_scale;
    command_vy = vy * direction_sign * speed_scale;

    /* 航向误差计算。 */
    current_yaw = IMU_GetYaw();
    error = Chassis_WrapAngle(target_yaw - current_yaw);
    derivative = (error - previous_error) /
                 ((float)MOVE_CONTROL_PERIOD_MS / 1000.0f);
    previous_error = error;

    /* PID修正旋转速度。 */
    wz = MOVE_HEADING_KP * error + MOVE_HEADING_KD * derivative;
    if (wz > MOVE_HEADING_MAX_WZ) wz = MOVE_HEADING_MAX_WZ;
    if (wz < -MOVE_HEADING_MAX_WZ) wz = -MOVE_HEADING_MAX_WZ;

    Chassis_Control(command_vx, command_vy, wz);
    HAL_Delay(MOVE_CONTROL_PERIOD_MS);
  }

  Chassis_Stop();
}
