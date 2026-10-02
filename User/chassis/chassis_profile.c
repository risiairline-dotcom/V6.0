#include "chassis_profile.h"
#include "chassis_config.h"
#include <math.h>

static float profile_speed_mm_s;
static float profile_accel_mm_s2;

void Chassis_ProfileInit(void)
{
  profile_speed_mm_s = 0.0f;
  profile_accel_mm_s2 = 0.0f;
}

float Chassis_ProfileUpdate(float max_speed_mm_s,
                            float remaining_distance_mm,
                            float period_s)
{
  float braking_distance = remaining_distance_mm - MOVE_STOP_DISTANCE_MM;
  float target_speed_mm_s = 0.0f;
  float target_accel_mm_s2;
  float accel_delta;
  float accel_step;

  if (braking_distance > 0.0f)
  {
    target_speed_mm_s = sqrtf(2.0f * MOVE_DECEL_LIMIT * braking_distance);
    if (target_speed_mm_s > max_speed_mm_s)
    {
      target_speed_mm_s = max_speed_mm_s;
    }
  }

  if (period_s <= 0.0f) return profile_speed_mm_s;

  /* 先得到目标加速度，再限制加速度变化率。 */
  target_accel_mm_s2 = (target_speed_mm_s - profile_speed_mm_s) / period_s;
  if (target_accel_mm_s2 > MOVE_ACCEL_LIMIT)
    target_accel_mm_s2 = MOVE_ACCEL_LIMIT;
  if (target_accel_mm_s2 < -MOVE_DECEL_LIMIT)
    target_accel_mm_s2 = -MOVE_DECEL_LIMIT;

  accel_delta = target_accel_mm_s2 - profile_accel_mm_s2;
  accel_step = MOVE_JERK_LIMIT * period_s;
  if (accel_delta > accel_step)
    accel_delta = accel_step;
  if (accel_delta < -accel_step)
    accel_delta = -accel_step;

  profile_accel_mm_s2 += accel_delta;
  profile_speed_mm_s += profile_accel_mm_s2 * period_s;

  /* 防止离散周期越过目标速度。 */
  if ((profile_accel_mm_s2 >= 0.0f) &&
      (profile_speed_mm_s > target_speed_mm_s))
  {
    profile_speed_mm_s = target_speed_mm_s;
  }
  else if ((profile_accel_mm_s2 < 0.0f) &&
           (profile_speed_mm_s < target_speed_mm_s))
  {
    profile_speed_mm_s = target_speed_mm_s;
  }

  return profile_speed_mm_s;
}
