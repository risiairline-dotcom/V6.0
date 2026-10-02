#include "chassis_profile.h"
#include "chassis_config.h"
#include <math.h>

static float profile_speed_mm_s;
static float profile_accel_mm_s2;

static float Chassis_ProfileBrakingDistanceToTarget(
    float current_speed_mm_s,
    float target_speed_mm_s,
    float current_accel_mm_s2,
    float max_decel_mm_s2,
    float max_jerk_mm_s3)
{
  float release_duration = 0.0f;
  float release_distance = 0.0f;
  float release_speed;
  float speed_delta;
  float jerk_phase_duration;
  float constant_decel_duration = 0.0f;
  float triangular_speed_delta;
  float total_duration;

  if ((max_decel_mm_s2 <= 0.0f) || (max_jerk_mm_s3 <= 0.0f))
    return 0.0f;
  if (current_speed_mm_s < 0.0f) current_speed_mm_s = 0.0f;
  if (target_speed_mm_s < 0.0f) target_speed_mm_s = 0.0f;
  if (target_speed_mm_s >= current_speed_mm_s)
    return 0.0f;

  /* 先解除当前正加速度，避免低估制动距离。 */
  if (current_accel_mm_s2 < 0.0f) current_accel_mm_s2 = 0.0f;
  if (current_accel_mm_s2 > max_decel_mm_s2)
    current_accel_mm_s2 = max_decel_mm_s2;
  if (current_accel_mm_s2 > 0.0f)
  {
    release_duration = current_accel_mm_s2 / max_jerk_mm_s3;
    release_distance = current_speed_mm_s * release_duration +
                       0.5f * current_accel_mm_s2 *
                       release_duration * release_duration -
                       max_jerk_mm_s3 * release_duration *
                       release_duration * release_duration / 6.0f;
    current_speed_mm_s += current_accel_mm_s2 * release_duration -
                          0.5f * max_jerk_mm_s3 *
                          release_duration * release_duration;
  }

  speed_delta = current_speed_mm_s - target_speed_mm_s;
  triangular_speed_delta = max_decel_mm_s2 * max_decel_mm_s2 /
                           max_jerk_mm_s3;
  if (speed_delta <= triangular_speed_delta)
  {
    jerk_phase_duration = sqrtf(speed_delta / max_jerk_mm_s3);
  }
  else
  {
    jerk_phase_duration = max_decel_mm_s2 / max_jerk_mm_s3;
    constant_decel_duration =
        (speed_delta - triangular_speed_delta) / max_decel_mm_s2;
  }
  total_duration = 2.0f * jerk_phase_duration +
                   constant_decel_duration;

  return release_distance +
         0.5f * (current_speed_mm_s + target_speed_mm_s) *
         total_duration;
}

float JerkLimitedBrakingDistance(float current_speed_mm_s,
                                 float current_accel_mm_s2,
                                 float max_decel_mm_s2,
                                 float max_jerk_mm_s3)
{
  return Chassis_ProfileBrakingDistanceToTarget(current_speed_mm_s,
                                                0.0f,
                                                current_accel_mm_s2,
                                                max_decel_mm_s2,
                                                max_jerk_mm_s3);
}

static float Chassis_ProfileTargetSpeed(float max_speed_mm_s,
                                        float remaining_distance_mm)
{
  float available_distance = remaining_distance_mm -
                             MOVE_STOP_DISTANCE_MM;
  float braking_distance;
  float low_speed;
  float high_speed;
  float mid_speed;
  unsigned int i;

  if (available_distance <= 0.0f) return 0.0f;

  braking_distance = JerkLimitedBrakingDistance(
      profile_speed_mm_s, profile_accel_mm_s2,
      MOVE_DECEL_LIMIT, MOVE_JERK_LIMIT);
  if (braking_distance <= available_distance)
    return max_speed_mm_s;

  /* 根据可用距离反解连续目标速度。 */
  low_speed = 0.0f;
  high_speed = profile_speed_mm_s;
  for (i = 0U; i < 8U; i++)
  {
    mid_speed = 0.5f * (low_speed + high_speed);
    braking_distance = Chassis_ProfileBrakingDistanceToTarget(
        profile_speed_mm_s, mid_speed, profile_accel_mm_s2,
        MOVE_DECEL_LIMIT, MOVE_JERK_LIMIT);
    if (braking_distance > available_distance)
      high_speed = mid_speed;
    else
      low_speed = mid_speed;
  }
  return low_speed;
}

void Chassis_ProfileInit(void)
{
  profile_speed_mm_s = 0.0f;
  profile_accel_mm_s2 = 0.0f;
}

float Chassis_ProfileUpdate(float max_speed_mm_s,
                            float remaining_distance_mm,
                            float period_s)
{
  float target_speed_mm_s;
  float target_accel_mm_s2;
  float accel_delta;
  float accel_step;

  target_speed_mm_s = Chassis_ProfileTargetSpeed(max_speed_mm_s,
                                                 remaining_distance_mm);

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
