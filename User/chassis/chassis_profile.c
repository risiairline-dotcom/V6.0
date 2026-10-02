#include "chassis_profile.h"
#include "chassis_config.h"
#include <math.h>

static float profile_speed_mm_s;

void Chassis_ProfileInit(void)
{
  profile_speed_mm_s = 0.0f;
}

float Chassis_ProfileUpdate(float max_speed_mm_s,
                            float remaining_distance_mm,
                            float period_s)
{
  float braking_distance = remaining_distance_mm - MOVE_STOP_DISTANCE_MM;
  float target_speed_mm_s = 0.0f;
  float speed_delta;
  float speed_step;

  if (braking_distance > 0.0f)
  {
    target_speed_mm_s = sqrtf(2.0f * MOVE_DECEL_LIMIT * braking_distance);
    if (target_speed_mm_s > max_speed_mm_s)
    {
      target_speed_mm_s = max_speed_mm_s;
    }
  }

  speed_delta = target_speed_mm_s - profile_speed_mm_s;
  speed_step = ((speed_delta >= 0.0f) ? MOVE_ACCEL_LIMIT : MOVE_DECEL_LIMIT) *
               period_s;

  if (speed_delta > speed_step)
  {
    profile_speed_mm_s += speed_step;
  }
  else if (speed_delta < -speed_step)
  {
    profile_speed_mm_s -= speed_step;
  }
  else
  {
    profile_speed_mm_s = target_speed_mm_s;
  }

  return profile_speed_mm_s;
}
