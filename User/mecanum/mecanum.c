#include "mecanum.h"
#include "chassis_config.h"

#define MECANUM_PI  3.14159265358979323846f

void Mecanum_Inverse(float x, float y, float rotation_rad,
                     MecanumWheels_t *wheels)
{
  float rotation_component;

  if (wheels == 0)
  {
    return;
  }

  rotation_component = rotation_rad * ROTATE_FACTOR;
  wheels->m1 = x - y - rotation_component;
  wheels->m2 = x + y + rotation_component;
  wheels->m3 = x + y - rotation_component;
  wheels->m4 = x - y + rotation_component;
}

float distance_to_motor_angle(float distance_mm)
{
  return distance_mm * 360.0f / (MECANUM_PI * CHASSIS_WHEEL_DIAMETER);
}
