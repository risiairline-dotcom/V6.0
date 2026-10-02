#include "arm_rotate.h"
#include "servo.h"

/* 当前尚未确定具体工作姿态，仅限定有限角度舵机的可用范围。 */
#define ARM_ROTATE_POSE_MIN_ANGLE  0
#define ARM_ROTATE_POSE_MAX_ANGLE  359

void ArmRotate_Init(void)
{
  /* 姿态尚未标定，上电不主动旋转抓取机构。 */
}

void ArmRotate_MoveTo(float angle)
{
  if (!(angle >= ARM_ROTATE_POSE_MIN_ANGLE &&
        angle <= ARM_ROTATE_POSE_MAX_ANGLE))
    return;

  Servo_SetAngle(SERVO_2, angle);
}
