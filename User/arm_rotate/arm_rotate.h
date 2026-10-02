#ifndef ARM_ROTATE_H
#define ARM_ROTATE_H

/* SERVO_2 旋转抓取机构；姿态角由调用方指定。 */
void ArmRotate_Init(void);
void ArmRotate_MoveTo(float angle);

#endif /* ARM_ROTATE_H */
