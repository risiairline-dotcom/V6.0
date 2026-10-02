#ifndef CHASSIS_H
#define CHASSIS_H

#include <stdint.h>

/* Base motor control only. Motion planning is intentionally unavailable. */
void Chassis_Init(void);
void Chassis_Stop(void);
void Chassis_Control(float vx, float vy, float wz);
void Chassis_MoveX(float distance_mm, uint16_t speed_rpm, uint8_t accel);
void Chassis_MoveY(float distance_mm, uint16_t speed_rpm, uint8_t accel);
void Chassis_MoveDistance(float vx, float vy, float distance_mm,
                          float target_yaw);

#endif /* CHASSIS_H */
