#ifndef CHASSIS_H
#define CHASSIS_H

#include <stdint.h>

/* 底盘基础接口；Chassis_Control 是后续周期任务的连续运动入口。 */
void Chassis_Init(void);
void Chassis_Stop(void);
void Chassis_NormalStop(void);
void Chassis_Control(float vx, float vy, float wz);
/* 当前实车验证使用的阻塞式接口，后续再升级为周期任务控制。 */
void Chassis_MoveX(float distance_mm, uint16_t speed_rpm, uint8_t accel);
void Chassis_MoveY(float distance_mm, uint16_t speed_rpm, uint8_t accel);
void Chassis_MoveDistance(float vx, float vy, float distance_mm,
                          float target_yaw);

#endif /* CHASSIS_H */
