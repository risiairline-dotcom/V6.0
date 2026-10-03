#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include <stdint.h>

void Chassis_Task_Init(void);
void Chassis_Move_Start(float vx, float vy, float distance_mm,
                        float target_yaw);
void Chassis_Task(void);
uint8_t Chassis_IsFinished(void);

/*
 * 功能：获取当前底盘移动剩余距离。
 * 参数：无。
 * 返回：剩余距离，单位 mm；没有移动任务时返回 0。
 * 说明：该接口只用于状态查询，不参与底盘控制。
 */
float Chassis_GetRemainDistance(void);

#endif /* CHASSIS_TASK_H */
