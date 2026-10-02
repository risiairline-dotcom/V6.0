#ifndef EXTEND_H
#define EXTEND_H

#include <stdint.h>

/* ID6 伸缩轴；上电不自动运动。 */
void Extend_Init(void);
/* 初始化时已收到 ID6 新位置反馈才返回 1。 */
uint8_t Extend_IsReady(void);
/* angle 为电机轴相对角度，正负号决定转向。 */
void Extend_MoveRelative(int32_t angle);
/* 最近一次相对运动命令是否通过反馈检查并成功交给 Motor 层。 */
uint8_t Extend_CommandAccepted(void);
void Extend_Stop(void);
/* 返回新位置反馈的原始单位；读取失败返回 INT32_MIN。 */
int32_t Extend_GetPosition(void);
/* target 使用与 GetPosition 相同的原始反馈单位。 */
uint8_t Extend_IsArrived(int32_t target);

#endif /* EXTEND_H */
