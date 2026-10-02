#ifndef LIFT_H
#define LIFT_H

#include <stdint.h>

/* 仅控制 CAN 地址 5 的升降轴；成功返回 1。 */
uint8_t Lift_Init(void);
/* angle 为电机轴相对角度；实测正值下降、负值上升，成功发送返回 1。 */
uint8_t Lift_MoveRelative(int32_t angle);
/* 相对位移单位 mm：正值下降，负值上升；单次最大 50 mm。 */
uint8_t Lift_MoveRelativeMm(float distance_mm);
/* 相对调试零点的目标高度：向上为正；单次运动仍受毫米接口限幅。 */
uint8_t Lift_MoveHeight(float height_mm);
/* 记录当前位置为调试零点；读取新反馈成功返回 1。 */
uint8_t Lift_SetZero(void);
/* 相对调试零点的高度，向上为正；零点/方向/反馈无效时返回 NAN。 */
float Lift_GetHeight(void);
void Lift_Stop(void);
/* 等待 ID5 新位置反馈；成功时写入原始位置并返回 1。 */
uint8_t Lift_GetPosition(int32_t *position);
/* 非阻塞调用入口；仅在周期到达时查询新反馈。 */
uint8_t Lift_IsArrived(void);

#endif /* LIFT_H */
