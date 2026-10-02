#ifndef HEADING_PID_H
#define HEADING_PID_H

/**
 * @brief  初始化航向控制状态
 */
void Heading_Init(void);

/**
 * @brief  角度与陀螺仪阻尼控制绝对航向，稳定后停止
 * @param  target_deg 目标航向角，单位 °，正值左转
 */
void Heading_RotateTo(float target_deg);

/**
 * @brief  按当前航向相对旋转
 * @param  delta_deg 相对旋转角度，单位 °，正值左转
 */
void Heading_RotateRelative(float delta_deg);

#endif /* HEADING_PID_H */
