#ifndef TEST_GRIPPER_H
#define TEST_GRIPPER_H

/* 当前 SERVO_1 逻辑通道仅用于 PE9 夹爪测试，实际比赛功能不改变。 */
/* 初始化 TIM1_CH1 PWM 并设置起始角度。 */
void Gripper_Test_Init(void);
/* 每次 PC0 按下后在两个测试角度之间切换。 */
void Gripper_Test_Run(void);

#endif /* TEST_GRIPPER_H */
