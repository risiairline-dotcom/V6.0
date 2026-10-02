#ifndef TEST_LIFT_H
#define TEST_LIFT_H

/* 上电读取 ID5 位置；PC0 依次设零点、上升到目标高度、回零点。 */
void Lift_Test_Init(void);
void Lift_Test_Run(void);

#endif /* TEST_LIFT_H */
