#ifndef TURNTABLE_CONFIG_H
#define TURNTABLE_CONFIG_H

/* ==================== 转盘工位角度 ==================== */
/* 三个工位角度相互独立，单位为 degree。 */
#define TURNTABLE_POS_1_ANGLE  45
#define TURNTABLE_POS_2_ANGLE  135
#define TURNTABLE_POS_3_ANGLE  225

/* ==================== 转盘完成判断 ==================== */
/* 无位置反馈，使用保守等待时间判断旋转盘动作完成。 */
#define TURNTABLE_MOVE_TIME_MS  1000

#endif /* TURNTABLE_CONFIG_H */
