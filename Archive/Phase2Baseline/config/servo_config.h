#ifndef SERVO_CONFIG_H
#define SERVO_CONFIG_H

/* TIM1计数频率为1 MHz，因此脉宽微秒数等于CCR数值。 */

/* ==================== 舵机1角度映射 ==================== */
#define SERVO_1_MIN_PULSE_US  500
#define SERVO_1_MAX_PULSE_US  2500
#define SERVO_1_MAX_ANGLE     270

/* ==================== 舵机3角度映射 ==================== */
#define SERVO_3_MIN_PULSE_US  500
#define SERVO_3_MAX_PULSE_US  2500
#define SERVO_3_MAX_ANGLE     270

#endif /* SERVO_CONFIG_H */
