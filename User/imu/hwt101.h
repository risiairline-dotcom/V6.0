#ifndef HWT101_H
#define HWT101_H

#include "stm32f4xx_hal.h"

extern volatile uint32_t hwt_rx_count;
extern volatile uint32_t hwt_frame_count;
extern volatile uint32_t hwt_angle_count;
extern volatile uint32_t hwt_gyro_count;
extern volatile uint32_t hwt_last_rx_tick;
extern volatile uint32_t imu_gyro_last_tick;

/* [DEBUG] 附加诊断变量，可用于判断校验或接收重启故障。 */
extern volatile uint32_t hwt_checksum_error_count;
extern volatile uint32_t hwt_rx_restart_error_count;

HAL_StatusTypeDef HWT101_Init(void);

/**
 * @brief  获取软件清零后的当前航向角
 * @return 航向角，单位 °，范围 -180° 到 +180°
 */
float IMU_GetYaw(void);

/**
 * @brief  将当前有效航向角设为软件零点
 */
void IMU_ZeroYaw(void);

/**
 * @brief  获取当前 Z 轴角速度
 * @return 角速度，单位 °/s
 */
float IMU_GetGyroZ(void);

#endif
