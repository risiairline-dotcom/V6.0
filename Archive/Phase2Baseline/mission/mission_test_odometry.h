#ifndef MISSION_TEST_ODOMETRY_H
#define MISSION_TEST_ODOMETRY_H

#include <stdint.h>

/* Keil Watch 输出；distance_* = end_* - start_*，yaw_change 同理。 */
extern volatile float odom_test_start_x;
extern volatile float odom_test_end_x;
extern volatile float odom_test_distance_x;
extern volatile float odom_test_start_y;
extern volatile float odom_test_end_y;
extern volatile float odom_test_distance_y;
extern volatile float odom_test_start_yaw;
extern volatile float odom_test_end_yaw;
extern volatile float odom_test_yaw_change;
extern volatile uint8_t odom_test_done;
extern volatile uint8_t odom_test_error;

void Mission_TestOdometry_Init(void);
void Mission_TestOdometry_Start(void);
void Mission_TestOdometry_Stop(void);
void Mission_TestOdometry_Run(void);
uint8_t Mission_TestOdometry_IsFinished(void);

#endif /* MISSION_TEST_ODOMETRY_H */
