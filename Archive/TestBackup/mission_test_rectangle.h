#ifndef MISSION_TEST_RECTANGLE_H
#define MISSION_TEST_RECTANGLE_H

#include <stdint.h>

/* Keil Watch：done=1 时，final_x/y 为复位起点以来的闭合误差。 */
extern volatile uint8_t rectangle_test_done;
extern volatile uint8_t rectangle_test_error;
extern volatile float rectangle_final_x;
extern volatile float rectangle_final_y;
extern volatile float rectangle_final_yaw;

void Mission_TestRectangle_Init(void);
void Mission_TestRectangle_Start(void);
void Mission_TestRectangle_Stop(void);
void Mission_TestRectangle_Run(void);
uint8_t Mission_TestRectangle_IsFinished(void);

#endif /* MISSION_TEST_RECTANGLE_H */
