#ifndef MISSION_TEST_PATH_H
#define MISSION_TEST_PATH_H

#include <stdint.h>

/* 独立底盘 Path 测试任务，不改变正式比赛路线。 */
void Mission_TestPath_Init(void);
void Mission_TestPath_Start(void);
void Mission_TestPath_Stop(void);
void Mission_TestPath_Run(void);
uint8_t Mission_TestPath_IsFinished(void);

#endif /* MISSION_TEST_PATH_H */
