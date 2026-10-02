#ifndef MISSION_TEST_FAST_BLOCK_H
#define MISSION_TEST_FAST_BLOCK_H

#include <stdint.h>

extern volatile uint8_t fast_block_test_path_started;
extern volatile uint8_t fast_block_test_done;
extern volatile uint8_t fast_block_test_error;

void Mission_TestFastBlock_Init(void);
void Mission_TestFastBlock_Start(void);
void Mission_TestFastBlock_Stop(void);
void Mission_TestFastBlock_Run(void);
uint8_t Mission_TestFastBlock_IsFinished(void);

#endif /* MISSION_TEST_FAST_BLOCK_H */
