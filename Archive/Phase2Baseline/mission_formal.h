#ifndef MISSION_FORMAL_H
#define MISSION_FORMAL_H

#include <stdint.h>

void Mission_Formal_Init(void);
void Mission_Formal_Start(void);
void Mission_Formal_Run(void);
void Mission_Formal_Stop(void);
uint8_t Mission_Formal_IsFinished(void);

#endif /* MISSION_FORMAL_H */
