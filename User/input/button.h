#ifndef BUTTON_H
#define BUTTON_H

#include <stdint.h>

void Button_Init(void);
void Button_Update(void);
uint8_t Button_GetStartEvent(void);
uint8_t Button_GetStopEvent(void);
uint8_t Button_IsOn(void);

#endif /* BUTTON_H */
