#ifndef TIM_H
#define TIM_H

#include <stdint.h>

void delayMs(uint32_t time);
void startTimeOutMs(uint32_t time);
void endTimeOutMs(void);
uint8_t checkTimeOut(void);

#endif
