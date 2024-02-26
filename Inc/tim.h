#ifndef TIM_H
#define TIM_H

#include <stdint.h>

void TIM_PWM_InitConfig(void);

void delayMs(uint32_t time);
void startTimeOutMs(uint32_t time);
void endTimeOutMs(void);
uint8_t checkTimeOut(void);

#endif
