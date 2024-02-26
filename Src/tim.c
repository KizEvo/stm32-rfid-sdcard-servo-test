/* ============================================================================
 *
 * tim.c
 *
 * Created on: Feb 25, 2024
 * Author: Nguyen Duc Phu
 *
 * Description: AHB clock is 56MHz and the clock input for systick is AHB/8 = 7MHz.
 * 							So 1ms equals 7000 clock period
 *
 * ============================================================================
 */

#include <stdint.h>
#include "tim.h"
#include "stm32f10x.h"
#include "stm32f10x_tim.h" // Keil::Device:StdPeriph Drivers:TIM
#include "core_cm3.h"

static uint32_t CLOCK_PERIOD_ONE_MS = 6999;	// N = 7000 (required) => N - 1 = 6999 (period) = 1 (ms)
static volatile uint8_t timeOutFlag = 0;

/*============================================================================
 *
 * Delay function in miliseconds that use SysTick hardware, take "time" in milisecond as parameter
 * No delayMs function should be called between startTimeOutMs and endTimeOutMs
 * 
 * ============================================================================
 */
void delayMs(uint32_t time)
{
	SysTick->LOAD = (CLOCK_PERIOD_ONE_MS * time);
	SysTick->VAL &= ~(UINT32_MAX);	// Clear previous value in the register
	SysTick->CTRL = SysTick_CTRL_ENABLE;	// Enable systick
	while(!((SysTick->CTRL & SysTick_CTRL_COUNTFLAG) != 0));	// Wait for the hardware to set the flag
	
	SysTick->CTRL &= ~(SysTick_CTRL_ENABLE);	// Disable systick
}


/*============================================================================
 *
 * Start systick timeout function, take "time" in milisecond as parameter
 *
 *============================================================================
 */
void startTimeOutMs(uint32_t time)
{
	timeOutFlag = 0;	// Clear timeOutFlag
	NVIC_EnableIRQ(SysTick_IRQn);
	SysTick->LOAD = (CLOCK_PERIOD_ONE_MS * time);
	SysTick->VAL &= ~(UINT32_MAX);	// Clear previous value in the register
	SysTick->CTRL = (SysTick_CTRL_TICKINT | SysTick_CTRL_ENABLE);	// Enable systick and systick interrupt
}

/*============================================================================
 *
 * End systick timeout function, startTimeOutMs() must be called before calling
 * this function.
 * endTimeOutMs should be called after timeOutFlag is set or when a task finished
 * before timeOutFlag is set
 *
 *============================================================================
 */
void endTimeOutMs(void)
{
	SysTick->CTRL &= ~(SysTick_CTRL_TICKINT | SysTick_CTRL_ENABLE);
	timeOutFlag = 0;
}

/*============================================================================
 *
 * Get timeout status
 * Return 1 if timeOutFlag is set by SysTick IRQ else 0
 *
 *============================================================================
 */
uint8_t checkTimeOut(void)
{
	return timeOutFlag;
}

/*============================================================================
 *
 * SysTick interrupt service routine
 * Set timeOutFlag and disable systick interrupt
 *
 *============================================================================
 */
void SysTick_Handler(void)
{
	timeOutFlag = 1;
	SysTick->CTRL &= ~(SysTick_CTRL_TICKINT | SysTick_CTRL_ENABLE);
}


/*============================================================================
 *
 * Setup TIM2 to work in PWM mode on APB1 bus
 *
 *============================================================================
 */
void TIM_PWM_InitConfig(void)
{
	// Turn on TIM2 clock
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
	
	// Config IO for TIM2_CH2
	GPIO_InitTypeDef TIM2_CH2_IO;
	TIM2_CH2_IO.GPIO_Mode = GPIO_Mode_AF_PP;
	TIM2_CH2_IO.GPIO_Pin = GPIO_Pin_1;
	TIM2_CH2_IO.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(GPIOA, &TIM2_CH2_IO);
	
	uint16_t arrValue = 2240 * 2;
	uint16_t dutyCycle = 5;
	
	// Setup TIM2 parameters
	TIM_TimeBaseInitTypeDef TIM2_TimeBase;
	TIM2_TimeBase.TIM_CounterMode = TIM_CounterMode_Up;
	TIM2_TimeBase.TIM_Prescaler = 250 - 1;
	TIM2_TimeBase.TIM_Period = arrValue;
	
	TIM_ARRPreloadConfig(TIM2, ENABLE);
	
	TIM_OCInitTypeDef TIM2_PWM;
	TIM2_PWM.TIM_OCMode = TIM_OCMode_PWM1;
	TIM2_PWM.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM2_PWM.TIM_Pulse = (arrValue * dutyCycle) / 100;
	TIM2_PWM.TIM_OutputState = TIM_OutputState_Enable;
	
	TIM_OC2Init(TIM2, &TIM2_PWM);
	TIM_TimeBaseInit(TIM2, &TIM2_TimeBase);
	TIM_CCxCmd(TIM2, TIM_Channel_2, TIM_CCx_Enable);
	TIM_Cmd(TIM2, ENABLE);
}
