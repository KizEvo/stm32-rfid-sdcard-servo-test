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
	SysTick->VAL &= ~(UINT32_MAX); 																					// Clear previous value in the register
	SysTick->CTRL = SysTick_CTRL_ENABLE;																		// Enable systick
	while(!((SysTick->CTRL & SysTick_CTRL_COUNTFLAG) != 0));								// Wait for the hardware to set the flag
	
	SysTick->CTRL &= ~(SysTick_CTRL_ENABLE);																// Disable systick
}


/*============================================================================
 *
 * Start systick timeout function, take "time" in milisecond as parameter
 *
 *============================================================================
 */
void startTimeOutMs(uint32_t time)
{
	timeOutFlag = 0;																												// Clear timeOutFlag
	NVIC_EnableIRQ(SysTick_IRQn);
	SysTick->LOAD = (CLOCK_PERIOD_ONE_MS * time);
	SysTick->VAL &= ~(UINT32_MAX); 																					// Clear previous value in the register
	SysTick->CTRL = (SysTick_CTRL_TICKINT | SysTick_CTRL_ENABLE);						// Enable systick and systick interrupt
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
