#include "time.h"
#include "mpu6050.h"

volatile unsigned long systemTime=0; //useful global variable that indicates milliseconds from boot
volatile unsigned long mpuTimeOut=0;

void SysTick_Handler(void)
{
	systemTime++;

	static unsigned long mpuCallProcess=100;
	if(!mpuCallProcess--)
	{
		mpuCallProcess=50;
		mpu6050Process();
	}
	if(mpuTimeOut)
	{
		mpuTimeOut--;
	}

}

void timeInit(void)
{
	RCC_ClocksTypeDef RCC_Clocks;
	// SystTick configuration: an interrupt every 1ms
	RCC_GetClocksFreq(&RCC_Clocks);
	if (SysTick_Config(RCC_Clocks.SYSCLK_Frequency / 1000))
	{
			// Capture error
			while (1) {}
	}
	// Update the SysTick IRQ priority should be higher than others
	NVIC_SetPriority(SysTick_IRQn,0);
}

/*
 * delays the processor for a number of ms
 * during delay, the processor can only respond to interrupts
 */
void delay_ms(unsigned long ms)
{
	unsigned long endTime = systemTime + ms;
	while(endTime>systemTime){}
}

/*
 * Allows for very short delays
 * us should be less than 1000, otherwise, there might be glitches
 * during delay, the processor can only respond to interrupts
 */
void delay_us(unsigned long us)
{
	volatile unsigned long enterVal = SysTick->VAL;
	volatile unsigned long finishVal = enterVal-us*TICKS_PER_US+31;	//adjust for the time this function takes to run

	if(finishVal >= enterVal)	//timer will wrap around during delay
	{
		finishVal+=TICKS_PER_MS;	//finishVal will wrap around and adjust itself correctly
		while((SysTick->VAL < enterVal) || (finishVal < SysTick->VAL)){}	//we know we need to go down to systick=0. after that we count to the correct value
	}
	else
	{
		while(finishVal < SysTick->VAL){}
	}
}

/*
 * Returns the number of microseconds since start
 */
unsigned long microSeconds()
{
	unsigned long tickVal = SysTick->VAL;
	return (systemTime*1000 + (TICKS_PER_MS-tickVal)/TICKS_PER_US);
}

/*
 * Formats seconds to hours. minutes and seconds
 * Parameters are a pointer to the struct you want the value in and and a total of seconds
 */
void timeFormat(timeStruct* outStruct,unsigned long seconds)
{
	outStruct->hours=seconds/3600;
	outStruct->minutes=seconds/60-(outStruct->hours)*60;
	outStruct->seconds=seconds%60;
}
