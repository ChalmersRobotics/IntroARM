#ifndef __TIME_H__
#define __TIME_H__

#include "stm32f10x.h"

typedef struct{
	unsigned char hours;
	unsigned char minutes;
	unsigned char seconds;
}timeStruct;

//-----------------------------------------------------------------------
// External Variables
//-----------------------------------------------------------------------
extern volatile unsigned long systemTime;			//Contains the number of milliseconds since start
extern volatile unsigned long mpuTimeOut;			//Used for timeout for mpu6050 communication
//-----------------------------------------------------------------------
// External functions
//-----------------------------------------------------------------------
//
void timeInit(void);
void delay_ms(unsigned long ms);
void delay_us(unsigned long us);
unsigned long microSeconds();
void timeFormat(timeStruct* outStruct,unsigned long seconds);

void SysTick_Handler(void);
//-----------------------------------------------------------------------

//Constants. Input your clock frequency here
#ifndef SYSTEM_CORE_CLOCK
#define SYSTEM_CORE_CLOCK 24000000UL
#endif

#define TICKS_PER_US (SYSTEM_CORE_CLOCK/1000000)
#define TICKS_PER_MS (SYSTEM_CORE_CLOCK/1000)
#endif


