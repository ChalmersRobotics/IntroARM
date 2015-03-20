/*
 * IR.c
 *
 *  Created on: 26 jan 2013
 *      Author: peter
 */
#include "stm32f10x_conf.h"
#include "stm32f10x.h"
#include "IR.h"


volatile unsigned char ir1Active=0;
volatile unsigned char ir2Active=0;
volatile unsigned char ir3Active=0;
volatile unsigned char ir4Active=0;

void irInit()
{
	GPIO_InitTypeDef GPIOInitStruct;
	TIM_TimeBaseInitTypeDef	TimeBaseInitStruct;
	TIM_OCInitTypeDef OCInitStruct;
	NVIC_InitTypeDef NVIC_Init_Struct;

	NVIC_Init_Struct.NVIC_IRQChannel=IR_TIM_IRQn;
	NVIC_Init_Struct.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_Init_Struct);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM17, ENABLE);

	//Init IR pulse pin
	GPIOInitStruct.GPIO_Mode=GPIO_Mode_AF_OD;
	GPIOInitStruct.GPIO_Speed=GPIO_Speed_50MHz;
#if HARDWARE_REV == 3
	GPIOInitStruct.GPIO_Pin=GPIO_Pin_9;
	GPIO_Init(GPIOB, &GPIOInitStruct);
#elif HARDWARE_REV == 2
	GPIOInitStruct.GPIO_Pin=GPIO_Pin_3;
	GPIO_Init(GPIOA, &GPIOInitStruct);
#endif

	//Init Front receivers
#if HARDWARE_REV == 3
	GPIOInitStruct.GPIO_Mode=GPIO_Mode_IN_FLOATING;
	GPIOInitStruct.GPIO_Pin=GPIO_Pin_3;
	GPIOInitStruct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIOInitStruct);
#elif HARDWARE_REV == 2
	GPIOInitStruct.GPIO_Mode=GPIO_Mode_IN_FLOATING;
	GPIOInitStruct.GPIO_Pin=GPIO_Pin_9;
	GPIOInitStruct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIOInitStruct);
#endif
	GPIOInitStruct.GPIO_Mode=GPIO_Mode_IN_FLOATING;
	GPIOInitStruct.GPIO_Pin=GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIOInitStruct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIOInitStruct);


	// Init pulse timer
	TimeBaseInitStruct.TIM_Prescaler=0;
	TimeBaseInitStruct.TIM_CounterMode=TIM_CounterMode_Up;
	TimeBaseInitStruct.TIM_Period=SystemCoreClock/IR_FREQ; //Set frequency to 38 kHz
	TimeBaseInitStruct.TIM_ClockDivision=0;
	TimeBaseInitStruct.TIM_RepetitionCounter=32;//Set repetition to 32 pulses
	TIM_TimeBaseInit(IR_TIM, &TimeBaseInitStruct);

	//Duty Cycle should be 25%-33%
	//32 pulses per active period
	//~400 pulses per silent period

	OCInitStruct.TIM_OCMode=TIM_OCMode_PWM1;
	OCInitStruct.TIM_Pulse=0; //Will be set directly later
	OCInitStruct.TIM_OutputState=TIM_OutputState_Enable;
	OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_Low;
	OCInitStruct.TIM_OCIdleState = TIM_OCIdleState_Set;	//Since P-channel FET
#if HARDWARE_REV == 3
	TIM_OC1Init(IR_TIM, &OCInitStruct);
	TIM_OC1PreloadConfig(IR_TIM, TIM_OCPreload_Enable);
#elif HARDWARE_REV == 2
	TIM_OC2Init(IR_TIM, &OCInitStruct);
	TIM_OC2PreloadConfig(IR_TIM, TIM_OCPreload_Enable);
#endif
	//Used since the library is incorrect
	IR_TIM->CR2 |= (uint16_t)(TIM_OCIdleState_Set << 2);


	TIM_ARRPreloadConfig(IR_TIM, ENABLE);
	// TIM counter enable //
	TIM_Cmd(IR_TIM, ENABLE);

	//Enable Repetition counter interrupt
	TIM_ITConfig(IR_TIM,TIM_IT_Update,ENABLE);

	// TIM Main Output Enable //
	TIM_CtrlPWMOutputs(IR_TIM, ENABLE);

	// Set pulse duty cycle
	IR_TIM->CCR1=SystemCoreClock/IR_FREQ/4;

}

/*
 * Enable pulsing of the IR-LEDs.
 * The sensor values will be frozen when the pulsing is disabled
 */
void irDisable()
{
	NVIC_DisableIRQ(IR_TIM_IRQn);
	TIM_CtrlPWMOutputs(IR_TIM,DISABLE);
}

/*
 * Enable pulsing of the IR-LEDs.
 */
void irEnable()
{
	NVIC_EnableIRQ(IR_TIM_IRQn);
	TIM_CtrlPWMOutputs(IR_TIM,ENABLE);
}

/*
 * Set the frequency of the pulsing of the IR-LEDs
 */
void irSetFreq(unsigned short freq)
{
	if(freq<IR_FREQ_MIN)
	{
		freq=IR_FREQ_MIN;
	}
	else if(freq>IR_FREQ_MAX)
	{
		freq=IR_FREQ_MAX;
	}
	IR_TIM->CCR1=SystemCoreClock/freq/4;
	TIM_SetAutoreload(IR_TIM,SystemCoreClock/freq);

}

/*
 * Set the sensitivity of the IR-receivers, by setting the frequency of the pulse.
 * sens is a number between 0 and IR_SENS_MAX. Higher value -> more sensitive
 */
void irSetSens(unsigned char sens)
{
	if (sens>IR_SENS_MAX)
	{
		sens=IR_SENS_MAX;
	}
	irSetFreq(IR_FREQ_MIN + IR_SENS_FREQ_STEP*(IR_SENS_MAX-sens));
}

#define PULSE_MAX_TIME	1

//Repetition counter interrupt (I really don't know what goes on here, but it works :) )
void IR_TIM_IRQ()
{
	TIM_ClearFlag(IR_TIM,TIM_FLAG_Update);
	static unsigned char pulsing=0;
	static unsigned char ir1Seen=0;
	static unsigned char ir2Seen=0;
	static unsigned char ir3Seen=0;
	static unsigned char ir4Seen=0;
	if(pulsing)
	{
		if(pulsing==PULSE_MAX_TIME)
		{
			TIM_CtrlPWMOutputs(IR_TIM, ENABLE);
		}
		else
		{
			TIM_CtrlPWMOutputs(IR_TIM, DISABLE);
		}

		IR_TIM->RCR=250; //250
		pulsing--;
	}
	else
	{
		//Low pass filters for each sensor
		if(IRMOT1 && ir1Seen<IR_FILTER_MAX)
		{
			if(++ir1Seen==IR_FILTER_MAX)
			{
				ir1Active=1;
			}
		}
		else if(ir1Seen)
		{
			if(!--ir1Seen)
			{
				ir1Active=0;
			}
		}

		if(IRMOT2 && ir2Seen<IR_FILTER_MAX)
		{
			if(++ir2Seen==IR_FILTER_MAX)
			{
				ir2Active=1;
			}
		}
		else if(ir2Seen)
		{
			if(!--ir2Seen)
			{
				ir2Active=0;
			}
		}

		if(IRMOT3 && ir3Seen<IR_FILTER_MAX)
		{
			if(++ir3Seen==IR_FILTER_MAX)
			{
				ir3Active=1;
			}
		}
		else if(ir3Seen)
		{
			if(!--ir3Seen)
			{
				ir3Active=0;
			}
		}

		//IRMOT4
		if(IRMOT4 && ir4Seen<IR_FILTER_MAX)
		{
			if(++ir4Seen==IR_FILTER_MAX)
			{
				ir4Active=1;
			}
		}
		else if(ir4Seen)
		{
			if(!--ir4Seen)
			{
				ir4Active=0;
			}
		}
		TIM_CtrlPWMOutputs(IR_TIM, DISABLE);
		IR_TIM->RCR=32;
		pulsing=PULSE_MAX_TIME;
	}
}


