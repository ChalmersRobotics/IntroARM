/*
 *	buzzer.c
 *
 *	Created on: 22 feb 2015
 *		Author: Sterna
 *
 *		Controls the on-board buzzer. It can only beep in different frequencies and is not level-controlled.
 *		It is therefore not used with the DAC.
 *		Utilizes, Timer16_CH1, DMA1_CH6, PB8
 */

#include "buzzer.h"

//Debug
#include "uart.h"
#include "xprintf.h"
#include "utils.h"

//The timer base frequency in Hz
#define BUZZER_TIMER_FREQ	1000000

//---------Start of melodies------------//

const uint16_t itemCatch[]={
		A4,160,
		AS4,210,
		B4,210,
		C5,800,
		0,0
};

const uint16_t secretDiscovered[]={
		G5, 180,
		FS5, 150,
		DS5, 180,
		A4, 180,
		GS4, 190,
		E5, 190,
		GS5, 210,
		C6, 500,
		0,0
};

const uint16_t mm4Victory[]={
		DS4,100,
		0,5,
		DS4,100,
		0,175,
		F4, 100,
		0,5,
		F4,100,
		0,175,
		FS4, 100,
		0,5,
		FS4,100,
		0,175,
		GS4, 100,
		FS4, 100,
		GS4, 100,
		AS4,1000,
		0,0
};

const uint16_t oneUp[]={
		D5,90,
		0,70,
		F5,90,
		0,70,
		D6,90,
		0,70,
		B6,90,
		0,70,
		E6,90,
		0,70,
		F6,500,
		0,0
};

const uint16_t bossEnter[]={
		G3,100,
		0,100,
		G3,100,
		F3,200,
		G3,900,
		0,200,
		A3,100,
		0,100,
		A3,100,
		G3,200,
		A3,900,
		AS3,100,
		0,5,
		AS3,100,
		0,100,
		AS3,100,
		0,400,	//insert short drumsolo here
		CS4,100,
		0,5,
		CS4,100,
		0,100,
		CS4,100,
		0,400,	//insert short drumsolo here
		D4,100,
		0,100,
		C4,200,
		AS3,100,
		C4,100,
		CS4,100,
		D4,50,
		DS4,50,
		D4,50,
		DS4,50,
		D4,50,
		DS4,50,
		D4,600,
		0,0
};

const uint16_t ffVictoryTheme[]={
		/*C2,50,	//The extra stuff in the beginning. Just doesn't sound good
		E2,50,
		G2,50,
		C3,50,
		E3,50,
		G3,50,
		C4,50,
		E4,50,
		G4,50,*/
		C4,70,
		//End of extra 0,5,
		0,80,
		C4,70,
		0,80,
		C4,70,
		0,80,
		C4,420,
		GS3,420,
		AS3,420,
		C4,210,
		0,110,
		AS3,100,
		C4,1230,
		0,0
};

const uint16_t marioFlagPole[]={
		E2,130,
		G2,130,
		C3,130,	//C3
		E3,130,
		G3,130,
		C4,300,
		0,20,
		G3,400,
		0,100, //Skall kortas
		DS2,130,
		GS2,130,
		C3,130,	//C3
		DS3,130,
		GS3,130,
		C4,300,
		0,20,
		GS3,400,
		0,100, //Kan ocksŒ kortas
		F2,130,
		AS2,130,
		D3,130,
		F3,130,
		AS3,130,
		D4,400,
		0,20,
		D4,400,
		0,20,
		C5,600,	//borde sŠnkas, C5
		0,0
};

//---------End of melodies------------//


void buzzerInit()
{
	GPIO_InitTypeDef GPIOInitStruct;
	TIM_TimeBaseInitTypeDef TIMBaseInitStruct;
	TIM_OCInitTypeDef TIMOCInitStruct;
	NVIC_InitTypeDef NVICInitStruct;

	//Setup clocks
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM16 | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO,ENABLE);

	//Init Pin (PB8, TIM16_CH1)
	GPIOInitStruct.GPIO_Pin = GPIO_Pin_8;
	GPIOInitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIOInitStruct.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_Init(GPIOB,&GPIOInitStruct);

	//Init timer
	TIMBaseInitStruct.TIM_ClockDivision = 0;
	TIMBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIMBaseInitStruct.TIM_Period = 65535;	//Frequency set to max by default
	TIMBaseInitStruct.TIM_RepetitionCounter = 0;
	TIMBaseInitStruct.TIM_Prescaler = SystemCoreClock/BUZZER_TIMER_FREQ - 1;
	TIM_TimeBaseInit(TIM16,&TIMBaseInitStruct);

	//Init OC
	TIMOCInitStruct.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIMOCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
	TIMOCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
	TIMOCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIMOCInitStruct.TIM_Pulse = 0;	//Volume is 0 to start
	TIM_OC1Init(TIM16,&TIMOCInitStruct);

	//Init interrupt and repetition counter
	TIM_Cmd(TIM16,ENABLE);


}

/*
 * Plays a tone on the buzzer with a length, volume and duration
 * beep is a pointer to a buzzerBeepType containing 3 parameters:
 * freq is the frequency of the beep in Hz. Can be up to 65535 (but you won't hear that ;) )
 * vol is the volume, max 255.
 * duration is the duration of the tone in ms
 */
void buzzerPlayBeep(buzzerBeepType* beep)
{
	TIM_CtrlPWMOutputs(TIM16,ENABLE);
	//Set frequency
	unsigned short period;
	period = BUZZER_TIMER_FREQ/(beep->freq);
	TIM_SetAutoreload(TIM16,period);

	//Calculate duty cycle
	//vol can be at most 255 and divided by 512, the duty cycle will be 50%, which will be max volume
	TIM16->CCR1 = period-(period*beep->vol)/512;
	delay_ms(beep->duration);
	TIM_CtrlPWMOutputs(TIM16,DISABLE);
}

/*
 * Plays a series of beeps in busy wait mode
 * Input is a pointer to a series of beeps
 * samples is the number of beeps in the pointer
 */
void buzzerPlayMelodyBusyWait(buzzerBeepType buffer[], unsigned long samples)
{
	for(unsigned long i=0; i<samples;i++)
	{
		buzzerPlayBeep(&buffer[i]);
	}
}

/*
 * Plays a melody from memory. For available melodies, see buzzer.h
 */
void buzzerPlayMelodyFromMem(unsigned char number)
{
	uint16_t* melodyPtr;
	buzzerBeepType tmpMelodyBeep[40];
	switch (number)
	{
	case MusicItemCatch:
		melodyPtr=(uint16_t*)itemCatch;
		break;
	case MusicSecretDiscovered:
		melodyPtr=(uint16_t*)secretDiscovered;
		break;
	case MusicMm4Victory:
		melodyPtr=(uint16_t*)mm4Victory;
		break;
	case MusicOneUp:
		melodyPtr=(uint16_t*)oneUp;
		break;
	case MusicBbossEnter:
		melodyPtr=(uint16_t*)bossEnter;
		break;
	case MusicFfVictoryTheme:
		melodyPtr=(uint16_t*)ffVictoryTheme;
		break;
	case MusicMarioFlagPole:
		melodyPtr=(uint16_t*)marioFlagPole;
		break;
	default:
		return;
	}
	unsigned long i=0;
	while(melodyPtr[2*i+1])	//If duration is 0, this is the last sample
	{
		tmpMelodyBeep[i].freq=melodyPtr[2*i];
		tmpMelodyBeep[i].duration=melodyPtr[2*i+1];
		tmpMelodyBeep[i].vol = 100;
		i++;
	}
	buzzerPlayMelodyBusyWait(tmpMelodyBeep,i);
}

