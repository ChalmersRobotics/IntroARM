/*
 * motors.c
 *
 *  Created on: 18 jan 2013
 *      Author: peter
 */


#include "motors.h"
#include "math.h"
#include "adc.h"
#include "time.h"
#include "flash.h"

//Debug
#include "leds.h"

signed char motorRightDirection=1;
signed char motorLeftDirection=1;

volatile unsigned char motorRegulationActive=0;
volatile unsigned short motorRegulationPeriod=100;
volatile unsigned short motorRegulationFreq=100;
volatile signed short targetPulsePerSecondLeft=0;
volatile signed short targetPulsePerSecondRight=0;

#ifndef LEFT
#define LEFT 0
#endif

#ifndef RIGHT
#define RIGHT 1
#endif

static uint32_t selectedPWMperiod=0;


//PID-params
static float motorRegP=0.005;
static float motorRegI=0.001;
static float motorRegD=0.001;
//iMax is scaled so that maximum
unsigned short iMax=0;

void motorInit(uint32_t PWMfreqency){

	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_Init_Struct;

	NVIC_Init_Struct.NVIC_IRQChannel=TIM1_UP_TIM16_IRQn;
	NVIC_Init_Struct.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init_Struct.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_Init_Struct.NVIC_IRQChannelSubPriority = 2;
	NVIC_Init(&NVIC_Init_Struct);


	//RCC Configuration
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	// Init MotorSleep Pin
	GPIO_InitStructure.GPIO_Pin= GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed= GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// GPIOA Configuration: Channel 4 as alternate function push-pull
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// GPIOB Configuration: Channel 1N, 2N and 3N as alternate function push-pull
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//Calculate the top value for the PWM counter, note that it is limited to 16 bits in hardware
	selectedPWMperiod = (SystemCoreClock / PWMfreqency);

	/* Time Base configuration */
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = selectedPWMperiod;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

	/* Channel 1, 2, 3 and 4 Configuration in PWM mode */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Set;

	//TIM1->BDTR &= ~(TIM_BDTR_MOE | TIM_BDTR_OSSR | TIM_BDTR_OSSI);


	//Channel 1 setup
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

	//Channel 2 setup
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);

	//Channel 3 setup
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);

	//Channel 4 setup
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);

	// TIM1 counter enable //
	TIM_Cmd(TIM1, ENABLE);

	// TIM1 Main Output Enable //
	TIM_CtrlPWMOutputs(TIM1, ENABLE);

	//Set the compare match registers for each channel to 0 to start the PWM at idle
	TIM1->CCR1=0;
	TIM1->CCR2=0;
	TIM1->CCR3=0;
	TIM1->CCR4=0;

	//Init motor reg params from flash
	flashReadParam(&motorRegP,FLASH_TYPE_FLOAT,FLASH_PARAM_KP);
	flashReadParam(&motorRegI,FLASH_TYPE_FLOAT,FLASH_PARAM_KI);
	flashReadParam(&motorRegD,FLASH_TYPE_FLOAT,FLASH_PARAM_KD);

	motorRegulationActive=0;
	if(motorRegI)
	{
		iMax=(unsigned short)(1.0/motorRegI);
	}



	//Enable motor controller
	MOTOR_ENABLE();
}


void motorLeftSetSpeed(float speed)
{
	speed = speed*MC_LEFT_DIR;
	if(speed>1){
		speed=1;
	}
	if(speed<-1){
		speed=-1;
	}
	//Reversed
	if(speed<0)
	{
		//selectedPWMperiod=750
		TIM1->CCR1=0;
		TIM1->CCR2=(unsigned short)(fabs(speed)*(float)selectedPWMperiod);
		motorLeftDirection=-1;
	}
	else
	{
		TIM1->CCR1=(unsigned short)(speed*(float)selectedPWMperiod);
		TIM1->CCR2=0;
		motorLeftDirection=1;
	}
	motorLeftDirection=(signed char)(motorLeftDirection*MC_LEFT_DIR);
}

void motorRightSetSpeed(float speed)
{
	speed = speed*MC_RIGHT_DIR;
	if(speed>1)
	{
		speed=1;
	}
	if(speed<-1)
	{
		speed=-1;
	}
	if(speed<0)
	{
		TIM1->CCR4=0;
		TIM1->CCR3=(unsigned short)(fabs(speed)*(float)selectedPWMperiod);
		motorRightDirection=-1;
	}
	else
	{
		TIM1->CCR4=(unsigned short)(speed*(float)selectedPWMperiod);
		TIM1->CCR3=0;
		motorRightDirection=1;
	}
	motorRightDirection=(signed char)(motorRightDirection*MC_RIGHT_DIR);
}

/*
 * Run the motors to achieve a certain distance on the wheels (in mm)
 * This function works in busy wait
 */
void motorsRunDistance(float speedLeft, unsigned long distLeft, float speedRight, unsigned long distRight)
{
	unsigned long targetCountsLeft=(unsigned long)(distLeft/WHEEL_LENGTH_PER_COUNT)+tachoLeftCount;
	unsigned long targetCountsRight=(unsigned long)(distRight/WHEEL_LENGTH_PER_COUNT)+tachoRightCount;
	motorLeftSetSpeed(speedLeft);
	motorRightSetSpeed(speedRight);
	while(tachoLeftCount<targetCountsLeft || tachoRightCount<targetCountsRight)
	{
		if(tachoLeftCount>=targetCountsLeft)
		{
			motorLeftSetSpeed(0);
		}
		if(tachoRightCount>=targetCountsRight)
		{
			motorRightSetSpeed(0);
		}
	}
	motorLeftSetSpeed(0);
	motorRightSetSpeed(0);
}

/*
 * Run the motor to achieve a certain distance on the wheels (in mm)
 * This function works in busy wait
 */
void motorLeftRunDistance(float speed, unsigned long dist)
{
	unsigned long targetCounts=(unsigned long)(dist/WHEEL_LENGTH_PER_COUNT)+tachoLeftCount;
	motorLeftSetSpeed(speed);
	while(tachoLeftCount<targetCounts){}
	motorLeftSetSpeed(0);
}

/*
 * Run the motor to achieve a certain distance on the wheels (in mm)
 * This function works in busy wait
 */
void motorRightRunDistance(float speed, unsigned long dist)
{
	unsigned long targetCounts=(unsigned long)(dist/WHEEL_LENGTH_PER_COUNT)+tachoLeftCount;
	motorRightSetSpeed(speed);
	while(tachoLeftCount<targetCounts){}
	motorRightSetSpeed(0);
}

/*
 * Set the frequency (in Hz) for the motor regulator
 * The regulator will run in the overflow interrupt of the motor, so it will run synchronously with the motors
 */
void motorRegulatorSetFreq(unsigned short freq)
{
	motorRegulationPeriod = SystemCoreClock/(selectedPWMperiod*freq);
	//Used so that we don't have to calculate this all the time
	motorRegulationFreq=freq;
}

void motorRegulatorEnable()
{
	motorRegulationActive=1;
	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);

}

void motorRegulatorDisable()
{
	motorRegulationActive=0;
	TIM_ITConfig(TIM1, TIM_IT_Update, DISABLE);
	errILeft=0;
}

/*
 * Set the speed of the motor in rpm. The speed is regulated by the tachometers
 * A frequency must be set on beforehand
 * Note that RPM must be larger than 60/WHEEL_NOF_COUNT
 */
void motorLeftSetRpm(signed short rpm)
{
	static signed short lastRpm=0;
	//Convert rpm to counts/second
	if(rpm == 0)
	{
		motorRegulatorDisable();
		motorLeftSetSpeed(0);
	}
	else
	{
		targetPulsePerSecondLeft=WHEEL_NOF_COUNT*rpm/60;
		motorRegulatorEnable();
		if(lastRpm!=rpm)
		{
			errILeft=0;
			motorRegulator();
			lastRpm=rpm;
		}
	}
}

void motorSetRegP(float p)
{
	motorRegP=p;

}

void motorSetRegI(float i)
{
	motorRegI=i;
	if(i)
	{
		iMax=(unsigned short)(1.0/i);
	}
}

void motorSetRegD(float d)
{
	motorRegD=d;
}

float motorGetRegP()
{
	return motorRegP;
}

float motorGetRegI()
{
	return motorRegI;
}

float motorGetRegD()
{
	return motorRegD;
}

volatile signed long errILeft=0;
volatile signed long errDLeft=0;
volatile signed short leftOutputPower=0;
/*
 * The motor regulator function. This is to be called in an interrupt
 * the ADC/Tacho-function handles all sensing and only communicates using tachoXCount and speed
 * Input is the current motor to be regulated
 */
void motorRegulator()
{
	float outputLeft;
	static signed short lastErrLeft=0;
	static signed short lastLeftCounts=0;

	float outputRight;
	unsigned long lastTime=0;
	//Check if regulator is enabled
	if(!motorRegulationActive)
	{
		return;
	}
	//Used to measure the time it takes
	lastTime = microSeconds();

	//Calculate errors
	signed short errLeft=targetPulsePerSecondLeft-tachoLeftCountPerSecond;
//	errDLeft = tachoLeftCountPerSecond-lastLeftCounts;
//	lastLeftCounts=tachoLeftCountPerSecond;
	errDLeft = errLeft-lastErrLeft;
	lastErrLeft=errLeft;
	errILeft+=errLeft;

	//Todo: Testa med ringbuffer-Ierr
	//Antiwindup for I
	if(errILeft>iMax)
	{
		errILeft=iMax;
	}
	else if(errILeft<(signed short)(-iMax))
	{
		errILeft=(signed short)(-iMax);
	}
	//Calculate output error
	outputLeft=(float)(errLeft * motorRegP + errILeft * motorRegI + errDLeft*motorRegD);
	//Check if the regulator wants to go in the wrong direction
	if((outputLeft*targetPulsePerSecondLeft)<=0)
	{
		//Todo: testa att alltid ge ett minsta output
		outputLeft=0;
	}
	leftOutputPower=(signed short)(100*outputLeft); //Used for debugging

	//Set output
	motorLeftSetSpeed(outputLeft);
	//motorRightSetSpeed(outputRight);

	//Debug
	adcTime = microSeconds() - lastTime;
	LED_2_TOGGLE();
}

/*
 * Used as TIM1 overflow interrupt mostly to trigger the motor regulator
 * WIll most likely not be used, since the regulator must be triggered on the sensor sampling
 */
void TIM1_UP_TIM16_IRQHandler()
{
	static unsigned short regulatorTrigger=0;

	TIM_ClearFlag(TIM1,TIM_FLAG_Update);
//	if(!regulatorTrigger--)
//	{
//		regulatorTrigger = motorRegulationPeriod;
//		motorRegulator();
//	}


//	if(currentMotor==LEFT)
//	{
//		//CH1N - PC13, CH2N - PC14, CH3N - PC15, CH4 - PA11
//
//		//Clear GPIO config for all ports
//		GPIOC->CRH &= ~(GPIO_CRH_CNF13 | GPIO_CRH_CNF14 | GPIO_CRH_CNF15);
//		GPIOA->CRH &= ~(GPIO_CRH_CNF11);
//		//Disable left motor outputs (CH1 and 2) (set as general output, open drain and set to open)
//		GPIOC->CRH |= GPIO_CRH_CNF13_0 | GPIO_CRH_CNF14_0;
//		GPIOC->BSRR = GPIO_Pin_12 | GPIO_Pin_13;
//
//		//Enable right motor (CH3 and 4) (set as AFIO, push pull)
//		GPIOC->CRH |= GPIO_CRH_CNF15_1;
//		GPIOA->CRH |= GPIO_CRH_CNF11_1;
//
////		//Disable channel 1 and 2 output
////		TIM_CCxCmd(TIM1,TIM_Channel_1,TIM_CCxN_Disable);
////		TIM_CCxCmd(TIM1,TIM_Channel_2,TIM_CCxN_Disable);
////		//Enable channel 3 and 4 output
////		TIM_CCxCmd(TIM1,TIM_Channel_3,TIM_CCxN_Enable);
////		TIM_CCxCmd(TIM1,TIM_Channel_4,TIM_CCx_Enable);
//		currentMotor=RIGHT;
//		LED_2_TOGGLE();
//	}
//	else if(currentMotor==RIGHT)
//	{
//
//		//Clear GPIO config for all ports
//		GPIOC->CRH &= ~(GPIO_CRH_CNF13 | GPIO_CRH_CNF14 | GPIO_CRH_CNF15);
//		GPIOA->CRH &= ~(GPIO_CRH_CNF11);
//
//		//Enable left motor outputs (CH1 and 2) (set as AFIO, push pull)
//		GPIOC->CRH |= GPIO_CRH_CNF13_1 | GPIO_CRH_CNF14_1;
//
//		//Disable right motor (CH3 and 4) (set as general output, open drain and set to open)
//		GPIOC->CRH |= GPIO_CRH_CNF15_1;
//		GPIOA->CRH |= GPIO_CRH_CNF11_1;
//		GPIOC->BSRR = GPIO_Pin_15;
//		GPIOA->BSRR = GPIO_Pin_11;
//
////		//Enable channel 1 and 2 output
////		TIM_CCxCmd(TIM1,TIM_Channel_1,TIM_CCxN_Enable);
////		TIM_CCxCmd(TIM1,TIM_Channel_2,TIM_CCxN_Enable);
////		//Disable channel 3 and 4 output
////		TIM_CCxCmd(TIM1,TIM_Channel_3,TIM_CCxN_Disable);
////		TIM_CCxCmd(TIM1,TIM_Channel_4,TIM_CCx_Disable);
//		currentMotor=LEFT;
//		LED_3_TOGGLE();
//	}
}
