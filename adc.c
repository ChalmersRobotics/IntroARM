/*
 * adc.c
 *
 *  Created on: 21 jan 2013
 *      Author: peter
 */

#include "stm32f10x_conf.h"
#include "stm32f10x.h"
#include "adc.h"
#include "motors.h"
#include "time.h"

volatile unsigned short ADC_values[ADC_NOF_CHAN];


volatile signed long tachoLeftCount=0;
volatile signed long tachoRightCount=0;
volatile signed long tachoLeftCountPerSecond=0;
volatile signed long tachoRightCountPerSecond=0;
volatile unsigned char tachoCalibration=0;
volatile unsigned char tachoIsCalibrated=0;
volatile unsigned long adcTime=0;

//Not needed outside the ADC interrupt
volatile unsigned short tachoLeftFiltered=0;
volatile unsigned short tachoRightFiltered=0;

#define TACHO_STATE_LOW 0
#define TACHO_STATE_FALLING 1
#define TACHO_STATE_RISING 2
#define TACHO_STATE_HIGH 3

static volatile tachoStateType tachoLeft;
static volatile tachoStateType tachoRight;

//Internal functions
static unsigned char tachoUpdateState(tachoStateType *tacho, unsigned short adcRawVal);


void adcInit(unsigned long freq)
{

	ADC_InitTypeDef ADC_InitStruct;
	GPIO_InitTypeDef GPIO_InitStruct;
	DMA_InitTypeDef DMA_InitStruct;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
	NVIC_InitTypeDef NVIC_InitStruct;
	TIM_OCInitTypeDef OC_InitStruct;

	//RCC

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	RCC_ADCCLKConfig(RCC_PCLK2_Div2);

	//Init GPIO
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AIN;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_0 | GPIO_Pin_1 |GPIO_Pin_2;
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AIN;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_Init(GPIOB, &GPIO_InitStruct);

	//NVIC Init
	//ADC interrupt
	NVIC_InitStruct.NVIC_IRQChannel=ADC1_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd=ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority=1;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority=2;
	NVIC_Init(&NVIC_InitStruct);

	//DMA Init
	DMA_DeInit(DMA1_Channel1);
	DMA_InitStruct.DMA_PeripheralBaseAddr = ADC1_DR_Address;
	DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t)&ADC_values;
	DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStruct.DMA_BufferSize = ADC_NOF_CHAN;
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStruct.DMA_Priority = DMA_Priority_High;
	DMA_InitStruct.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStruct);

	// Enable DMA1 channel1
	DMA_Cmd(DMA1_Channel1, ENABLE);

	//Setup timer2
	TIM_TimeBaseStruct.TIM_Prescaler = 0;
	TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStruct.TIM_Period = SystemCoreClock/freq;
	TIM_TimeBaseStruct.TIM_ClockDivision = 0;
	TIM_TimeBaseStruct.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStruct);

	//Init output compare, since ADC trigger cannot be done without it. This is not mapped to any output pin
	OC_InitStruct.TIM_OCMode = TIM_OCMode_PWM1;
	OC_InitStruct.TIM_OutputState = TIM_OutputState_Enable;
	OC_InitStruct.TIM_Pulse=SystemCoreClock/(2*freq);
	OC_InitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC2Init(TIM2,&OC_InitStruct);



	//Init ADC
	ADC_DeInit(ADC1);
	ADC_InitStruct.ADC_ContinuousConvMode=DISABLE;
	ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStruct.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_CC2;
	ADC_InitStruct.ADC_NbrOfChannel = ADC_NOF_CHAN;
	ADC_InitStruct.ADC_ScanConvMode = ENABLE;
	ADC_InitStruct.ADC_Mode = ADC_Mode_Independent;
	ADC_Init(ADC1,&ADC_InitStruct);

	//Init channel 0 (EdgeRight)
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0,1,ADC_SampleTime_28Cycles5);
	//Init channel 1 (Battery)
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1,2,ADC_SampleTime_28Cycles5);
	//Init channel 2 (EdgeLeft)
	ADC_RegularChannelConfig(ADC1, ADC_Channel_2,3,ADC_SampleTime_28Cycles5);
	//Init channel 8 (TachoRight)
	ADC_RegularChannelConfig(ADC1, ADC_Channel_8,4,ADC_SampleTime_28Cycles5);
	//Init channel 9 (TachoLeft)
	ADC_RegularChannelConfig(ADC1, ADC_Channel_9,5,ADC_SampleTime_28Cycles5);
	//Init channel 16 (TempSens)
	ADC_RegularChannelConfig(ADC1, ADC_Channel_TempSensor,6,ADC_SampleTime_28Cycles5);


	//Enable external trigger
	ADC_ExternalTrigConvCmd(ADC1, ENABLE);
	//Enable ADC discontinuous mode
	ADC_DiscModeCmd(ADC1,ENABLE);
	//Configure the number of channels to scan (again...)
	ADC_DiscModeChannelCountConfig(ADC1,ADC_NOF_CHAN);
	//Enable ADC interrupt at end of conversion (that is when all channels have been converted)
	ADC_ITConfig(ADC1,ADC_IT_EOC, ENABLE);
	//Enable ADC to send requests to DMA
	ADC_DMACmd(ADC1, ENABLE);

	//Wake up temp sensor
	ADC_TempSensorVrefintCmd(ENABLE);
	//Start ADC
	ADC_Cmd(ADC1, ENABLE);

	// Enable ADC1 reset calibration register
	ADC_ResetCalibration(ADC1);
	// Check the end of ADC1 reset calibration register
	while(ADC_GetResetCalibrationStatus(ADC1));

	// Start ADC1 calibration
	ADC_StartCalibration(ADC1);
	// Check the end of ADC1 calibration
	while(ADC_GetCalibrationStatus(ADC1));

	//TIM2 counter enable
	TIM_Cmd(TIM2, ENABLE);

}

/*
 * Returns the number of counts (signed long) for the specified tachometer
 * Parameter may LEFT or RIGHT
 */
signed long tachoGetCount(unsigned char side)
{
	if (side==LEFT)
	{
		return tachoLeft.count;
	}
	else if (side == RIGHT)
	{
		return tachoRight.count;
	}
	else
	{
		return 0;
	}
}

/*
 * Returns the speed (counts per second) (signed long) for the specified tachometer
 * Parameter may LEFT or RIGHT
 */
signed long tachoGetCountPerSecond(unsigned char side)
{
	if (side==LEFT)
	{
		return tachoLeft.countPerSecond;
	}
	else if (side == RIGHT)
	{
		return tachoRight.countPerSecond;
	}
	else
	{
		return 0;
	}
}


#define MOTOR_REG_TIMEOUT_MS	200

void ADC1_IRQHandler()
{
	//Interrupt bit is cleared by the DMA.
	//========Tachometer block========//
//	static unsigned long leftfiltertot=0;
//	static unsigned char leftfilterindex=0;
//	static unsigned short leftvalues[tacho_filter_len];
//	static unsigned char leftstate=tacho_state_low;
//	static unsigned long leftlastcounttime=0;
//	static unsigned short leftmin=6000;
//	static unsigned short leftmax=0;
//	static unsigned short leftthresh=0;
//	static unsigned long leftregtime=0;
//	static signed char leftlastmotordir=1;
//
//	static unsigned long rightfiltertot=0;
//	static unsigned char rightfilterindex=0;
//	static unsigned short rightvalues[tacho_filter_len];
//	static unsigned short rightmin=6000;
//	static unsigned short rightmax=0;
//	static unsigned short rightthresh=0;
//	static unsigned char rightstate=tacho_state_low;
//	static unsigned long rightlastcounttime=0;
//	static unsigned long rightregtime=0;

	//Used to measure timings for debugging purposes
//	static unsigned long startTime=0;
//	adcTime=microSeconds()-startTime;
//	startTime=microSeconds();

	//Update tachometer states
	tachoUpdateState(&tachoLeft,ADCTachometerLeft);
	tachoUpdateState(&tachoRight,ADCTachometerRight);

	//This whole block should not be needed anymore

//	//-------Left tachometer-----//
//	//Add the new value
//	leftFilterTot = leftFilterTot-leftValues[leftFilterIndex]+ADCTachometerLeft;
//	//Calculate the new filtered value
//	tachoLeftFiltered=leftFilterTot>>TACHO_FILTER_SHIFT;
//	//Update values
//	leftValues[leftFilterIndex++] = ADCTachometerLeft;
//	if (leftFilterIndex>=TACHO_FILTER_LEN)
//	{
//		leftFilterIndex=0;
//	}
//	//Perform tachometer calibration
//	if(tachoCalibration)
//	{
//		if(tachoLeftFiltered>leftMax)
//		{
//			leftMax=tachoLeftFiltered;
//		}
//		if(tachoLeftFiltered<leftMin)
//		{
//			leftMin=tachoLeftFiltered;
//		}
//		leftThresh=((leftMax-leftMin)/TACHO_THRESH_HYST_DIV);
//	}
//	//Only allow count when tachometer is calibrated
//	if(tachoIsCalibrated)
//	{
//		//Check against thresholds and update counter (only allow speed update if the direction is the same as last time todo: TEST)
//		if(leftState == TACHO_STATE_LOW && tachoLeftFiltered>(leftMax-leftThresh))
//		{
//			unsigned long timeDiffL = systemTime-leftLastCountTime;
//			//Calculate speed (2000 is since there are two counts for every update of speed)
//			tachoLeftCountPerSecond=motorLeftDirection*(2000/(timeDiffL));
//			leftLastCountTime=systemTime;
//			tachoLeftCount+=motorLeftDirection;
//			leftState=TACHO_STATE_HIGH;
//
//			//Handle when the regulator
//			motorRegulator();
//			//Assume that we will get the next value dependent on the speed. Also a
//			//leftRegTime=systemTime+timeDiffL+timeDiffL/4;
//			leftRegTime=systemTime+1200/(targetPulsePerSecondLeft);
//		}
//		else if(leftState==TACHO_STATE_HIGH && tachoLeftFiltered<(leftMin+leftThresh))
//		{
//			tachoLeftCount+=motorLeftDirection;
//			leftState=TACHO_STATE_LOW;
//		}
//		else if(systemTime>(leftLastCountTime+TACHO_SPEED_TIMEOUT_MS))
//		{
//			//If timeout, assume that the speed is 0
//			tachoLeftCountPerSecond=0;
//		}
//		//Timeout for motor regulator
//		if(systemTime>leftRegTime)
//		{
//			motorRegulator();
////			leftRegTime=systemTime+(systemTime-leftLastCountTime)+(systemTime-leftLastCountTime)/4;
//			//leftRegTime=systemTime+MOTOR_REG_TIMEOUT_MS;
//			leftRegTime=systemTime+1200/(targetPulsePerSecondLeft);
//		}
//	}
//	//------End of Left tachometer----//
//
//	//-----Right tachometer------/
//	//Add the new value
//	rightFilterTot = rightFilterTot-rightValues[rightFilterIndex]+ADCTachometerRight;
//	//Calculate the new filtered value
//	tachoRightFiltered=rightFilterTot>>TACHO_FILTER_SHIFT;
//	//Update values
//	rightValues[rightFilterIndex++] = ADCTachometerRight;
//	if (rightFilterIndex>=TACHO_FILTER_LEN)
//	{
//		rightFilterIndex=0;
//	}
//	//Perform tachometer calibration
//	if(tachoCalibration)
//	{
//		if(tachoRightFiltered>rightMax)
//		{
//			rightMax=tachoRightFiltered;
//		}
//		if(tachoRightFiltered<rightMin)
//		{
//			rightMin=tachoRightFiltered;
//		}
//		rightThresh=((rightMax-rightMin)/TACHO_THRESH_HYST_DIV);
//	}
//	//Only allow count when tachometer is calibrated
//	if(tachoIsCalibrated)
//	{
//		//Check against thresholds and update counter
//		if(rightState == TACHO_STATE_LOW && tachoRightFiltered>(rightMax-rightThresh))
//		{
//			//Calculate speed (2000 is since there are two counts for every update of speed)
//			tachoRightCountPerSecond=motorRightDirection*(2000/(systemTime-rightLastCountTime));
//			rightLastCountTime=systemTime;
//			tachoRightCount+=motorRightDirection;
//			rightState=TACHO_STATE_HIGH;
//			//motorRegulator();
////			rightRegTime=systemTime+MOTOR_REG_TIMEOUT_MS;
//		}
//		else if(rightState==TACHO_STATE_HIGH && tachoRightFiltered<(rightMin+rightThresh))
//		{
//			tachoRightCount+=motorRightDirection;
//			rightState=TACHO_STATE_LOW;
//		}
//		else if(systemTime>(rightLastCountTime+TACHO_SPEED_TIMEOUT_MS))
//		{
//			//If timeout, assume that the speed is 0
//			tachoRightCountPerSecond=0;
//		}
//		if(systemTime>rightRegTime)
//		{
//			//motorRegulator();
////			rightRegTime=systemTime+MOTOR_REG_TIMEOUT_MS;
//		}
//	}
	//-----End of Right tachometer------/
	//========End of Tachometer block========//

}

void tachoUpdateMotorDir(unsigned char side, signed char newDir)
{
	if(side==LEFT)
	{
		tachoLeft.motorDir=newDir;
	}
	else if(side == RIGHT)
	{
		tachoRight.motorDir=newDir;
	}
}

static unsigned char tachoUpdateState(tachoStateType *tacho, unsigned short adcRawVal)
{
	unsigned char shouldRunReg=0;
	//Calculate the new filtered value
	tacho->filterTot=tacho->filterTot-tacho->filterValues[tacho->filterIndex]+adcRawVal;
	tacho->filteredVal=tacho->filterTot>>TACHO_FILTER_SHIFT;
	//Update values
	tacho->filterValues[tacho->filterIndex++]=adcRawVal;
	if(tacho->filterIndex>=TACHO_FILTER_LEN)
	{
		tacho->filterIndex=0;
	}
	//Perform tachometer calibration
	if(tachoCalibration)
	{
		if(tacho->filteredVal>tacho->max)
		{
			tacho->max=tacho->filteredVal;
		}
		if(tacho->filteredVal>tacho->min)
		{
			tacho->min=tacho->filteredVal;
		}
		tacho->thresh=(tacho->max - tacho->min)/TACHO_THRESH_HYST_DIV;
	}
	//Only allow count when tachometer is calibrated
	if(tachoIsCalibrated)
	{
		//Check against thresholds and update counter (only allow speed update if the direction is the same as last time todo: TEST)
		if(tacho->state == TACHO_STATE_LOW && tacho->filteredVal>(tacho->max-tacho->thresh))
		{
			unsigned long timeDiff = systemTime-tacho->lastCountTime;
			//Calculate speed (2000 is since there are two counts for every update of speed)
			tacho->countPerSecond = tacho->motorDir*(2000/timeDiff);
			tacho->lastCountTime=systemTime;
			tacho->count+=tacho->motorDir;
			tacho->state=TACHO_STATE_HIGH;

			//Perhaps run motor regulator and handle that
//			//Handle when the regulator
			shouldRunReg=1;
//			motorRegulator();
//			//Assume that we will get the next value dependent on the speed. Also a
//			//leftRegTime=systemTime+timeDiffL+timeDiffL/4;
//			leftRegTime=systemTime+1200/(targetPulsePerSecondLeft);

		}
		else if(tacho->state==TACHO_STATE_HIGH && tacho->filteredVal<(tacho->min+tacho->thresh))
		{
			tacho->count+=tacho->motorDir;
			tacho->state=TACHO_STATE_LOW;
		}
		else if(systemTime>(tacho->lastCountTime+TACHO_SPEED_TIMEOUT_MS))
		{
			//If timeout, assume that the speed is 0
			tacho->countPerSecond=0;
		}
		//Timeout for motor regulator
//		if(systemTime>leftRegTime)
//		{
//			motorRegulator();
//			//			leftRegTime=systemTime+(systemTime-leftLastCountTime)+(systemTime-leftLastCountTime)/4;
//			//leftRegTime=systemTime+MOTOR_REG_TIMEOUT_MS;
//			leftRegTime=systemTime+1200/(targetPulsePerSecondLeft);
//		}
	}
	return shouldRunReg;
}

/*
 * Run the motors a little to calibrate the max/min values of the tachometers
 */
void ADCTachoCalibrate()
{
	motorRegulatorDisable();
	motorLeftSetSpeed(0.2);
	motorRightSetSpeed(0.2);
	tachoCalibration=1;
	delay_ms(2000);
	tachoCalibration=0;
	tachoIsCalibrated=1;
	motorLeftSetSpeed(0);
	motorRightSetSpeed(0);
	//Todo: Add a way to calibrate the distances of high and low counts
}
