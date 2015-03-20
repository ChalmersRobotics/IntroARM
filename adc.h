/*
 * adc.h
 *
 *  Created on: 21 jan 2013
 *      Author: peter
 */

#ifndef ADC_H_
#define ADC_H_

#include "hardwareRev.h"

#define ADC1_DR_Address    ((uint32_t)0x4001244C)
#define ADC_NOF_CHAN		6


void adcInit(unsigned long freq);
void ADCTachoCalibrate();
signed long tachoGetCount(unsigned char side);
signed long tachoGetCountPerSecond(unsigned char side);

extern volatile unsigned short ADC_values[ADC_NOF_CHAN];

extern volatile signed long tachoLeftCount;
extern volatile signed long tachoRightCount;
extern volatile unsigned char tachoCalibration;
extern volatile signed long tachoLeftCountPerSecond;
extern volatile signed long tachoRightCountPerSecond;

//Used for debugging
extern volatile unsigned short tachoLeftFiltered;
extern volatile unsigned short tachoRightFiltered;
extern volatile unsigned long adcTime;
//End of used for debugging


#if HARDWARE_REV == 3
#define ADCEdgeRight		ADC_values[0]	//ADC_CH0
#define ADCBatteryLevel		ADC_values[1]	//ADC_CH1
#define ADCEdgeLeft			ADC_values[2]	//ADC_CH2
#define ADCTachometerRight	ADC_values[3]	//ADC_CH8
#define ADCTachometerLeft	ADC_values[4]	//ADC_CH9
#define ADCTempRaw			ADC_values[5]	//ADC_CH16

#elif HARDWARE_REV==2
#define ADCEdgeRight		ADC_values[0] //
#define ADCEdgeLeft			ADC_values[1] //
#define ADCBatteryLevel		ADC_values[2]
#define ADCTachometerLeft	ADC_values[3] //
#define ADCTachometerRight	ADC_values[4] //
#endif

//Constants for temperature sensor
#define ADC_V25		1.41	//V
#define TEMP_AVG_SLOPE 4.3	//mv/¡C
//Value of temp sensor in ¡C
#define ADCTempDegC			(((ADC_V25-ADCTempRaw*3.3/4096)/(TEMP_AVG_SLOPE/1000))+25)
//Returns the battery voltage in mV
#define ADCBattmV			(ADCBatteryLevel*3.2)
//The length of the tachometer filter in powers of 2 (filterLength=2^TACHO_FILTER_LEN)
#define TACHO_FILTER_SHIFT	3
#define TACHO_FILTER_LEN (1<<TACHO_FILTER_SHIFT)
//The threshold hysteresis divisor. The difference between the max and min value is divided by this value
//This value should not be smaller than 2.
#define TACHO_THRESH_HYST_DIV	4
#define TACHO_SPEED_TIMEOUT_MS		500

#ifndef LEFT
#define LEFT 0
#endif

#ifndef RIGHT
#define RIGHT 1
#endif

typedef struct
{
	signed long count;
	signed long countPerSecond;
	unsigned long filterTot;
	unsigned char filterIndex;
	unsigned short filterValues[TACHO_FILTER_LEN];
	unsigned short filteredVal;
	unsigned char state;
	unsigned long lastCountTime;
	signed char motorDir;
	unsigned short min;
	unsigned short max;
	unsigned short thresh;
} tachoStateType;

#endif /* ADC_H_ */
