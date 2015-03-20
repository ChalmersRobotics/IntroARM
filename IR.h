/*
 * IR.h
 *
 *  Created on: 26 jan 2013
 *      Author: peter
 */

#ifndef IR_H_
#define IR_H_

#include "hardwareRev.h"

void irInit();
void irDisable();
void irEnable();
void irSetFreq(unsigned short freq);
void irSetSens(unsigned char sens);

#define IRMOT1				(!(GPIOC->IDR & GPIO_Pin_13))
#define IRMOT2				(!(GPIOC->IDR & GPIO_Pin_14))
#define IRMOT3				(!(GPIOC->IDR & GPIO_Pin_15))
#if HARDWARE_REV == 3
#define IRMOT4				(!(GPIOA->IDR & GPIO_Pin_3))
#elif HARDWARE_REV == 2
#define IRMOT4				(!(GPIOB->IDR & GPIO_Pin_9))
#endif

//Not actually used for IR-sensors
#define SENSE_REMOTE		(!(GPIOA->IDR & GPIO_Pin_12))

//Settings for the pulse frequency of the IR-LEDs
#define IR_FREQ_MIN 38000	//Minimum possible setting (maximum sensitivity)
#define IR_FREQ_MAX 50000	//Maximum possible setting (minimum sensitivity)
#define IR_FREQ 42000		//The default frequency

#define IR_SENS_MAX 16		//The number of steps used for sensitivity setting
#define IR_SENS_FREQ_STEP ((IR_FREQ_MAX - IR_FREQ_MIN)/IR_SENS_MAX)	//The step size for sensitivity setting

#define IR_FILTER_MAX 5	//The length of the low-pass filter used for the receivers

//Variables containing the filtered values of each receiver
extern volatile unsigned char ir1Active;
extern volatile unsigned char ir2Active;
extern volatile unsigned char ir3Active;
extern volatile unsigned char ir4Active;

#define IR_LEFT     ir4Active
#define IR_FR_LEFT  ir3Active
#define IR_FR_RIGHT ir2Active
#define IR_RIGHT    ir1Active

#if HARDWARE_REV == 3
#define IR_TIM TIM17
#define IR_TIM_IRQ TIM1_TRG_COM_TIM17_IRQHandler
#define IR_TIM_IRQn TIM1_TRG_COM_TIM17_IRQn
#define TIM_RCC RCC_APB2Periph_TIM17
#elif HARDWARE_REV == 2
#define IR_TIM TIM15
#define IR_TIM_IRQ TIM1_BRK_TIM15_IRQHandler
#define IR_TIM_IRQn TIM1_BRK_TIM15_IRQn
#define TIM_RCC RCC_APB2Periph_TIM15
#endif

#endif /* IR_H_ */
