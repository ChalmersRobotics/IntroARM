/*
 * switches.h
 *
 *  Created on: 8 feb 2013
 *      Author: peter
 */

#ifndef SW_H_
#define SW_H_

#include "hardwareRev.h"

#if HARDWARE_REV==3
#define SW1				(!(GPIOB->IDR & GPIO_Pin_12))
#define SW2				(!(GPIOA->IDR & GPIO_Pin_8))
#elif HARDWARE_REV==2
#define SW1				(!(GPIOB->IDR & GPIO_Pin_3))
#define SW2				(!(GPIOA->IDR & GPIO_Pin_8))
#endif



//The last button is a reset-button and is not accesible by software

void switchesInit();

#endif /* SW_H_ */
