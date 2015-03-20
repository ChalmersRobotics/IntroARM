/*
 * main.h
 *
 *  Created on: Nov 8, 2012
 *      Author: gurgalof
 */

#ifndef MAIN_H_
#define MAIN_H_

#include "rc5.h"
void timing_handler();
void Delay(volatile uint32_t nCount);
void getZRO(void);
void angle(float *angleX, float *angleY, uint32_t dtime);
void rc5DataHandler(RC5DATA *data);
void delay_ms(volatile unsigned long ms);
void showBattery();

#ifndef 	_BV
#define	_BV(bit) 		(1<<(bit))
#endif

#endif /* MAIN_H_ */
