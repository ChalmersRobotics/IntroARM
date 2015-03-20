/*
 * motors.h
 *
 *  Created on: 18 jan 2013
 *      Author: peter
 */

#ifndef MOTORS_H_
#define MOTORS_H_

#include "stm32f10x_conf.h"
#include "stm32f10x.h"

signed char motorLeftDirection;
signed char motorRightDirection;
extern volatile signed short targetPulsePerSecondLeft;
extern volatile signed short targetPulsePerSecondRight;

extern volatile signed long errILeft;
extern volatile signed long errDLeft;
extern volatile signed short leftOutputPower;

void motorInit(uint32_t PWMfreq);
void motorLeftSetSpeed(float speed);
void motorRightSetSpeed(float speed);
void motorLeftRunDistance(float speed, unsigned long dist);
void motorRightRunDistance(float speed, unsigned long dist);
void motorsRunDistance(float speedLeft, unsigned long distLeft, float speedRight, unsigned long distRight);

void motorLeftSetRpm(signed short rpm);
void motorRegulatorDisable();
void motorRegulatorEnable();
void motorRegulatorSetFreq(unsigned short freq);
void motorSetRegP(float p);
void motorSetRegI(float i);
void motorSetRegD(float d);
float motorGetRegP();
float motorGetRegI();
float motorGetRegD();

void motorRegulator();

#define MOTOR_DISABLE()	(GPIOB->BRR = GPIO_Pin_5)
#define MOTOR_ENABLE()	(GPIOB->BSRR = GPIO_Pin_5)

//Used to set the direction of the motors. Should be 1.0 or -1.0
#define MC_RIGHT_DIR	1.0
#define MC_LEFT_DIR		1.0

#define MC_LEFT_DIR_INT		((signed char)MC_LEFT_DIR)
#define MC_RIGHT_DIR_INT	((signed char)MC_RIGHT_DIR)

//There are 12 teeth on the wheel. A "count" is generated twice per tooth
#define WHEEL_NOF_TEETH			12
#define WHEEL_COUNT_PER_TOOTH	2
#define WHEEL_NOF_COUNT			(WHEEL_NOF_TEETH*WHEEL_COUNT_PER_TOOTH)
//The circumference is 110mm (well, almost)
#define WHEEL_CIRC		110.0
//The wheel has traveled this distance per tachometer count (mm)
#define WHEEL_LENGTH_PER_COUNT	(WHEEL_CIRC/(WHEEL_NOF_TEETH*WHEEL_COUNT_PER_TOOTH))

#endif /* MOTORS_H_ */
