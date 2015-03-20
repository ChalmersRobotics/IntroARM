/*
 * SW.c
 *
 *  Created on: 8 feb 2013
 *      Author: peter
 */
#include "stm32f10x_conf.h"
#include "stm32f10x.h"
#include "switches.h"

void switchesInit()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_IPU;
	GPIO_InitStruct.GPIO_Speed= GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_Pin= GPIO_Pin_8;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
#if HARDWARE_REV==3
	GPIO_InitStruct.GPIO_Pin= GPIO_Pin_12;
	GPIO_Init(GPIOB, &GPIO_InitStruct);

#elif HARDWARE_REV==2
	GPIO_InitStruct.GPIO_Pin= GPIO_Pin_3;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
#endif

}
