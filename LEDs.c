/*
 * LEDs.c
 *
 *  Created on: 17 jan 2013
 *      Author: peter
 */

#include "stm32f10x_conf.h"
#include "stm32f10x.h"
#include "LEDs.h"

//STARTLED at PB10 (TX3)

#include "main.h"

void ledInit(){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Mode= GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed= GPIO_Speed_50MHz;
#if HARDWARE_REV == 2
	GPIO_InitStruct.GPIO_Pin= GPIO_Pin_4 | GPIO_Pin_2 | GPIO_Pin_12 | GPIO_Pin_10;
#elif HARDWARE_REV == 3
	GPIO_InitStruct.GPIO_Pin= GPIO_Pin_4 | GPIO_Pin_2 | GPIO_Pin_3;// | GPIO_Pin_10;
#endif
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Pin= GPIO_Pin_15;
	GPIO_Init(GPIOA,&GPIO_InitStruct);

	LED_2_CLEAR();
	LED_3_CLEAR();
	LED_4_CLEAR();
	LED_5_CLEAR();

}

void ledUartInit(){

	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Mode= GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed= GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_Pin= GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_Init(GPIOA,&GPIO_InitStruct);

	LED_RX_CLEAR();
	LED_TX_CLEAR();
}


void ledShowNumber(unsigned char n)
{
	LED_ALL_CLEAR();
	if (1&(n>>3))	//16
	{
		LED_2_SET();
	}
	if(1&(n>>2))
	{
		LED_3_SET();
	}
	if(1&(n>>1))
	{
		LED_4_SET();
	}
	if(1&n)
	{
		LED_5_SET();
	}
}

