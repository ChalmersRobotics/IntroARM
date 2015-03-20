/*
 *	remote.c
 *
 *	Created on: 27 dec 2014
 *		Author: Sterna
 *
 *	Use IntroARM as remote control
 *
 */

#include "remote.h"

//Internal functions
void delaySirc(unsigned char periods);
void irOff();
void irOn();

/*
 * SIRC runs at 40kHz
 * RC-5 runs at 38kHz
 */
void remoteInit(unsigned short freq)
{
	irInit();
	irSetFreq(freq);
	TIM_ITConfig(IR_TIM,TIM_IT_Update,DISABLE);
}

void irOn()
{
	TIM_CtrlPWMOutputs(IR_TIM,ENABLE);
}

void irOff()
{
	TIM_CtrlPWMOutputs(IR_TIM,DISABLE);
}

/*
 * Sends a message using Sonys SIRC-protocol using busy wait
 */
void remoteSendSirc(unsigned char adr, unsigned char cmd)
{
	//send start (4T high, 1T low)
	irOn();
	delaySirc(4);
	irOff();
	delaySirc(1);
	//Send cmd (lsb first)
	for (unsigned char i=0;i<=6;i++)
	{
		irOn();
		if(cmd&1)	//bit is one
		{
			delaySirc(2);
		}
		else	//Bit is zero
		{
			delaySirc(1);
		}
		irOff();
		delaySirc(1);
		cmd=cmd>>1;
	}
	//Send adress (lsb first)
	for (unsigned char i=0;i<=4;i++)
	{
		irOn();
		if(adr&1)	//bit is one
		{
			delaySirc(2);
		}
		else	//Bit is zero
		{
			delaySirc(1);
		}
		irOff();
		delaySirc(1);
		adr=adr>>1;
	}
	//You must wait 10ms in idle
	delay_ms(10);
}

void remoteSendSircRepeat(unsigned char adr, unsigned char cmd, unsigned char repeats)
{
	unsigned long repeatTime=0;
	while(repeats--)
	{
		if(systemTime>repeatTime)
		{
			repeatTime=systemTime+45;
			remoteSendSirc(adr,cmd);
		}
	}
}
//Used since delay_us() cannot handle delays larger than 1000us
void delaySirc(unsigned char periods)
{
	while(periods--)
	{
		delay_us(REMOTE_SIRC_BASE_PERIOD);
	}
}


void remoteRun()
{
	unsigned long repeatTime=0;
	unsigned long checkTime=0;
	unsigned char tempCmd=SIRC_CMD_POWER;
	while(1)
	{
		if(systemTime>checkTime && USART_GetFlagStatus(USART1,USART_FLAG_RXNE))
		{
			tempCmd=(unsigned char)(USART1->DR);
			LED_2_TOGGLE();
			remoteSendSircRepeat(SIRC_ADR_TV1,tempCmd,10);
		}
		while(SW2)
		{
			LED_3_TOGGLE();
			if(systemTime>repeatTime)
			{
				repeatTime=systemTime+45;
				remoteSendSirc(SIRC_ADR_TV1,tempCmd);
			}
		}
	}
}
