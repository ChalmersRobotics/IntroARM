/*
	Copyright 2009-2012 Benjamin Vedder	benjamin@vedder.se

	This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

/*
 * rc5.c
 *
 *  Created on: 26 dec 2009
 *      Author: benjamin and sternŒ
 *
 *      How to use:
 *      1. Change the definitions to fit your system (default using timer6 as time reference)
 *      2. If desired, set the RC5 callback function using rc5SetDataHandler. This is called from an interrupt, so keep it short.
 *      3. Call rc5Init to init the function.
 *
 *      User can switch on/off the function using rc5Enable/Disable (which really switches on/off the interrupt)
 */

#include "rc5.h"
#include "LEDs.h"


/*
 * Interrupt states
 */
#define	RC5_STARTBIT1	0
#define	RC5_STARTBIT2	1
#define	RC5_TOGGLEBIT	2
#define	RC5_ADDR		3
#define	RC5_CMD			4



/*
 * Variables
 */
volatile static int8_t state;
volatile static int8_t toggle;
volatile static uint8_t repeats;
volatile static uint8_t buffer_read, buffer_write;
volatile static uint8_t timeout;

/*
 * Buffer with received data
 */
volatile static RC5DATA buffer[RC5_BUFFER_SIZE];

/*
 * Data handler function
 */
static void(*rc5DataFunc)(RC5DATA *data) = 0;

/*
 * Debug
 */
#if IR_DBG_EN
volatile unsigned int 		cnt_start2,
								cnt_toggle,
								cnt_addr,
								cnt_cmd;
#endif


void rc5Init(void) {
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	uint16_t PrescalerValue = 0;
	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	//Init variables
	state = RC5_STARTBIT1;
	buffer_read = 0;
	buffer_write = 0;
	toggle = 0;
	timeout = 0;

#if IR_DBG_EN
	cnt_start2 = 0;
	cnt_toggle = 0;
	cnt_addr = 0;
	cnt_cmd = 0;
#endif

	// ------------- EXTI -------------- //
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO,ENABLE);
	// Configure PA3 pin as input floating
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource12);

	// Configure EXTI Line3
	EXTI_InitStructure.EXTI_Line = EXTI_Line12;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	// Enable and set EXTI Line3 Interrupt to rather high prio
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);



	// ------------- Timer6 ------------- //
	// Compute the prescaler value (timer shall run at 1MHz)
	// TIM6 clock enable
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
	PrescalerValue = (uint16_t) ((SystemCoreClock) / RC5_COUNTER_CLOCK - 1);

	NVIC_InitStructure.NVIC_IRQChannel = TIM6_DAC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = IR_TIMEOUT*1000;
	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);

	TIM_PrescalerConfig(TIM6, PrescalerValue, TIM_PSCReloadMode_Immediate);

	TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE);

	// TIM6 enable counter
	TIM_Cmd(TIM6, ENABLE);

	rc5Enable();
}

/*
 * Enable the RC5 interrupt (enable the function)
 */
void rc5Enable()
{
	NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/*
 * Disable the RC5 interrupt (disable the function)
 */
void rc5Disable()
{
	NVIC_DisableIRQ(EXTI15_10_IRQn);
}

/*
 * Sets a data handler function to be called whenever RC5 data has been successfully received
 * The function shall be on the form void handlerFunc(RC5Data *data){...} (a pointer to RC5DATA)
 * Note that this function is called from an interrupt, so keep it short!
 */
void rc5SetDataHandler(void(*func)(RC5DATA *data)) {
	rc5DataFunc = func;
}

void rc5RemoveDataHandler() {
	rc5DataFunc = 0;
}

//Timer overflow interrupt
void TIM6_DAC_IRQHandler()
{
	//Clear interrupt bit
	TIM_ClearITPendingBit(TIM6,TIM_IT_Update);
	state = RC5_STARTBIT1;
}

/*
 * Checks if there are any new messages available
 */
signed char rc5HasNext(void) {
	if (buffer_read == buffer_write)
	{
		return 0;
	}
	else
	{
		return 1;
	}
}

/*
 * Fetches the next unread data from the RC5 data buffer.
 */
void rc5GetNext(RC5DATA *data) {

	while(buffer_read == buffer_write);

	data->address_low = buffer[buffer_read].address_low;
	data->command = buffer[buffer_read].command;

	if (buffer_read < RC5_BUFFER_SIZE - 1) {
		buffer_read++;
	}
	else
	{
		buffer_read = 0;
	}
}

unsigned char rc5GetTepeats(void) {
	return repeats;
}

/*
 * Interrupt handler for RC5. Call this from your pin interrupt handler
 */
void rc5ExtiHandler()
{
	static uint8_t index = 0, addr = 0, cmd = 0, toggle_last = 0;

	// Reset timeout
	timeout = 0;
	switch (state)
	{
		case RC5_STARTBIT1:
			if (RC5_EDGE_LOW()) {
				state = RC5_STARTBIT2;
				RC5_CNT = 0;
			} else {
				state = RC5_STARTBIT1;
			}
			break;

		case RC5_STARTBIT2:
	#if RC5_DBG_EN
			cnt_start2 = RC5_CNT;
	#endif

			if ((RC5_CNT - MAX_ERROR) < BIT_CNT && (RC5_CNT + MAX_ERROR) > BIT_CNT) {
				state = RC5_TOGGLEBIT;
				RC5_CNT = 0;

			} else if ((RC5_CNT - MAX_ERROR) < HALF_BIT_CNT && (RC5_CNT + MAX_ERROR) > HALF_BIT_CNT) {
				RC5_CNT = HALF_BIT_CNT;	// Synchronize..
				return;
			} else {
				state = RC5_STARTBIT1;
				return;
			}
			break;

		case RC5_TOGGLEBIT:

	#if RC5_DBG_EN
			cnt_toggle = RC5_CNT;
	#endif
			if ((RC5_CNT - MAX_ERROR) < BIT_CNT && (RC5_CNT + MAX_ERROR) > BIT_CNT)
			{
				toggle_last = toggle;

				if (RC5_EDGE_LOW()) {
					toggle = 1;
				} else {
					toggle = 0;
				}

				index = 0;
				addr = 0;
				cmd = 0;

				RC5_CNT = 0;
				state = RC5_ADDR;

			} else if ((RC5_CNT - MAX_ERROR) < HALF_BIT_CNT && (RC5_CNT + MAX_ERROR) > HALF_BIT_CNT) {
				RC5_CNT = HALF_BIT_CNT;	// Synchronize..
				return;
			} else {
				state = RC5_STARTBIT1;
				return;
			}
			break;

		case RC5_ADDR:
	#if IR_DBG_EN
			cnt_addr = IR_CNT;
	#endif

			if ((RC5_CNT - MAX_ERROR) < BIT_CNT && (RC5_CNT + MAX_ERROR) > BIT_CNT)
			{
				if (RC5_EDGE_LOW())
				{
					addr |= _BV(ADDR_LEN - index - 1);
				}
				else
				{
					//Here, nothing should happen, since the bit should be 0
				}

				index++;

				if (index == ADDR_LEN) {
					index = 0;
					state = RC5_CMD;
				}
				RC5_CNT = 0;
			} else if ((RC5_CNT - MAX_ERROR) < HALF_BIT_CNT && (RC5_CNT + MAX_ERROR) > HALF_BIT_CNT) {
				RC5_CNT = HALF_BIT_CNT;	// Synchronize..
				return;
			} else {
				state = RC5_STARTBIT1;
				return;
			}
			break;

		case RC5_CMD:
			if ((RC5_CNT - MAX_ERROR) < BIT_CNT && (RC5_CNT + MAX_ERROR) > BIT_CNT)
			{
#if IR_DBG_EN
		cnt_cmd = IR_CNT;
#endif
				if (RC5_EDGE_LOW())
				{
					cmd |= _BV(CMD_LEN - index - 1);
				} else
				{
					//Here, nothing should happen, since the bit should be 0
				}
				index++;
				if (index == CMD_LEN) {
					if (toggle == toggle_last && !RC5_DISABLE_REPEATS) {
						repeats++;
					}
					else
					{

						//If there is a callback function, call this
						if (rc5DataFunc) {
							static RC5DATA tmp_data;
							tmp_data.address_low = addr;
							tmp_data.command = cmd;
							rc5DataFunc(&tmp_data);
						}
						//If there is no callback function, put data into buffer
						else
						{
							buffer[buffer_write].address_low = addr;
							buffer[buffer_write].command = cmd;

							if (buffer_write < RC5_BUFFER_SIZE - 1) {
								buffer_write++;
							} else {
								buffer_write = 0;
							}
						}

						repeats = 0;
					}
					state = RC5_STARTBIT1;
				}
				RC5_CNT = 0;
			} else if ((RC5_CNT - MAX_ERROR) < HALF_BIT_CNT && (RC5_CNT + MAX_ERROR) > HALF_BIT_CNT) {
				RC5_CNT = HALF_BIT_CNT;	// Synchronize..
				return;
			} else {
				state = RC5_STARTBIT1;
				return;
			}
		break;
	}
}


/*
 * Call this function 1000 times/sec
 */
void rc5TimingHandler() {
	if (timeout >= IR_TIMEOUT) {
		state = RC5_STARTBIT1;
	}
	else
	{
		timeout++;
	}
}

/*
 * Interrupt handler for input pin of RC5-receiver. Switch this if pin is switched
 */
void EXTI15_10_IRQHandler()
{
	EXTI_ClearITPendingBit(EXTI_Line12);
	rc5ExtiHandler();
}
