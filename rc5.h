/*
 * rc5.h
 *
 *  Created on: 25 dec 2009
 *      Author: benjamin
 *
 * Copyright (c) 2009 Benjamin Vedder
 *
 * Parts in this driver are implemented using the information
 * available at:
 *
 * http://www.sbprojects.com/knowledge/ir/ir.htm
 *
 * This driver has been tested with several different remote controls and
 * everything seems to work so far.
 *
 * How to use it:
 * 1. Change the hardware-dependent macros below to fit your hardware.
 * 2. Initialize io-ports and the timer. That is, edit the ir_init() function.
 * 3. Make sure that the pin change interrupt and timing interrupt handlers below
 * are called from your interrupt handlers.
 *
 */

#ifndef RC5_H_
#define RC5_H_

#include <stdint.h>
#include "stm32f10x.h"
#include "stm32f10x_conf.h"

void EXTI15_10_IRQHandler();

/*
 * HW-dependent parameters. Change these macros if your hardware differs.
 */
// Macro to get the current IR counter value
#define RC5_CNT				(TIM6->CNT)

// The clock speed for the IR counter in Hz. Make sure that this is what IR_CNT is updated at.
// That is, either change this macro or the rate of your counter. Any value higher than 10 kHz
// should work.
#define RC5_COUNTER_CLOCK	1000000

// Macro to get the value of the IR pin to determine the edge
#define RC5_GET_EDGE()		(GPIOA->IDR & GPIO_Pin_12)
#define RC5_EDGE_LOW()		!RC5_GET_EDGE()
#define RC5_EDGE_HIGH()		RC5_GET_EDGE()
// ------------- END HW-parameters -------------------- //

/*
 * Some macros
 */
#ifndef 	_BV
#define	_BV(bit) 		(1<<(bit))
#endif

/*
 * Debugging
 */
#define RC5_DBG_EN	0

/*
 * Buffer size
 */
#define RC5_BUFFER_SIZE 20

// RC5 specific settings
#define RC5_DISABLE_REPEATS	1

typedef struct {
	uint8_t address_low;
	uint8_t command;
	uint8_t repeats;
} RC5DATA;

#if IR_DBG_EN
extern volatile unsigned int 	cnt_start2,
								cnt_toggle,
								cnt_addr,
								cnt_cmd;
#endif

/*
 * Functions
 */
void rc5Init();
signed char rc5HasNext();
void rc5GetNext(RC5DATA *data);
unsigned char rc5GetTepeats();
void rc5SetDataHandler(void(*func)(RC5DATA *data));
void rc5RemoveDataHandler();
void rc5DataHandler(RC5DATA *data);
void rc5Enable();
void rc5Disable();
/*
 * Interrupt handlers. Make sure that these are called from your
 * interrupt vectors.
 */
void rc5ExtiHandler();
void rc5TimingHandler();

/*
 * Protocol definitions
 */
#define ADDR_LEN	5
#define CMD_LEN		6

/*
 * Milliseconds for state machine to time out (must be larger than BIT_LEN+MAX_ERROR (about 2ms))
 */
#define IR_TIMEOUT		5

/*
 * Timings in us
 */
#define HALF_BIT_LEN	889UL
#define BIT_LEN		1778UL

/*
 * Timer constants
 */

// Counter values corresponding to bit lengths
#define HALF_BIT_CNT	(HALF_BIT_LEN * (RC5_COUNTER_CLOCK / 1000000L))
#define BIT_CNT			(BIT_LEN * (RC5_COUNTER_CLOCK / 1000000L))

// Maximum timing error in timer ticks
#define MAX_ERROR			((HALF_BIT_CNT)/8)


/*
 * RC5 protocol standard addresses and commands
 */
#define RC5_ADR_TV1				0x00
#define RC5_ADR_TV2				0x01
#define RC5_ADR_TELETEXT		0x02
#define RC5_ADR_VIDEO			0x03
#define RC5_ADR_LV1				0x04
#define RC5_ADR_VCR1			0x05
#define RC5_ADR_VCR2			0x06
#define RC5_ADR_EXPERIMENTAL	0x07	//Also used for start module (see just below)
#define RC5_ADR_STARTMODULE		0x07
#define RC5_ADR_SAT1			0x08
#define RC5_ADR_CAMERA			0x09
#define RC5_ADR_SAT2			0x0A
#define RC5_ADR_PROGRAMMING		0x0B	//Used for programming start module
#define RC5_ADR_CDV				0x0C
#define RC5_ADR_CAMCORDER		0x0D
#define RC5_ADR_PREAMP			0x10
#define RC5_ADR_TUNER			0x11
#define RC5_ADR_RECORDER1		0x12
#define RC5_ADR_PREAMP2			0x13
#define RC5_ADR_CDPLAYER		0x14
#define RC5_ADR_PHONO			0x15
#define RC5_ADR_SATA			0x16
#define RC5_ADR_RECORDER2		0x17
#define RC5_ADR_CDR				0x1A
#define RC5_ADR_LIGHTING		0x1D
#define RC5_ADR_LIGHTING2		0x1E
#define RC5_ADR_PHONE			0x1F

#define RC5_CMD_0				0x00
#define RC5_CMD_1				0x01
#define RC5_CMD_2				0x02
#define RC5_CMD_3				0x03
#define RC5_CMD_4				0x04
#define RC5_CMD_5				0x05
#define RC5_CMD_6				0x06
#define RC5_CMD_7				0x07
#define RC5_CMD_8				0x08
#define RC5_CMD_9				0x09
#define RC5_CMD_MIN				0x0A
#define RC5_CMD_STANDBY			0x0C
#define RC5_CMD_MUTE			0x0D
#define RC5_CMD_VPLUS			0x10
#define RC5_CMD_VMIN			0x11
#define RC5_CMD_BPLUS			0x12
#define RC5_CMD_BMIN			0x13
#define RC5_CMD_PPLUS			0x20
#define RC5_CMD_PMIN			0x21

#define RC5_CMD_SLEEP			0x2A
#define RC5_CMD_SEARCH_MODE	0x1A

#define RC5_CMD_FRWD			0x32
#define RC5_CMD_FFWD			0x34
#define RC5_CMD_PLAY			0x35
#define RC5_CMD_STOP			0x36
#define RC5_CMD_RECORDING		0x37



#endif /* RC5_H_ */





