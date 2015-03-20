/*
 * LEDs.h
 *
 *  Created on: 17 jan 2013
 *      Author: peter
 */

#ifndef LEDS_H_
#define LEDS_H_

#include "hardwareRev.h"

void ledInit();
void ledUartInit();
void ledShowNumber(unsigned char n);
/*
 * REV3:
 * LED2: B4
 * LED3: B3
 * LED4: A15
 * LED5: B2
 * LED6: A10 (RX1)
 * LED7: A9 (TX1)
 */
#if HARDWARE_REV==3
#define LED_3_CLEAR()		(GPIOB->BRR = GPIO_Pin_3)
#define LED_3_SET()			(GPIOB->BSRR = GPIO_Pin_3)
#define LED_3_TOGGLE()		(GPIOB->ODR^= GPIO_Pin_3)

#define LED_4_SET()			(GPIOA->BSRR = GPIO_Pin_15)
#define LED_4_CLEAR()		(GPIOA->BRR = GPIO_Pin_15)
#define LED_4_TOGGLE()		(GPIOA->ODR^= GPIO_Pin_15)

#define LED_2_CLEAR()		(GPIOB->BRR = GPIO_Pin_4)
#define LED_2_SET()			(GPIOB->BSRR = GPIO_Pin_4)
#define LED_2_TOGGLE()		(GPIOB->ODR ^=GPIO_Pin_4)

#elif HARDWARE_REV==2
#define LED_3_SET()			(GPIOA->BRR = GPIO_Pin_15)
#define LED_3_CLEAR()		(GPIOA->BSRR = GPIO_Pin_15)
#define LED_3_TOGGLE()		(GPIOA->ODR^= GPIO_Pin_15)

#define LED_4_CLEAR()		(GPIOB->BSRR = GPIO_Pin_12)
#define LED_4_SET()			(GPIOB->BRR = GPIO_Pin_12)
#define LED_4_TOGGLE()		(GPIOB->ODR^= GPIO_Pin_12)

#define LED_2_SET()			(GPIOB->BRR = GPIO_Pin_4)
#define LED_2_CLEAR()		(GPIOB->BSRR = GPIO_Pin_4)
#define LED_2_TOGGLE()		(GPIOB->ODR ^=GPIO_Pin_4)
#endif

#define LED_5_CLEAR()		(GPIOB->BRR = GPIO_Pin_2)
#define LED_5_SET()			(GPIOB->BSRR = GPIO_Pin_2)
#define LED_5_TOGGLE()		(GPIOB->ODR^= GPIO_Pin_2)

/*
#define LED_START_CLEAR()	(GPIOB->BRR = GPIO_Pin_10)
#define LED_START_SET()		(GPIOB->BSRR = GPIO_Pin_10)
#define LED_START_TOGGLE()	(GPIOB->ODR^= GPIO_Pin_10)
*/

#define LED_START_CLEAR()	LED_TX_CLEAR()
#define LED_START_SET()		LED_TX_SET()
#define LED_START_TOGGLE()	LED_TX_TOGGLE()

#define LED_TX_SET()		(GPIOA->BRR = GPIO_Pin_10)
#define LED_TX_CLEAR()		(GPIOA->BSRR = GPIO_Pin_10)
#define LED_TX_TOGGLE()		(GPIOA->ODR ^= GPIO_Pin_10)

#define LED_RX_SET()		(GPIOA->BRR = GPIO_Pin_9)
#define LED_RX_CLEAR()		(GPIOA->BSRR = GPIO_Pin_9)
#define LED_RX_TOGGLE()		(GPIOA->ODR ^= GPIO_Pin_9)

#define LED_6_SET()			LED_TX_SET()
#define LED_6_CLEAR()		LED_TX_CLEAR()
#define LED_6_TOGGLE()		LED_TX_TOGGLE()

#define LED_7_SET()			LED_RX_SET()
#define LED_7_CLEAR()		LED_RX_CLEAR()
#define LED_7_TOGGLE()		LED_RX_TOGGLE()

#define LED_ALL_CLEAR()		LED_2_CLEAR();LED_3_CLEAR();LED_4_CLEAR();LED_5_CLEAR();LED_6_CLEAR();LED_7_CLEAR()
#define LED_ALL_SET()		LED_2_SET();LED_3_SET();LED_4_SET();LED_5_SET();LED_6_SET();LED_7_SET()
#define LED_ALL_TOGGLE()	LED_2_TOGGLE();LED_3_TOGGLE();LED_4_TOGGLE();LED_5_TOGGLE();LED_6_TOGGLE();LED_7_TOGGLE()

#endif /* LEDS_H_ */
