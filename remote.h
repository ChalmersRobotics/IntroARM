/*
 *	remote.h
 *
 *	Created on: 27 dec 2014
 *		Author: Sterna
 */
#ifndef REMOTE_H_
#define REMOTE_H_

#include "stm32f10x.h"
#include "stm32f10x_conf.h"

#include "ir.h"
#include "time.h"
#include "sirc.h"
#include "rc5.h"
#include "switches.h"
#include "uart.h"
#include "leds.h"


#define REMOTE_SIRC_BASE_PERIOD	600

void remoteSendSirc(unsigned char adr, unsigned char cmd);
void remoteInit(unsigned short freq);
void remoteSendSircRepeat(unsigned char adr, unsigned char cmd, unsigned char repeats);

#endif /* REMOTE_H_ */
