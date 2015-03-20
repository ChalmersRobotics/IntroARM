/*
 *	buzzer.h
 *
 *	Created on: 22 feb 2015
 *		Author: Sterna
 */
#ifndef BUZZER_H_
#define BUZZER_H_

#include "stm32f10x_conf.h"
#include "time.h"
#include "tones.h"


/*
 * Contains a specification of a beep
 */
typedef struct
{
	unsigned short freq;
	unsigned char vol;
	unsigned long duration;
} buzzerBeepType;

void buzzerInit();
void buzzerPlayBeep(buzzerBeepType* beep);
void buzzerPlayMelodyBusyWait(buzzerBeepType buffer[], unsigned long samples);
void buzzerPlayMelodyFromMem(unsigned char number);

enum
{
	MusicItemCatch,
	MusicSecretDiscovered,
	MusicMm4Victory,
	MusicOneUp,
	MusicBbossEnter,
	MusicFfVictoryTheme,
	MusicMarioFlagPole,
	MusicNofMusic
};




#endif /* BUZZER_H_ */
