/*
 *	sirc.h
 *
 *	Created on: 27 dec 2014
 *		Author: Sterna
 */
#ifndef SIRC_H_
#define SIRC_H_


#include "stm32f10x.h"
#include "stm32f10x_conf.h"


#define SIRC_ADR_TV1			1
#define SIRC_ADR_VCR1			2
#define SIRC_ADR_VCR2			3
#define SIRC_ADR_LASER_DISC		6
#define SIRC_ADR_SURROUND_SOUND	12
#define SIRC_ADR_TUNER			16
#define SIRC_ADR_CD				17
#define SIRC_ADR_EQ				18


#define SIRC_CMD_1				0
#define SIRC_CMD_2				1
#define SIRC_CMD_3				2
#define SIRC_CMD_4				3
#define SIRC_CMD_5				4
#define SIRC_CMD_6				5
#define SIRC_CMD_7				6
#define SIRC_CMD_8				7
#define SIRC_CMD_9				8
#define SIRC_CMD_0				9	//May also be 10


#define SIRC_CMD_CHPLUS			16
#define SIRC_CMD_CHMIN			17
#define SIRC_CMD_VPLUS			18
#define SIRC_CMD_VMIN			19
#define SIRC_CMD_MUTE			20
#define SIRC_CMD_POWER			21
#define SIRC_CMD_RESET			22
#define SIRC_CMD_AUDIOMODE		23
#define SIRC_CMD_CONTRASTPLUS	24
#define SIRC_CMD_CONTRASTMIN	25
#define SIRC_CMD_COLOURPLUS		26
#define SIRC_CMD_COLOURMIN		27
#define SIRC_CMD_BRIGHTPLUS		30
#define SIRC_CMD_BRIGHTMIN		31
#define SIRC_CMD_BALANCE_LEFT	38
#define SIRC_CMD_BALANCE_RIGHT	39
#define SIRC_CMD_STANDBY		47
#define SIRC_CMD_GUIDE			96


/*
000 1 button
001 2 button
002 3 button
003 4 button
004 5 button
005 6 button
006 7 button
007 8 button
008 9 button
009 10 button/0 button
011 Enter
016 channel up
017 channel down
018 volume up
019 volume down
020 Mute
021 Power
022 Reset TV
023 Audio Mode:Mono/SAP/Stereo
024 Picture up
025 Picture down
026 Color up
027 Color down
030 Brightness up
031 Brightness down
032 Hue up
033 Hue down
034 Sharpness up
035 Sharpness down
036 Select TV tuner
038 Balance Left
039 Balance Right
041 Surround on/off
042 Aux/Ant
047 Power off
048 Time display
054 Sleep Timer
058 Channel Display
059 Channel jump
064 Select Input Video1
065 Select Input Video2
066 Select Input Video3
074 Noise Reduction on/off
078 Cable/Broadcast
079 Notch Filter on/off
088 PIP channel up
089 PIP channel down
091 PIP on
092 Freeze screen
094 PIP position
095 PIP swap
096 Guide
097 Video setup
098 Audio setup
099 Exit setup
107 Auto Program
112 Treble up
113 Treble down
114 Bass up
115 Bass down
116 + key
117 - key
120 Add channel
121 Delete channel
125 Trinitone on/off
127 Displays a red RtestS on the screen -
See more at: http://www.picaxe.com/BASIC-Commands/Digital-InputOutput/infraout/#sthash.Kr1KbTSI.dpuf
 */

#endif /* SIRC_H_ */
