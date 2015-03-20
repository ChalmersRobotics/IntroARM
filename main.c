#include <stdio.h>
#include <string.h>
#include <math.h>
#include "stm32f10x_conf.h"
#include "stm32f10x.h"
#include "main.h"
#include "time.h"
#include "LEDs.h"
#include "motors.h"
#include "uart.h"
#include "adc.h"
#include "switches.h"
#include "xprintf.h"
#include "IR.h"
#include "sirc.h"
#include "remote.h"
#include "utils.h"
#include "flash.h"
#include "buzzer.h"
#include "mpu6050.h"


//Enums describing the state of startmodule
enum
{
	START_POWER,
	START_STARTED,
	START_STOPPED,
	START_PROGRAMMING
};

volatile unsigned char startAddress=0; //The command used to start the robot
volatile unsigned char startState=START_POWER;	//The current state of the module

// Private functions
void delay_ms(volatile unsigned long ms);
void runRemoteRobot();
void sendADCVals();
void sendADCTacho();
void showIrSensors();

//Needed due to some quirks of arm-gcc
void _init(void){

}

//Used for remote-controlled robot
#define ST_STOP 	0
#define ST_FWD 		1
#define ST_REV		2
#define ST_RIGHT	3
#define ST_LEFT		4
#define	ST_FWD_LEFT	5
#define	ST_FWD_RIGHT	6
#define ST_TIMEOUT	200

volatile float rc5PowerLeft=0.4;
volatile float rc5PowerRight=0.4;
volatile unsigned char driveState=ST_STOP;
volatile unsigned long abortTime=0;


int main(void)
{
	//Configure the interrupt priority (use 2 bit preempetion and 2 bits for subprio)
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	flashInit();
	timeInit();
	ledInit();

	//ledUartInit();
	motorInit(30000);
	motorRegulatorSetFreq(100);
	switchesInit();
	uart1Init(115200);
	//delay_ms(2);
	rc5SetDataHandler(rc5DataHandler);
	rc5Init();
	adcInit(10000);
	//irDisable();
	//remoteInit(40000);
	mpu6050Init();
	showBattery();

	irInit();
	irEnable();
	//runRemoteRobot();
	unsigned char currentSens=10;
	irSetSens(currentSens);
	buzzerInit();
	unsigned long sendTime=0;
	buzzerBeepType beep;
	beep.duration=300;
	beep.freq = 200;
	beep.vol = 10;
#define DEL 25

	while(1)
	{
		if(SW1)
		{

			/*//A4,160,
			 * AS4,210,
			B4,210,
			C5,800,
			0,0
			 */
			/*beep.vol = 150;
			beep.freq = G5;
			beep.duration=180;
			buzzerPlayBeep(&beep);
			beep.freq = FS5;
			beep.duration=150;
			buzzerPlayBeep(&beep);
			beep.freq = DS5;
			beep.duration=180;
			buzzerPlayBeep(&beep);
			beep.freq = A4;
			beep.duration=180;
			buzzerPlayBeep(&beep);
			beep.freq = GS4;
			beep.duration=190;
			buzzerPlayBeep(&beep);
			beep.freq = E5;
			beep.duration=190;
			buzzerPlayBeep(&beep);
			beep.freq = GS5;
			beep.duration=210;
			buzzerPlayBeep(&beep);
			beep.freq = C6;
			beep.duration=500;
			buzzerPlayBeep(&beep);*/


			buzzerPlayMelodyFromMem(MusicSecretDiscovered);

		}
		else if(SW2)
		{
			LED_4_SET();
			beep.vol=30;
			beep.freq=10;
			while(beep.freq<6000)
			{
				buzzerPlayBeep(&beep);
				beep.freq+=100;
				LED_3_TOGGLE();
			}
			delay_ms(500);
			LED_4_CLEAR();
		}
		else
		{
//			motorLeftSetSpeed(0);
//			motorRegulatorDisable();
//			motorRightSetSpeed(0);
		}
		if (uartStatus == UART_STATUS_CMD_RECEIVED)
		{
			uartStatus=UART_STATUS_WAIT_FOR_CMD;
			switch (uartCurrentCmd)
			{
				case UART_CMD_SET_P:
					xprintf("Reg: %s\n",motorRegPString);
					xprintf("RxBuf: %s\n",uartRxBuffer);
					xprintf("RegF: %s\n",ftostr(0,xatof((char*)uartRxBuffer),6));
				break;
				case UART_CMD_TEST:
					xprintf("Dir 42.0042: %s\n",ftostr(0,42.0042,4));
					xprintf("Dir 0.0042: %s\n",ftostr(0,0.0042,4));
					xprintf("Xatof 0.0042: %s\n",ftostr(0,xatof("0.0042"),4));
				break;
			}
		}
		if (systemTime>sendTime)
		{
			sendTime+=100;
			LED_5_TOGGLE();
			xprintf("X: %d\n",IMUValX);
			xprintf("Y: %d\n",IMUValY);
			xprintf("Z: %d\n",IMUValZ);
			xprintf("Roll: %d\n",IMUValRoll);
			xprintf("Pitch: %d\n",IMUValPitch);
			xprintf("Yaw: %d\n\n",IMUValYaw);

			//sendADCTacho();
			//sendADCVals();
		}
	}

	while(!(SW1 && SW2))
	{
		showIrSensors();
		if(SW1)
		{
			LED_ALL_CLEAR();
			delay_ms(100);
			if(++currentSens>IR_SENS_MAX)
			{
				currentSens=IR_SENS_MAX;
			}
			irSetSens(currentSens);
			ledShowNumber(currentSens);
			delay_ms(1000);
		}
		else if(SW2)
		{
			LED_ALL_CLEAR();
			delay_ms(100);
			if(!--currentSens)
			{
				currentSens=1;
			}
			irSetSens(currentSens);
			ledShowNumber(currentSens);
			delay_ms(1000);
		}
		if (systemTime>sendTime)
		{
			sendTime=systemTime+400;
			sendADCVals();
		}

	}
	irDisable();
	ledUartInit();

	startState=START_POWER;

	showBattery();

#define UART_PERIOD 10
	unsigned long uartSendTime=systemTime;
	unsigned long ledBlinkTime=systemTime;

	float leftPwr=0;
	float rightPwr=0;
	//Wait until the startmodule statemachine is in started
	//Make sure the IR-sensors are not started
	//float tmpSpd=0;
	irDisable();

	//In case I forget to wave in front of the sensor
	leftPwr=0.5;
	rightPwr=-0.5;
	while(startState!=START_STARTED)
	{
		if(systemTime>ledBlinkTime)
		{
			ledBlinkTime=systemTime+1000;
			LED_3_TOGGLE();
		}
		if (startState==START_PROGRAMMING)
		{
			//Show that the robot has accepted the programming
			LED_START_SET();
			LED_2_SET();
			delay_ms(500);
			LED_START_CLEAR();
			delay_ms(500);
			LED_START_SET();
			delay_ms(500);
			LED_START_CLEAR();
			startState=START_POWER;
			LED_2_CLEAR();
		}
		if(SW1)
		{
			motorLeftSetSpeed(0.5);
			motorRightSetSpeed(0.5);
		}
		else
		{
			motorLeftSetSpeed(0);
			motorRightSetSpeed(0);
		}
		if(SW2)
		{
			irEnable();
			while(SW2)
			{
				showIrSensors();
				if(IR_RIGHT)
				{
					leftPwr=0.5;
					rightPwr=-0.5;
				}
				else if(IR_LEFT)
				{
					rightPwr=0.5;
					leftPwr=-0.5;
				}
			}
			irDisable();
			LED_2_CLEAR();//MEASUE BATTERY VOLTAGE!!!!
			LED_3_CLEAR();
			LED_4_CLEAR();
			LED_5_CLEAR();
		}
	}
	//Show that the robot is active
	//LED_START_SET();
	//Activate IR-sensors
	irEnable();
	//Value is HIGHER when white
#define EDGE_THRESH	900
#define SPEED_FULL_REVERSE 0.7
#define SPEED_TURN_SECOND 0.6
#define SPEED_TURN_PRIMARY 0.7
#define HOLD_TIME_REVERSE 300
#define HOLD_TIME_TURN 400
	//sumo program
	unsigned long holdTime=0;
	unsigned long holdTime2=0;
	unsigned long updateTime=0;
	float leftPwr2=0;
	float rightPwr2=0;
	while(1)
	{
		//Blink while waiting for start command
		if(systemTime>ledBlinkTime)
		{
			ledBlinkTime=systemTime+1000;
			LED_3_TOGGLE();
		}
		if(startState==START_STARTED)
		{
			LED_START_SET();
			irEnable();
			updateTime=systemTime+350;
			motorLeftSetSpeed(leftPwr);
			motorRightSetSpeed(rightPwr);
			//Sumo program is run as long as robot is in state started
			while(startState==START_STARTED)
			{
				if(ADCEdgeLeft>EDGE_THRESH && ADCEdgeRight>EDGE_THRESH)	//Right on the line
				{
					leftPwr=-SPEED_FULL_REVERSE;
					rightPwr=-SPEED_FULL_REVERSE;
					holdTime=HOLD_TIME_REVERSE;

					leftPwr2=-0.6;
					rightPwr2=0.6;
					holdTime2=HOLD_TIME_TURN;
				}
				else if(ADCEdgeLeft>EDGE_THRESH)
				{
					leftPwr=-SPEED_FULL_REVERSE;
					rightPwr=-SPEED_FULL_REVERSE;
					holdTime=HOLD_TIME_REVERSE;

					leftPwr2=SPEED_TURN_SECOND;
					rightPwr2=-SPEED_TURN_PRIMARY;
					holdTime2=HOLD_TIME_TURN;
				}
				else if(ADCEdgeRight>EDGE_THRESH)
				{

					leftPwr=-SPEED_FULL_REVERSE;
					rightPwr=-SPEED_FULL_REVERSE;
					holdTime=HOLD_TIME_REVERSE;

					leftPwr2=-SPEED_TURN_PRIMARY;
					rightPwr2=SPEED_TURN_SECOND;
					holdTime2=HOLD_TIME_TURN;
				}
				else if(IR_FR_LEFT && IR_FR_RIGHT)
				{
					leftPwr=0.7;
					rightPwr=0.7;
				}
				else if(IR_FR_LEFT)
				{
					leftPwr=0.35;
					rightPwr=0.6;
				}
				else if(IR_FR_RIGHT)
				{
					leftPwr=0.6;
					rightPwr=0.35;
				}
				else if(IR_LEFT)
				{
					leftPwr=-0.3;
					rightPwr=0.5;
				}
				else if(IR_RIGHT)
				{
					leftPwr=0.5;
					rightPwr=-0.3;
				}
				if(holdTime)
				{
					updateTime=systemTime+holdTime;
					holdTime=0;
					motorLeftSetSpeed(leftPwr);
					motorRightSetSpeed(rightPwr);
				}
				if(systemTime>updateTime)
				{
					if (holdTime2)
					{
						updateTime=systemTime+holdTime2;
						holdTime2=0;
						motorLeftSetSpeed(leftPwr2);
						motorRightSetSpeed(rightPwr2);
					}
					else
					{
						motorLeftSetSpeed(leftPwr);
						motorRightSetSpeed(rightPwr);
					}
				}
				//At the end of the sumo program, check the start module state
				//If Stopped, shut of the motors and go to infite loop, blinking. If both buttons are pressed,
				if(startState==START_STOPPED)
				{
					motorLeftSetSpeed(0);
					motorRightSetSpeed(0);
					while(startState==START_STOPPED)
					{
						delay_ms(1000);
						LED_START_TOGGLE();
						if(SW1 && SW2)
						{
							startState=START_POWER;
						}
					}
				}
				//If programming, shut off the motors, blink programming accepted, disable IR-sensors and jump out from the sumo program
				if(startState==START_PROGRAMMING)
				{
					motorLeftSetSpeed(0);
					motorRightSetSpeed(0);
					LED_START_SET();
					delay_ms(500);
					LED_START_CLEAR();
					delay_ms(500);
					LED_START_SET();
					delay_ms(500);
					LED_START_CLEAR();
					irDisable();
					startState=START_POWER;
				}
				showIrSensors();

				//Check if battery is too low
				if(ADCBatteryLevel<1800)//>6V
				{
					motorLeftSetSpeed(0);
					motorRightSetSpeed(0);
					irDisable();
					delay_ms(50);
					while(ADCBatteryLevel<1800)
					{
						LED_2_TOGGLE();
						LED_3_TOGGLE();
						LED_4_TOGGLE();
						LED_5_TOGGLE();
						delay_ms(100);
					}
					LED_2_CLEAR();
					LED_3_CLEAR();
					LED_4_CLEAR();
					LED_5_CLEAR();
					irEnable();
				}
			}
		}
	}

	return 0;

}


/*
 * Sets the robot into a state when it can be run with a remote control using RC5.
 * Needs to have rc5 init to work
 * Do not run ir sensors when running this function
 */
void runRemoteRobot()
{
	unsigned long uartTime=0;
	irDisable();
	//Below here is program for remote controlled robot
	while(1)
	{
		if(systemTime<abortTime)
		{
			LED_2_SET();
			LED_3_CLEAR();
			switch(driveState)
			{
			case ST_FWD:
				motorLeftSetSpeed(rc5PowerLeft);
				motorRightSetSpeed(rc5PowerRight);
				break;
			case ST_REV:
				motorLeftSetSpeed(-rc5PowerLeft);
				motorRightSetSpeed(-rc5PowerRight);
				break;
			case ST_LEFT:
				motorLeftSetSpeed(-rc5PowerLeft*0.5);
				motorRightSetSpeed(rc5PowerRight*0.5);
				break;
			case ST_RIGHT:
				motorLeftSetSpeed(rc5PowerLeft*0.5);
				motorRightSetSpeed(-rc5PowerRight*0.5);
				break;
			case ST_FWD_LEFT:
				motorLeftSetSpeed(rc5PowerLeft*0.5);
				motorRightSetSpeed(rc5PowerRight);
				break;
			case ST_FWD_RIGHT:
				motorLeftSetSpeed(rc5PowerLeft);
				motorRightSetSpeed(rc5PowerRight*0.5);
				break;
			}
		}
		else
		{
			LED_2_CLEAR();
			LED_3_SET();
			motorLeftSetSpeed(0);
			motorRightSetSpeed(0);
		}
		//Check battery
		if(ADCBatteryLevel<1800)//>6V
		{
			motorLeftSetSpeed(0);
			motorRightSetSpeed(0);
			irDisable();
			delay_ms(50);
			while(ADCBatteryLevel<1800)
			{
				LED_2_TOGGLE();
				LED_3_TOGGLE();
				LED_4_TOGGLE();
				LED_5_TOGGLE();
				delay_ms(100);
			}
			LED_2_CLEAR();
			LED_3_CLEAR();
			LED_4_CLEAR();
			LED_5_CLEAR();
		}
	}
}

/*
 * This function is called when the RC5 receives data
 */
void rc5DataHandler(RC5DATA *data)
{
	//Start of block used for startmodule system//

	//The rc5DataHandler only sets the current state. It is up to the user to implement correct signaling based on the state
	//The start command and state should really be saved in an non-volatile memory. This is yet to be implemented

	//Start module command address.
	if (data->address_low==RC5_ADR_STARTMODULE)
	{
		//Start has the last bit set
		if(data->command == (startAddress | 1))
		{
			startState=START_STARTED;
		}
		//Stop has the last bit cleared
		else if(data->command ==(startAddress & 0b11111110))
		{
			startState=START_STOPPED;
		}
	}
	//Start module programming address
	else if(data->address_low==RC5_ADR_PROGRAMMING)
	{
		startAddress=data->command;
		if(startState!=START_STOPPED)
		{
			startState=START_PROGRAMMING;
		}
	}

	//End of block used for startmodule system//


	//Start of block used for remote robot//
	static unsigned char newCommandCntr=0;
	static unsigned char currentAdr=RC5_ADR_TV1;
	LED_5_TOGGLE();
	if (data->address_low == currentAdr)
	{
		switch(data->command)
		{
			case RC5_CMD_1:
				driveState=ST_FWD_LEFT;
				abortTime=systemTime+ST_TIMEOUT;
				break;
			case RC5_CMD_2:	//Forward
				driveState=ST_FWD;
				abortTime=systemTime+ST_TIMEOUT;
				break;
			case RC5_CMD_3:
				driveState=ST_FWD_RIGHT;
				abortTime=systemTime+ST_TIMEOUT;
				break;
			case RC5_CMD_8:	//Backward
			case RC5_CMD_5:
				driveState=ST_REV;
				abortTime=systemTime+ST_TIMEOUT;
				break;
			case RC5_CMD_4:	//Rotate left
				driveState=ST_LEFT;
				abortTime=systemTime+ST_TIMEOUT;
				break;
			case RC5_CMD_6:	//rotate right
				driveState=ST_RIGHT;
				abortTime=systemTime+ST_TIMEOUT;
				break;
			case RC5_CMD_VPLUS:	//inc speed
				rc5PowerLeft +=0.1;
				if(rc5PowerLeft>1)
				{
					rc5PowerLeft=1;
				}
				rc5PowerRight +=0.1;
				if(rc5PowerRight>1)
				{
					rc5PowerRight=1;
				}
				break;
			case RC5_CMD_VMIN:	//dec speed
				rc5PowerLeft -=0.1;
				if(rc5PowerLeft<0.1)
				{
					rc5PowerLeft=0.1;
				}
				rc5PowerRight -=0.1;
				if(rc5PowerRight<0.1)
				{
					rc5PowerRight=0.1;
				}
				break;
		}
	}
	//Hold "9" for a while to set the current address to the this remote control
	if (data->command == RC5_CMD_9 )
	{
		if(newCommandCntr++>5)
		{
			currentAdr=data->address_low;
			newCommandCntr=0;
		}
	}
	else
	{
		newCommandCntr=0;
	}
	//End of block used for remote robot//

}

/*
 * Send all ADC values through uart
 */
void sendADCVals()
{
	char tmpBuf[40];
	char totBuf[300];
	int tmpLen=0;
	int totLen=0;
	tmpLen=xsprintf(totBuf,"Battraw: %u\n",ADCBatteryLevel);
	totLen+=tmpLen;
	tmpLen=xsprintf(tmpBuf,"Tacho L: %u\n",ADCTachometerLeft);
	strcat(totBuf,tmpBuf);
	totLen+=tmpLen;
	tmpLen=xsprintf(tmpBuf,"Tacho R: %u\n",ADCTachometerRight);
	strcat(totBuf,tmpBuf);
	totLen+=tmpLen;
	tmpLen=xsprintf(tmpBuf,"REF L: %u\n",ADCEdgeLeft);
	strcat(totBuf,tmpBuf);
	totLen+=tmpLen;
	tmpLen=xsprintf(tmpBuf,"REF R: %u\n",ADCEdgeRight);
	strcat(totBuf,tmpBuf);
	totLen+=tmpLen;
	tmpLen=xsprintf(tmpBuf,"Tempraw: %u\n",ADCTempRaw);
	strcat(totBuf,tmpBuf);
	totLen+=tmpLen;
	while(uartGetDMAStatus()){}
	uartSendStringDMA(totBuf,totLen,1);//totLen

//	xprintf("Battraw: %u\n",ADCBatteryLevel);
//	xprintf("Tacho L: %u\n",ADCTachometerLeft);
//	xprintf("Tacho R: %u\n",ADCTachometerRight);
//	xprintf("REF L: %u\n",ADCEdgeLeft);
//	xprintf("REF R: %u\n",ADCEdgeRight);
//	xprintf("Temp: %u\n\n",ADCTempRaw);

}

void sendADCTacho()
{
	xprintf("L Count: %d\n",tachoLeftCount);
	xprintf("L Speed: %d\n",tachoLeftCountPerSecond);
	xprintf("L Target: %d\n",targetPulsePerSecondLeft);
	xprintf("L Err: %d\n",(signed long)(targetPulsePerSecondLeft-tachoLeftCountPerSecond));
	xprintf("L ErrI: %d\n",errILeft);
	xprintf("L ErrD: %d\n",errDLeft);
	xprintf("L Out: %d\n",leftOutputPower);
	xprintf("P: %s\nI: %s\nD: %s\n",motorRegPString,motorRegIString,motorRegDString);

	xprintf("R Count: %u\n",tachoRightCount);
	xprintf("R Speed: %u\n",tachoRightCountPerSecond);

	xprintf("ADC time: %u\n\n",adcTime);
}

/*
 * Shows the state of the IR-sensors using the LEDs
 */
void showIrSensors()
{
	if(IR_LEFT)
	{
		LED_5_SET();
	}
	else
	{
		LED_5_CLEAR();
	}

	if(IR_FR_LEFT)
	{
		LED_4_SET();
	}
	else
	{
		LED_4_CLEAR();
	}
	if(IR_FR_RIGHT)
	{
		LED_3_SET();
	}
	else
	{
		LED_3_CLEAR();
	}
	if(IR_RIGHT)
	{
		LED_2_SET();
	}
	else
	{
		LED_2_CLEAR();
	}
}

/*
 * Shows the current state of the battery.
 * With the current voltage divider (10k and 20k) the values are:
 * 2500 = 8V
 * 2300 = 7.4V
 * 2200 = 7V
 * 2000 = 6.4V
 */
void showBattery()
{
	delay_ms(100);
	LED_ALL_CLEAR();
	//Critically low battery. Stop everything. This ensures that the robot can never start at this low level
	while(ADCBatteryLevel<1700)
	{
		LED_2_TOGGLE();
		delay_ms(300);
	}
	if(ADCBatteryLevel>2000)
	{
		LED_2_SET();
	}
	if(ADCBatteryLevel>2200)
	{
		LED_3_SET();
	}
	if(ADCBatteryLevel>2300)
	{
		LED_4_SET();
	}
	if(ADCBatteryLevel>2500)
	{
		LED_5_SET();
	}
	delay_ms(1000);
	LED_ALL_CLEAR();
}

