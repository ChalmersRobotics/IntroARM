/*
 *	mpu6050.c
 *
 *	Created on: 19 mar 2015
 *		Author: Sterna and PŒsse
 */

#include "stm32f10x_i2c.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "misc.h"
#include "mpu6050.h"
#include "time.h"

//debug
#include "LEDs.h"

static unsigned char MPU6050Busy=0;
static unsigned char MPU6050RegisterBuffer[14];
volatile unsigned char mpuIsInited=0;

static void i2CReset();
static void i2cInitHw();
static void i2cWriteRegisterBlocking(unsigned char addr, unsigned char value);

volatile int16_t IMUValX=0;
volatile int16_t IMUValY=0;
volatile int16_t IMUValZ=0;
volatile int16_t IMUValRoll=0;
volatile int16_t IMUValPitch=0;
volatile int16_t IMUValYaw=0;

static int16_t IMUValXOffset=0;
static int16_t IMUValYOffset=0;
static int16_t IMUValZOffset=0;
static int16_t IMUValRollOffset=0;
static int16_t IMUValPitchOffset=0;
static int16_t IMUValYawOffset=0;



void mpu6050Init()
{
	i2cInitHw();

	delay_ms(100);
	i2cWriteRegisterBlocking(107,0);
	delay_ms(1);
	i2cWriteRegisterBlocking(0x19,7);
	delay_ms(1);
	i2cWriteRegisterBlocking(0x1A,3);
	delay_ms(1);
	i2cWriteRegisterBlocking(0x1B,0x18);
	delay_ms(1);
	i2cWriteRegisterBlocking(0x1C,8);
	delay_ms(1);
	i2cWriteRegisterBlocking(0x23,0);
	delay_ms(1);
	mpuIsInited=1;
}

static void i2cInitHw()
{
	//Enable clock for I2C module and I/O pins
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,DISABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,ENABLE);
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1,ENABLE);
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1,DISABLE);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);

	//GPIO init
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_OD;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);

	//I2C init
	I2C_InitTypeDef I2C_InitStruct;
	I2C_StructInit(&I2C_InitStruct);
	I2C_InitStruct.I2C_Ack=I2C_Ack_Enable;
	I2C_InitStruct.I2C_Mode=I2C_Mode_I2C;
	I2C_InitStruct.I2C_ClockSpeed=400000;
	I2C_Init(I2C1,&I2C_InitStruct);


	I2C_ITConfig(I2C1,I2C_IT_EVT | I2C_IT_BUF,ENABLE);
	I2C_Cmd(I2C1,ENABLE);

	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_InitStructure.NVIC_IRQChannel=I2C1_EV_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=5;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=5;
	NVIC_Init(&NVIC_InitStructure);
}



/*
 * Parses the buffer received from MPU6050
 * http://image.wikifoundry.com/image/1/gPhN8K8G60yyJ1xIoc7iHg25317/GW500H283
 * Due to the orientation of the chip, X and Y axis are inverted, to comply with ISO8855 (right-hand system)
 * Roll and pitch are inverted
 */
void mpu6050ParseBuffer()
{
	signed short value;
	value=(signed short)((MPU6050RegisterBuffer[0]<<8)|MPU6050RegisterBuffer[1]);
	IMUValX=-value;
	value=(signed short)((MPU6050RegisterBuffer[2]<<8)|MPU6050RegisterBuffer[5]);
	IMUValY=-value;
	value=(signed short)((MPU6050RegisterBuffer[4]<<8)|MPU6050RegisterBuffer[6]);
	IMUValZ=value;

	value=(signed short)((MPU6050RegisterBuffer[8]<<8)|MPU6050RegisterBuffer[9]);
	IMUValRoll=-value;
	value=(signed short)((MPU6050RegisterBuffer[10]<<8)|MPU6050RegisterBuffer[11]);
	IMUValPitch=-value;
	value=(signed short)((MPU6050RegisterBuffer[12]<<8)|MPU6050RegisterBuffer[13]);
	IMUValYaw=value;

	//Consider the first values as offset
	if(!IMUValPitchOffset)
	{
		IMUValPitchOffset=IMUValPitch;
	}
	if(!IMUValRollOffset)
	{
		IMUValRollOffset=IMUValRoll;
	}
	if(!IMUValYawOffset)
	{
		IMUValYawOffset=IMUValYaw;
	}

	if(!IMUValXOffset)
	{
		IMUValXOffset=IMUValX;
	}
	if(!IMUValYOffset)
	{
		IMUValYOffset=IMUValY;
	}
	if(!IMUValZOffset)
	{
		IMUValZOffset=IMUValZ;
	}

}

void I2C1_EV_IRQHandler()
{
	LED_4_TOGGLE();
	static unsigned char state=1;
	unsigned char lastState=state;
	switch(state)
	{
	case 1:
		if(I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT))
		{
			I2C_SendData(I2C1,0xD0|0);
			I2C_GenerateSTOP(I2C1,DISABLE);
			I2C_GenerateSTART(I2C1,DISABLE);
			state=2;
		}
		break;
	case 2:
		if(I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
		{
			I2C_SendData(I2C1,0x3B);
			state=3;
		}
		break;
	case 3:
		if(I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED))
		{
			I2C_GenerateSTART(I2C1,ENABLE);
			state=4;
		}
		break;
	case 4:
		if(I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT))
		{
			I2C_SendData(I2C1,0xD0|1);
			state=5;
		}
		break;
	case 5:
		if(I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
		{
			I2C_AcknowledgeConfig(I2C1,ENABLE);
			state=6;
		}
		break;
	case 6://AccX
	case 7:
	case 8://AccY
	case 9:
	case 10://AccZ
	case 11:
	case 12://Temperature
	case 13:
	case 14://GyroX
	case 15:
	case 16://GyroY
	case 17:
	case 18://GyroZ
	case 19:
		if(I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_RECEIVED))
		{
			MPU6050RegisterBuffer[state-6]=I2C_ReceiveData(I2C1);
			state++;
			if(state==19)
			{
				I2C_AcknowledgeConfig(I2C1,DISABLE);
				I2C_GenerateSTOP(I2C1,ENABLE);
			}
			if(state==20)
			{
				mpu6050ParseBuffer();
				state=1;
				MPU6050Busy=0;
			}
		}
		break;
	default:
		state=1;
		break;
	}
	if((lastState==state)&&I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT))
		state=1;

	I2C_ClearITPendingBit(I2C1,I2C_IT_EVT);
	I2C_ClearITPendingBit(I2C1,I2C_IT_BUF);
}

//Call this function every 100ms or so, to start a measurement.
void mpu6050Process()
{
	if(mpuIsInited)
	{
		if(MPU6050Busy)
			i2CReset();
			LED_3_TOGGLE();
		MPU6050Busy=1;
		I2C_GenerateSTART(I2C1,ENABLE);
		LED_2_TOGGLE();
	}
}

static void i2cWriteRegisterBlocking(unsigned char addr, unsigned char value)
{
	unsigned char retries=3;

	I2C_ITConfig(I2C1,I2C_IT_EVT | I2C_IT_BUF,DISABLE);

	do{
		mpuTimeOut=MPU_TIMEOUT_MS;
		I2C_GenerateSTART(I2C1,ENABLE);
		while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT) && mpuTimeOut);
		I2C_SendData(I2C1,0xD0);
		while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) && mpuTimeOut);
		I2C_SendData(I2C1,addr);
		while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED) && mpuTimeOut);
		I2C_SendData(I2C1,value);
		while(!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED) && mpuTimeOut);
		I2C_GenerateSTOP(I2C1,ENABLE);
		if(!mpuTimeOut)
			i2CReset();
	}while(retries-- && !mpuTimeOut);

	I2C_ClearITPendingBit(I2C1,I2C_IT_EVT);
	I2C_ClearITPendingBit(I2C1,I2C_IT_BUF);

	I2C_ITConfig(I2C1,I2C_IT_EVT | I2C_IT_BUF,ENABLE);
}


static void i2CReset()
{
	I2C_ITConfig(I2C1,I2C_IT_EVT | I2C_IT_BUF,DISABLE);
	I2C_Cmd(I2C1,DISABLE);
	i2cInitHw();
}
