/*
 *	flash.h
 *
 *	Created on: 12 feb 2015
 *		Author: Sterna
 */
#ifndef FLASH_H_
#define FLASH_H_


#include "stm32f10x_conf.h"
#include "time.h"
#include <string.h> //For memcpy
//-------------------------------------------------------------------
// Defines
//-------------------------------------------------------------------
#define FLASH_START_ADDRESS				0x0801FC00				//Start of flash page 127 (1KByte)

//CB har 128kB Flash

typedef enum
{
	FLASH_TYPE_UINT8,
	FLASH_TYPE_UINT16,
	FLASH_TYPE_UINT32,
	FLASH_TYPE_INT8,
	FLASH_TYPE_INT16,
	FLASH_TYPE_INT32,
	FLASH_TYPE_FLOAT
}flashType;

//Enum containing all parameters stored in the flash
typedef enum
{
	FLASH_PARAM_KP,
	FLASH_PARAM_KI,
	FLASH_PARAM_KD,
	FLASH_PARAM_START_STATE,
	FLASH_NOF_PARAMS
}flashParamType;


//-------------------------------------------------------------------
// Functions
//-------------------------------------------------------------------
void flashInit();
unsigned char flashRead(uint32_t offset,uint32_t *outbuffer,uint8_t numWords);
unsigned char flashWrite(uint32_t offset,uint32_t *inbuffer,uint8_t numWords);
void flashResetDataOnFlash();
unsigned char flashReadAllParameters();
unsigned char flashSaveAllParameters();
unsigned char flashSaveParam(void* val, flashParamType param);
unsigned char flashReadParam(void* outval,flashType type, flashParamType param);





#endif /* FLASH_H_ */
