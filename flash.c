/*
 *	flash.c
 *
 *	Created on: 12 feb 2015
 *		Author: Sterna
 *
 *		Handles the saving of data into flash. Data is stored on the last page of the flash
 *		and always takes up a multiple of pages. One page is 1KByta data.
 *
 *		All parameters take up 32 bits each (even if they are actually smaller)
 *		To add a new parameter, add it to the enum in flash.h and call flashSaveParam() when you want to save it.
 *
 */

#include "flash.h"



static uint32_t flashParams[FLASH_NOF_PARAMS];

/*
 * Init flash saving module
 */
void flashInit()
{
	//Read params from flash into RAM
	flashReadAllParameters();
}

/*
 * To save a param, call flashSaveParam(&val,param);
 * where val is the value to save and param is the parameter identifier
 * Note that the val may be at most 32 bits
 * Returns 0 if the param does not exist, 1 otherwise
 */
unsigned char flashSaveParam(void* val, flashParamType param)
{
	if(param >= FLASH_NOF_PARAMS)
	{
		//No such param
		return 0;
	}
	flashParams[param]=*((uint32_t*)val);
	return 1;

}

/*
 * Gets a certain parameter from the flashParams list
 * Outval is a pointer to where the output value is stored.
 * Type is the type of the parameter to be read. The value can be one of the types defined in FLASH_TYPE
 * Param is the parameter to be read. The value can be one of the values defined in FLASH_PARAM
 * The flashParams list needs to be updated on beforehand using flashReadAllParams
 * Returns 0 if the param or type does not exist, 1 otherwise
 */
unsigned char flashReadParam(void* outval,flashType type, flashParamType param)
{
	//Check if param exists
	if(param >= FLASH_NOF_PARAMS)
	{
		return 0;
	}
	switch (type)
	{
		case FLASH_TYPE_UINT8:
			*((uint8_t*)outval)=*((uint8_t*)&flashParams[param]);
		break;
		case FLASH_TYPE_UINT16:
			*((uint16_t*)outval)=*((uint16_t*)&flashParams[param]);
		break;
		case FLASH_TYPE_UINT32:
			*((uint32_t*)outval)=*((uint32_t*)&flashParams[param]);
		break;
		case FLASH_TYPE_INT8:
			*((int8_t*)outval)=*((int8_t*)&flashParams[param]);
		break;
		case FLASH_TYPE_INT16:
			*((int16_t*)outval)=*((int16_t*)&flashParams[param]);
		break;
		case FLASH_TYPE_INT32:
			*((int32_t*)outval)=*((int32_t*)&flashParams[param]);
		break;
		case FLASH_TYPE_FLOAT:
			*((float*)outval)=*((float*)&flashParams[param]);
		break;
		default:	//No such type
			return 0;
	}
	return 1;
}

/*
 * Clear all data stored in flash
 */
void flashResetDataOnFlash()
{
	for(unsigned int i=0;i<FLASH_NOF_PARAMS;i++)
	{
		flashParams[i]=0;
	}
	flashSaveAllParameters();
}

/*
 * Updates the flashParams-list with values from the Flash memory. Typically called only at startup
 */
unsigned char flashReadAllParameters()
{
	return flashRead(0,flashParams,FLASH_NOF_PARAMS);
}


/*
 * Saves all current parameter values into flash
 * Note that params needs to be updated in the flashParams list before using flashSaveParam
 * This function takes about 50 ms to run, so be aware of that
 */
unsigned char flashSaveAllParameters()
{
	return flashWrite(0,flashParams,FLASH_NOF_PARAMS);
}

/*
 * General function for reading data from the internal Flash memory
 */
unsigned char flashRead(uint32_t offset,uint32_t *outbuffer,uint8_t numWords)
{
	memcpy(outbuffer,(uint32_t*)(FLASH_START_ADDRESS+offset),4*numWords);
	return 1;
}

/*
 * Write data to internal flash
 * offset specifies the number of addresses from the predefined macro to write (typically 0)
 * inbuffer is the data as a pointer to a list of uint32_t
 * numWords is the number of words (32 bit) to be written
 * Note: This function takes about 30-50 ms to run. Be aware of that.
 */
unsigned char flashWrite(uint32_t offset,uint32_t *inbuffer,uint8_t numWords)
{

	FLASH_Unlock();

	if (FLASH_ErasePage(FLASH_START_ADDRESS)!=FLASH_COMPLETE)
	{
		return 0;
	}
	while (numWords>0)
	{
		FLASH_ProgramWord(FLASH_START_ADDRESS+offset,*inbuffer++);
		offset+=4;
		numWords--;
	}
	FLASH_Lock();

	return 1;
}
