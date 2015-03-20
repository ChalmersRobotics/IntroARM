/*
 *	utils.c
 *
 *	Created on: 31 dec 2014
 *		Author: Sterna
 *		Contains some useful functions
 */

#include "utils.h"
#include <ctype.h>
#include <math.h>
#include "xprintf.h"


/*
 * Can be used with printf when double values are not
 * supported (because hardfloat is used)
 * Buf is a pointer to a buffer where the output will be written
 * Float is the value to be converted
 * Places is the number of decimals to be used
 * If no buffer is provided, the function will use its own buffer
 */
char* ftostr(char* buf, float value, unsigned char places)
{
	static char _buffer[15];
	unsigned long whole;
//	char sign[2] = "";

//	if (value < 0) {
//		value = -value;
//		sign[0] = '-';
//		sign[1] = '\0';
//	}

	whole = (signed long) value;
	char fracBuffer[10];
	fracBuffer[places]='\0';
	for (unsigned char i=0;i<places;i++)
	{
		value=value*10.0;
		fracBuffer[i]=(unsigned char)('0'+(signed long)(value)%10);
	}
	if (buf)
	{
		xsprintf(buf, "%D.%s",whole,fracBuffer);
		return 0;
	}
	else
	{
		xsprintf(_buffer, "%D.%s",whole,fracBuffer);
		return _buffer;
	}
	//xsprintf(_buffer, "%s%lu.%*.*lu", sign, whole, places, places, fraction); //xsprintf does not support *
}

/*
 * Convert a string into a float.
 * Will return 0 if any incorrect character is received
 * Does not support scientific notification (E)
 */
float xatof(char *s)
{
	float a = 0.0;
	unsigned short magnitude = 1;
	signed char sign=1;
	unsigned long whole=0;
	unsigned long frac=0;
	unsigned char analysingFrac=0;
	unsigned char c;
	if(*s == '-')
	{
		sign=-1;
		s++;
	}
	while ((c = *s++) != '\0')
	{
		if(xisdigit(c))
		{
			if(!analysingFrac)
			{
				whole = whole*10 + (c - '0');
			}
			else
			{
				frac = frac*10 + (c - '0');
				magnitude=magnitude*10;
			}
		}
		else if(c=='.')
		{
			analysingFrac=1;
		}
		else
		{
			//Error, unknown character, return "safe" value
			return 0;
		}
	}
	a=(float)(whole)+(float)frac/(float)magnitude;
	a=a*((float)(sign));

	return a;
}

/*
 *	Convert a string into an integer
 *	The string needs to nullterminated
 */
signed long xatoi2(char* s)
{
	signed char sign=1;
	unsigned char c=0;
	signed long out=0;
	//Skip leading spaces (if any)
	if(*s==' ')
	{
		while(*++s==' '){}
	}
	//Is negative?
	if(*s == '-')
	{
		sign=-1;
		s++;
	}
	while((c=*s++)!='\0')
	{
		if(xisdigit(c))
		{
			out=out*10 + (c-'0');
		}
		else
		{
			//Error
			return 0;
		}
	}
	return out*sign;
}

unsigned char xisdigit(char c)
{
	return ((c>='0') && (c<='9'));
}
