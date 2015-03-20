/*
 *	utils.h
 *
 *	Created on: 31 dec 2014
 *		Author: Sterna
 */
#ifndef UTILS_H_
#define UTILS_H_

#ifndef _BV
#define _BV(x) (1<<x)
#endif


char* ftostr(char* buf, float value, unsigned char places);
float xatof(char *s);
signed long xatoi2(char* s);
unsigned char xisdigit(char c);

#endif /* UTILS_H_ */
