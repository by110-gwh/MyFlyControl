#ifndef _TIME_CNT_H
#define _TIME_CNT_H

#include <stdint.h>

typedef struct
{
	//µ¥Î»us
	uint32_t Last_Time;
	uint32_t Now_Time;
	uint32_t Time_Delta;
	uint32_t Time_Delta_INT;
} Testime;

typedef struct
{
	uint16_t hour;
	uint16_t minute;
	uint16_t second;
	uint16_t microsecond;
} Time_t;

extern Time_t Time_Sys;

void Get_Time_Init(void);
void Get_Time_Period(Testime *Time_Lab);
#endif

