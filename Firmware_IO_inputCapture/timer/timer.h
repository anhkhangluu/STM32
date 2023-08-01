#ifndef _TIMER_H_
#define _TIMER_H_

#include "structer.h"
#define MAX_TIME        3
#define TIME_RUN        1
#define TIME_STOP       0
#define TIME_FINISH     2

extern timer mtimer;

void timer_Init(void);
uint8_t timer_Status();
void timer_Clear();
void timer_Start(uint32_t count);

uint32_t time_getCount();
void time_Stop();
#endif
