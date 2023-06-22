#ifndef _TIMER_H_
#define _TIMER_H_

#include "structer.h"
#define MAX_TIME        3
#define TIME_RUN        1
#define TIME_STOP       0
#define TIME_FINISH     2

extern timer mtimer[MAX_TIME];

void timer_Init(void);
uint8_t timer_Status(uint8_t time_id);
void timer_Clear(uint8_t time_id);
void timer_Start(uint8_t time_id, uint32_t count);
uint32_t time_Stop(uint8_t time_id);
#endif
