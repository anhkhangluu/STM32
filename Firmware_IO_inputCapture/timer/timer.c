#include "timer.h"

timer mtimer;

uint8_t timer_Status() {
	uint8_t ret = TIME_STOP;
	if ((mtimer.status == TIME_RUN) && (!mtimer.flat)) {
		ret = TIME_RUN;
	} else if ((mtimer.status == TIME_RUN) && (mtimer.flat)) {
		ret = TIME_FINISH;
	} else if ((mtimer.status == TIME_FINISH)
			&& (mtimer.flat)) {
		ret = TIME_FINISH;
	}
	return ret;
}

void timer_Start( uint32_t count) {
	mtimer.count = count;
	mtimer.inc = 0;
	mtimer.flat = 0;
	mtimer.status = TIME_RUN;
}

void timer_Clear() {
	mtimer.status = TIME_STOP;
	mtimer.count = 0;
	mtimer.inc = 0;
	mtimer.flat = 0;
}

uint32_t time_getCount() {
	return mtimer.inc;
}

void time_Stop(){
	mtimer.status = TIME_STOP;
}
