#include "timer.h"

timer mtimer[MAX_TIME] = { { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, };

uint8_t timer_Status(uint8_t time_id) {
	uint8_t ret = TIME_STOP;
	if ((mtimer[time_id].status == TIME_RUN) && (!mtimer[time_id].flat)) {
		ret = TIME_RUN;
	} else if ((mtimer[time_id].status == TIME_RUN) && (mtimer[time_id].flat)) {
		ret = TIME_FINISH;
	} else if ((mtimer[time_id].status == TIME_FINISH)
			&& (mtimer[time_id].flat)) {
		ret = TIME_FINISH;
	}
	return ret;
}

void timer_Start(uint8_t time_id, uint32_t count) {
	if (time_id >= MAX_TIME)
		return;
	mtimer[time_id].count = count;
	mtimer[time_id].inc = 0;
	mtimer[time_id].flat = 0;
	mtimer[time_id].status = TIME_RUN;
}

void timer_Clear(uint8_t time_id) {
	if (time_id >= MAX_TIME)
		return;
	mtimer[time_id].status = TIME_STOP;
	mtimer[time_id].count = 0;
	mtimer[time_id].inc = 0;
	mtimer[time_id].flat = 0;
}

uint32_t time_Stop(uint8_t time_id) {
	mtimer[time_id].status = TIME_STOP;
	return mtimer[time_id].inc;
}
