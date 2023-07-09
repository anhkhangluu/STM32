/*
 * screen.h
 *
 *  Created on: Jun 27, 2023
 *      Author: Admin
 */

#ifndef SCREEN_H_
#define SCREEN_H_

#include "structer.h"

typedef enum _optionScreen {
	minNoneOption, //use for out of range
	measurement1Setting,
	measurement2Setting,
	measurementHis,
	VDLRZinput,
	timeSetting,
	showIP,
	maxNoneOption, //use for out of range
	measurement1HisList,
	measurement2HisList
} optionScreen_e_t;

void screen_DataMeasureType1(dataMeasure data, uint8_t setCalib, uint8_t measIndex, uint8_t showHisFlag);
void screen_DataMeasureType2(dataMeasure data, uint8_t setCalib, uint8_t measIndex, uint8_t showHisFlag);
void screen_Time(Time time);

void screen_setDateTime(Time time, CycleTime cycle);
void screen_OptionMenu(optionScreen_e_t *optionIndex);



#endif /* SCREEN_H_ */
