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
void screen_showIP(wiz_NetInfo *netInfo);
void screen_noSDCard(void);

void screen_setVDRLZ(VDRLZ_Input VDRLZ, VDRLZ_CycleSet cycle);
void screen_setDateTime(Time time, CycleTime cycle);
void screen_OptionMenu(optionScreen_e_t *optionIndex);
void screen_waitMeasurement(uint8_t measIndex);
void screen_errorXY(uint8_t measIndex);

#endif /* SCREEN_H_ */
