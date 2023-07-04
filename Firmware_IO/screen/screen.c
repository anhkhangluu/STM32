/*
 * screen.c
 *
 *  Created on: Jun 27, 2023
 *      Author: Admin
 */
#include "stdio.h"
#include "LCD.h"
#include "screen.h"

static screenData screenBuffer;

void screen_DataMeasure(dataMeasure data, uint8_t setCalib, uint8_t measIndex) {///////////////////////////
	if (CALIBSET == setCalib) {
		if (MEASUREALL == data.mode) {
			snprintf(screenBuffer.line1, LCD_LINE_SIZE + 1, "	MEASUREMENT %01d",
					measIndex);
			snprintf(screenBuffer.line2, LCD_LINE_SIZE + 1,
					"20%02d-%02d-%02d %02d:%02d", data.time.year,
					data.time.month, data.time.day, data.time.hour,
					data.time.minute);
			snprintf(screenBuffer.line3, LCD_LINE_SIZE + 1,
					"X=%+-#1.2f  Y=%+-#1.2f", (float) data.coordinates.X / 100,
					(float) data.coordinates.Y / 100);
			snprintf(screenBuffer.line4, LCD_LINE_SIZE + 1,
					"Z=%+-#1.2f  R=%+-#1.2f", (float) data.coordinates.Z / 100,
					(float) data.coordinates.R / 100);
//			screenBuffer.line4[6] = 0xDF;
//			screenBuffer.line4[15] = 0xDF;
		} else if (ZERROR1 == data.mode) {
			snprintf(screenBuffer.line1, LCD_LINE_SIZE + 1, "	MEASUREMENT %01d",
					measIndex);
			snprintf(screenBuffer.line2, LCD_LINE_SIZE + 1,
					"20%02d-%02d-%02d %02d:%02d", data.time.year,
					data.time.month, data.time.day, data.time.hour,
					data.time.minute);
			snprintf(screenBuffer.line3, LCD_LINE_SIZE + 1, "X=....   Y=.... ");
			snprintf(screenBuffer.line4, LCD_LINE_SIZE + 1, "Z=....   R=.... ");
		} else if (ZERROR2 == data.mode) {
			snprintf(screenBuffer.line1, LCD_LINE_SIZE + 1, "	MEASUREMENT %01d",
								measIndex);
			snprintf(screenBuffer.line2, LCD_LINE_SIZE + 1,
					"20%02d-%02d-%02d %02d:%02d", data.time.year,
					data.time.month, data.time.day, data.time.hour,
					data.time.minute);
			snprintf(screenBuffer.line3, LCD_LINE_SIZE + 1,
					"X=%+-#1.2f  Y=%+-#1.2f", (float) data.coordinates.X / 100,
					(float) data.coordinates.Y / 100);
			snprintf(screenBuffer.line4, LCD_LINE_SIZE + 1,
					"Z=....   R=%+-#1.2f", (float) data.coordinates.R / 100);
//			snprintf(screenBuffer.line4, LCD_LINE_SIZE + 1,
//					"A=%+-#1.1f   B=%+-#1.1f ",
//					(float) data.coordinates.aX / 10,
//					(float) data.coordinates.aY / 10);
//			screenBuffer.line4[6] = 0xDF;
//			screenBuffer.line4[15] = 0xDF;
		} else if (ZONLY == data.mode) {
			snprintf(screenBuffer.line1, LCD_LINE_SIZE + 1, "	MEASUREMENT %01d",
								measIndex);
			snprintf(screenBuffer.line2, LCD_LINE_SIZE + 1,
					"20%02d-%02d-%02d %02d:%02d", data.time.year,
					data.time.month, data.time.day, data.time.hour,
					data.time.minute);
			snprintf(screenBuffer.line3, LCD_LINE_SIZE + 1, "X=....   Y=.... ");
			snprintf(screenBuffer.line4, LCD_LINE_SIZE + 1,
					"Z=%+-#1.2f  R=.... ", (float) data.coordinates.Z / 100);
//			snprintf(screenBuffer.line4, LCD_LINE_SIZE + 1, "A=....   B=.... ");
		} else {
			snprintf(screenBuffer.line1, LCD_LINE_SIZE + 1, "	MEASUREMENT %01d",
					measIndex);
			snprintf(screenBuffer.line2, LCD_LINE_SIZE + 1, "                ");
			snprintf(screenBuffer.line3, LCD_LINE_SIZE + 1, "   No Data...!  ");
			snprintf(screenBuffer.line4, LCD_LINE_SIZE + 1, "                ");
		}
	} else {
		snprintf(screenBuffer.line1, LCD_LINE_SIZE + 1, "	MEASUREMENT %01d",
				measIndex);
		snprintf(screenBuffer.line2, LCD_LINE_SIZE + 1, "....-..-.. ..:..");
		snprintf(screenBuffer.line3, LCD_LINE_SIZE + 1, "X=....   Y=.... ");
		snprintf(screenBuffer.line4, LCD_LINE_SIZE + 1, "Z=....   R=.... ");
	}

	LCD_Puts(0, 0, screenBuffer.line1);
	LCD_Puts(0, 1, screenBuffer.line2);
	LCD_Puts(0, 2, screenBuffer.line3);
	LCD_Puts(0, 3, screenBuffer.line4);
}

void screen_Time(Time time) {
	snprintf(screenBuffer.line2, LCD_LINE_SIZE + 1, "   20%02d-%02d-%02d   ",
			time.year, time.month, time.day);
	snprintf(screenBuffer.line3, LCD_LINE_SIZE + 1, "    %02d:%02d    ",
			time.hour, time.minute);

	LCD_Puts(0, 0, "  TIME SETTING  ");
	LCD_Puts(0, 1, screenBuffer.line2);
	LCD_Puts(0, 2, screenBuffer.line3);
}

void screen_setDateTime(Time time, CycleTime cycle) {
	switch (cycle) {
	case SET_YEAR:
		snprintf(screenBuffer.line2, LCD_LINE_SIZE + 1,
				"  [20%02d]-%02d-%02d  ", time.year, time.month, time.day);
		snprintf(screenBuffer.line3, LCD_LINE_SIZE + 1, "     %02d:%02d      ",
				time.hour, time.minute);
		break;
	case SET_MONTH:
		snprintf(screenBuffer.line2, LCD_LINE_SIZE + 1,
				"  20%02d-[%02d]-%02d  ", time.year, time.month, time.day);
		snprintf(screenBuffer.line3, LCD_LINE_SIZE + 1, "     %02d:%02d      ",
				time.hour, time.minute);
		break;
	case SET_DAY:
		snprintf(screenBuffer.line2, LCD_LINE_SIZE + 1,
				"  20%02d-%02d-[%02d]  ", time.year, time.month, time.day);
		snprintf(screenBuffer.line3, LCD_LINE_SIZE + 1, "     %02d:%02d      ",
				time.hour, time.minute);
		break;
	case SET_HOUR:
		snprintf(screenBuffer.line2, LCD_LINE_SIZE + 1,
				"   20%02d-%02d-%02d   ", time.year, time.month, time.day);
		snprintf(screenBuffer.line3, LCD_LINE_SIZE + 1, "    [%02d]:%02d     ",
				time.hour, time.minute);
		break;
	case SET_MINUTE:
		snprintf(screenBuffer.line2, LCD_LINE_SIZE + 1,
				"   20%02d-%02d-%02d   ", time.year, time.month, time.day);
		snprintf(screenBuffer.line3, LCD_LINE_SIZE + 1, "    %02d:[%02d]     ",
				time.hour, time.minute);
		break;
	}
//	LCD_Puts(0, 0, "  TIME SETTING  ");
	LCD_Puts(0, 1, screenBuffer.line2);
	LCD_Puts(0, 2, screenBuffer.line3);
}

void screen_OptionMenu(optionScreen_e_t optionIndex) {
	if (optionIndex == measurement1HisList || optionIndex == measurement2HisList)
		;
	else if (optionIndex <= minNoneOption)
		optionIndex = measurement1Setting;
	else if	(optionIndex >= maxNoneOption)
		optionIndex = timeSetting;
	else
		;

	switch (optionIndex) {
	case measurement1Setting:
		sprintf(screenBuffer.line2, "MEASUREMENT 1");
		sprintf(screenBuffer.line3,  "SETTING");
		break;
	case measurement2Setting:
		sprintf(screenBuffer.line2,  "MEASUREMENT 2");
		sprintf(screenBuffer.line3,  "SETTING");
		break;
	case measurementHis:
		sprintf(screenBuffer.line2,  "MEASUREMENT");
		sprintf(screenBuffer.line3,  "HISTORY LIST");
		break;
	case measurement1HisList:
		snprintf(screenBuffer.line2, LCD_LINE_SIZE + 1, "MEASUREMENT 1");
		snprintf(screenBuffer.line3, LCD_LINE_SIZE + 1, "HISTORY LIST");
		break;
	case measurement2HisList:
		snprintf(screenBuffer.line2, LCD_LINE_SIZE + 1, "MEASUREMENT 2");
		snprintf(screenBuffer.line3, LCD_LINE_SIZE + 1, "HISTORY LIST");
		break;
	case VDLRZinput:
		sprintf(screenBuffer.line2, "V;D;L;R;Z INPUT");
		sprintf(screenBuffer.line3,  "%s", "");
		break;
	case timeSetting:
		sprintf(screenBuffer.line2, "TIME SETTING");
		sprintf(screenBuffer.line3,  "%s", "");
		break;
	default:
		break;
	}
	LCD_Puts(0, 1, screenBuffer.line2);
	LCD_Puts(0, 2, screenBuffer.line3);
}
