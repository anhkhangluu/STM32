/*
 * screen.c
 *
 *  Created on: Jun 27, 2023
 *      Author: Admin
 */
#include "stdio.h"
#include "LCD.h"
#include "screen.h"
#include "stdlib.h"
#include "main.h" ///use for cdc debug

#define SHOW_AB_FLAG		1
#define NOT_SHOW_AB_FLAG	0

static dataMeasure capData(dataMeasure input, uint8_t isShowAB);


void screen_DataMeasureType1(dataMeasure data, uint8_t setCalib,
		uint8_t measIndex, uint8_t showHisFlag) {
	LCD_Clear();
	screenData screenBuffer;

	if (!showHisFlag)
		snprintf(screenBuffer.line1, LCD_LINE_SIZE + 1, "MEASUREMENT %01d",
				measIndex);
	else
		snprintf(screenBuffer.line1, LCD_LINE_SIZE + 1, "MEAS.%01d HISTORY",
				measIndex);
	if (CALIBSET == setCalib) {
		data = capData(data, NOT_SHOW_AB_FLAG);
		snprintf(screenBuffer.line2, LCD_LINE_SIZE + 1,
				"20%02d/%02d/%02d %02d:%02d", data.time.year, data.time.month,
				data.time.day, data.time.hour, data.time.minute);
		if (MEASUREALL == data.mode) {
			snprintf(screenBuffer.line3, LCD_LINE_SIZE + 1,
					"X=%s%01d.%02d  Y=%s%01d.%02d",
					(data.coordinates.X >= 0) ? "+" : "-",
					(int16_t)(abs(data.coordinates.X) / 100), abs(data.coordinates.X) % 100,
					(data.coordinates.Y >= 0) ? "+" : "-",
					(int16_t)(abs(data.coordinates.Y) / 100), abs(data.coordinates.Y) % 100);
			snprintf(screenBuffer.line4, LCD_LINE_SIZE + 1,
					"Z=%s%01d.%02d  R=%s%01d.%02d",
					(data.coordinates.Z >= 0) ? "+" : "-",
					(int16_t)(abs(data.coordinates.Z) / 100), abs(data.coordinates.Z) % 100,
					(data.coordinates.R >= 0) ? "+" : "-",
					(int16_t)(abs(data.coordinates.R) / 100), abs(data.coordinates.R) % 100);
			DBG("LCD - MEASUREALL\n");
		} else if (ZERROR1 == data.mode) {
			snprintf(screenBuffer.line3, LCD_LINE_SIZE + 1, "X=.....  Y=.....");
			snprintf(screenBuffer.line4, LCD_LINE_SIZE + 1, "Z=.....  R=.....");
			DBG("LCD - ZERROR1\n");
		} else if (ZERROR2 == data.mode) {
			snprintf(screenBuffer.line3, LCD_LINE_SIZE + 1,
					"X=%s%01d.%02d  Y=%s%01d.%02d",
					(data.coordinates.X >= 0) ? "+" : "-",
					(int16_t)(abs(data.coordinates.X) / 100), abs(data.coordinates.X) % 100,
					(data.coordinates.Y >= 0) ? "+" : "-",
					(int16_t)(abs(data.coordinates.Y) / 100), abs(data.coordinates.Y) % 100);
			snprintf(screenBuffer.line4, LCD_LINE_SIZE + 1,
					"Z=.....  R=%s%01d.%02d", (data.coordinates.R >= 0) ? "+" : "-",
					(int16_t)(abs(data.coordinates.R) / 100), abs(data.coordinates.R) % 100);
			DBG("LCD - ZERROR2\n");
		} else if (ZONLY == data.mode) {
			snprintf(screenBuffer.line3, LCD_LINE_SIZE + 1, "X=.....  Y=.....");
			snprintf(screenBuffer.line4, LCD_LINE_SIZE + 1,
					"Z=%s%01d.%02d  R=.....", (data.coordinates.Z >= 0) ? "+" : "-",
					(int16_t)(abs(data.coordinates.Z) / 100), abs(data.coordinates.Z) % 100);
			DBG("LCD - ZONLY\n");
		} else {
			snprintf(screenBuffer.line2, LCD_LINE_SIZE + 1, "              ");
			snprintf(screenBuffer.line3, LCD_LINE_SIZE + 1, "   No Data...!");
			snprintf(screenBuffer.line4, LCD_LINE_SIZE + 1, "              ");
			DBG("LCD - No data\n");
		}
	} else {
		snprintf(screenBuffer.line2, LCD_LINE_SIZE + 1, "..../../.. ..:..");
		snprintf(screenBuffer.line3, LCD_LINE_SIZE + 1, "X=.....  Y=.....");
		snprintf(screenBuffer.line4, LCD_LINE_SIZE + 1, "Z=.....  R=.....");
	}

	LCD_Puts(0, 0, screenBuffer.line1);
	LCD_Puts(0, 1, screenBuffer.line2);
	LCD_Puts(0, 2, screenBuffer.line3);
	LCD_Puts(0, 3, screenBuffer.line4);
}

void screen_DataMeasureType2(dataMeasure data, uint8_t setCalib,
		uint8_t measIndex, uint8_t showHisFlag) {
	LCD_Clear();
	static screenData screenBuffer;
	if (!showHisFlag)
		snprintf(screenBuffer.line1, LCD_LINE_SIZE + 1, "MEASUREMENT %01d",
				measIndex);
	else
		snprintf(screenBuffer.line1, LCD_LINE_SIZE + 1, "MEAS.%01d HISTORY",
				measIndex);
	if (CALIBSET == setCalib) {

		snprintf(screenBuffer.line2, LCD_LINE_SIZE + 1,
				"20%02d/%02d/%02d %02d:%02d", data.time.year, data.time.month,
				data.time.day, data.time.hour, data.time.minute);
		if (MEASUREALL == data.mode || data.mode == ZERROR2) {
			data = capData(data, SHOW_AB_FLAG);
			snprintf(screenBuffer.line3, LCD_LINE_SIZE + 1, "   A=%s%2d.%01d",
					(data.coordinates.aX >= 0) ? "+" : "-",
					(int16_t)(abs(data.coordinates.aX) / 10), abs(data.coordinates.aX) % 10);
			snprintf(screenBuffer.line4, LCD_LINE_SIZE + 1, "   B=%s%2d.%01d",
					(data.coordinates.aY >= 0) ? "+" : "-",
					(int16_t)(abs(data.coordinates.aY) / 10), abs(data.coordinates.aY) % 10);
			screenBuffer.line3[10] = 0xDF;
			screenBuffer.line4[10] = 0xDF;
		} else if (ZERROR1 == data.mode || data.mode == ZONLY) {

			snprintf(screenBuffer.line3, LCD_LINE_SIZE + 1, "    A=.....");
			snprintf(screenBuffer.line4, LCD_LINE_SIZE + 1, "    B=.....");
		} else {
			snprintf(screenBuffer.line2, LCD_LINE_SIZE + 1, "                ");
			snprintf(screenBuffer.line3, LCD_LINE_SIZE + 1, "   No Data...!  ");
			snprintf(screenBuffer.line4, LCD_LINE_SIZE + 1, "                ");
		}
	} else {
		snprintf(screenBuffer.line2, LCD_LINE_SIZE + 1, "..../../.. ..:..");
		snprintf(screenBuffer.line3, LCD_LINE_SIZE + 1, "    A=.....");
		snprintf(screenBuffer.line4, LCD_LINE_SIZE + 1, "    B=.....");
	}
	LCD_Puts(0, 0, screenBuffer.line1);
	LCD_Puts(0, 1, screenBuffer.line2);
	LCD_Puts(0, 2, screenBuffer.line3);
	LCD_Puts(0, 3, screenBuffer.line4);
}

void screen_Time(Time time) {
	screenData screenBuffer;
	snprintf(screenBuffer.line2, LCD_LINE_SIZE + 1, "  20%02d/%02d/%02d",
			time.year, time.month, time.day);
	snprintf(screenBuffer.line3, LCD_LINE_SIZE + 1, "    %02d:%02d", time.hour,
			time.minute);

	LCD_Puts(0, 0, "  TIME SETTING");
	LCD_Puts(0, 1, screenBuffer.line2);
	LCD_Puts(0, 2, screenBuffer.line3);
	LCD_Puts(0,3, " ");
}

void screen_setDateTime(Time time, CycleTime cycle) {
	LCD_Clear();
	screenData screenBuffer;
	switch (cycle) {
	case SET_YEAR:
		snprintf(screenBuffer.line2, LCD_LINE_SIZE + 1, "[20%02d]/ %02d / %02d",
				time.year, time.month, time.day);
		snprintf(screenBuffer.line3, LCD_LINE_SIZE + 1, "    %02d : %02d",
				time.hour, time.minute);
		break;
	case SET_MONTH:
		snprintf(screenBuffer.line2, LCD_LINE_SIZE + 1, "20%02d /[%02d]/ %02d",
				time.year, time.month, time.day);
		snprintf(screenBuffer.line3, LCD_LINE_SIZE + 1, "    %02d : %02d",
				time.hour, time.minute);
		break;
	case SET_DAY:
		snprintf(screenBuffer.line2, LCD_LINE_SIZE + 1, "20%02d / %02d /[%02d]",
				time.year, time.month, time.day);
		snprintf(screenBuffer.line3, LCD_LINE_SIZE + 1, "   %02d : %02d",
				time.hour, time.minute);
		break;
	case SET_HOUR:
		snprintf(screenBuffer.line2, LCD_LINE_SIZE + 1, "20%02d / %02d / %02d",
				time.year, time.month, time.day);
		snprintf(screenBuffer.line3, LCD_LINE_SIZE + 1, "   [%02d]: %02d",
				time.hour, time.minute);
		break;
	case SET_MINUTE:
		snprintf(screenBuffer.line2, LCD_LINE_SIZE + 1, "20%02d / %02d / %02d",
				time.year, time.month, time.day);
		snprintf(screenBuffer.line3, LCD_LINE_SIZE + 1, "   %02d :[%02d]",
				time.hour, time.minute);
		break;
	}
	LCD_Puts(0, 0, "  TIME SETTING  ");
	LCD_Puts(0, 1, screenBuffer.line2);
	LCD_Puts(0, 2, screenBuffer.line3);
	LCD_Puts(0,3, " ");
}

void screen_OptionMenu(optionScreen_e_t *optionIndex) {
	screenData screenBuffer;
	if (*optionIndex == measurement1HisList
			|| *optionIndex == measurement2HisList)
		;
	else if (*optionIndex <= minNoneOption)
		*optionIndex = minNoneOption + 1;
	else if (*optionIndex >= maxNoneOption)
		*optionIndex = maxNoneOption - 1;
	else
		;

	switch (*optionIndex) {
	case measurement1Setting:
		snprintf(screenBuffer.line2, LCD_LINE_SIZE + 1, "MEASUREMENT 1");
		snprintf(screenBuffer.line3, LCD_LINE_SIZE + 1, "SETTING");
		break;
	case measurement2Setting:
		snprintf(screenBuffer.line2, LCD_LINE_SIZE + 1, "MEASUREMENT 2");
		snprintf(screenBuffer.line3, LCD_LINE_SIZE + 1, "SETTING");
		break;
	case measurementHis:
		snprintf(screenBuffer.line2, LCD_LINE_SIZE + 1, "MEASUREMENT");
		snprintf(screenBuffer.line3, LCD_LINE_SIZE + 1, "HISTORY LIST");
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
		snprintf(screenBuffer.line2, LCD_LINE_SIZE + 1, "V;D;L;R;Z INPUT");
		snprintf(screenBuffer.line3, LCD_LINE_SIZE + 1, "");
		break;
	case timeSetting:
		snprintf(screenBuffer.line2, LCD_LINE_SIZE + 1, "TIME SETTING");
		snprintf(screenBuffer.line3, LCD_LINE_SIZE + 1, "");
		break;
	case showIP:
		snprintf(screenBuffer.line2, LCD_LINE_SIZE + 1, "IP ADDRESS");
		snprintf(screenBuffer.line3, LCD_LINE_SIZE + 1, "");
		break;
	default:
		break;
	}

	LCD_Clear();
	LCD_Puts(0,0, " ");
	LCD_Puts(0, 1, screenBuffer.line2);
	LCD_Puts(0, 2, screenBuffer.line3);
	LCD_Puts(0,3, " ");
}

void screen_showIP(wiz_NetInfo *netInfo) {
	screenData screenBuffer;
	snprintf(screenBuffer.line1, LCD_LINE_SIZE + 1, "IP ADDRESS");
	snprintf(screenBuffer.line2, LCD_LINE_SIZE + 1, "%03d.%03d.%02d.%02d",
			netInfo->ip[0], netInfo->ip[1], netInfo->ip[2], netInfo->ip[3]);
	LCD_Clear();
	LCD_Puts(0, 0, screenBuffer.line1);
	LCD_Puts(0, 1, screenBuffer.line2);
	LCD_Puts(0,2, " ");
	LCD_Puts(0,3, " ");
}

void screen_setVDRLZ(VDRLZ_Input VDRLZ, VDRLZ_CycleSet cycle) {
	screenData screenBuffer;
	uint32_t R = VDRLZ.R * 10;
	uint32_t Z = VDRLZ.Z * 10;

	switch (cycle) {
	case V_set:
		snprintf(screenBuffer.line2, LCD_LINE_SIZE + 1, "V=[%02lu]  D= %02lu",
				VDRLZ.V, VDRLZ.D);
		snprintf(screenBuffer.line3, LCD_LINE_SIZE + 1, "L= %02lu", VDRLZ.L);
		snprintf(screenBuffer.line4, LCD_LINE_SIZE + 1,
				"R= %01lu.%01lu  Z= %01lu.%01lu", R / 10, R % 10, Z / 10, Z % 10);
		break;
	case D_set:
		snprintf(screenBuffer.line2, LCD_LINE_SIZE + 1, "V= %02lu  D=[%02lu]",
				VDRLZ.V, VDRLZ.D);
		snprintf(screenBuffer.line3, LCD_LINE_SIZE + 1, "L= %02lu", VDRLZ.L);
		snprintf(screenBuffer.line4, LCD_LINE_SIZE + 1,
				"R= %01lu.%01lu  Z= %01lu.%01lu", R / 10, R % 10, Z / 10, Z % 10);
		break;
	case L_set:
		snprintf(screenBuffer.line2, LCD_LINE_SIZE + 1, "V= %02lu  D= %02lu",
				VDRLZ.V, VDRLZ.D);
		snprintf(screenBuffer.line3, LCD_LINE_SIZE + 1, "L=[%02lu]", VDRLZ.L);
		snprintf(screenBuffer.line4, LCD_LINE_SIZE + 1,
				"R= %01lu.%01lu  Z= %01lu.%01lu", R / 10, R % 10, Z / 10, Z % 10);
		break;
	case R_set:
		snprintf(screenBuffer.line2, LCD_LINE_SIZE + 1, "V= %02lu  D= %02lu",
				VDRLZ.V, VDRLZ.D);
		snprintf(screenBuffer.line3, LCD_LINE_SIZE + 1, "L= %02lu", VDRLZ.L);
		snprintf(screenBuffer.line4, LCD_LINE_SIZE + 1,
				"R=[%01lu.%01lu]  Z= %01lu.%01lu", R / 10, R % 10, Z / 10,
				Z % 10);
		break;
	case Z_set:
		snprintf(screenBuffer.line2, LCD_LINE_SIZE + 1, "V= %02lu  D= %02lu",
				VDRLZ.V, VDRLZ.D);
		snprintf(screenBuffer.line3, LCD_LINE_SIZE + 1, "L= %02lu", VDRLZ.L);
		snprintf(screenBuffer.line4, LCD_LINE_SIZE + 1,
				"R= %01lu.%01lu  Z=[%01lu.%01lu]", R / 10, R % 10, Z / 10,
				Z % 10);
		break;
	default:
		break;
	}
	LCD_Clear();
	LCD_Puts(0, 0, " ");
	LCD_Puts(0, 1, screenBuffer.line2);
	LCD_Puts(0, 2, screenBuffer.line3);
	LCD_Puts(0, 3, screenBuffer.line4);
}

static dataMeasure capData(dataMeasure input, uint8_t isShowAB) {
	if (!isShowAB) {
		input.coordinates.X =
				(input.coordinates.X < -999) ? -999 :
				(input.coordinates.X > 999) ? 999 : input.coordinates.X;
		input.coordinates.Y =
				(input.coordinates.Y < -999) ? -999 :
				(input.coordinates.Y > 999) ? 999 : input.coordinates.Y;
		input.coordinates.Z =
				(input.coordinates.Z < -999) ? -999 :
				(input.coordinates.Z > 999) ? 999 : input.coordinates.Z;
		input.coordinates.R =
				(input.coordinates.R < -999) ? -999 :
				(input.coordinates.R > 999) ? 999 : input.coordinates.R;
	}
	else
	{
		input.coordinates.aX =
				(input.coordinates.aX < -999) ? -999 :
				(input.coordinates.aX > 999) ? 999 : input.coordinates.aX;
		input.coordinates.aY =
				(input.coordinates.aY < -999) ? -999 :
				(input.coordinates.aY > 999) ? 999 : input.coordinates.aY;
	}
	return input;
}

