/*
 * screen.c
 *
 *  Created on: Jun 27, 2023
 *      Author: Admin
 */
#include "stdio.h"
#include "LCD.h"
#include "screen.h"

#include "main.h" ///use for cdc debug

static screenData screenBuffer;

void screen_DataMeasureType1(dataMeasure data, uint8_t setCalib, uint8_t measIndex, uint8_t showHisFlag) {
	LCD_Clear();
	if(!showHisFlag)
		snprintf(screenBuffer.line1, LCD_LINE_SIZE + 1, "MEASUREMENT %01d",
						measIndex);
	else
		snprintf(screenBuffer.line1, LCD_LINE_SIZE + 1, "MEAS.%01d HISTORY",
							measIndex);
	if (CALIBSET == setCalib) {
		snprintf(screenBuffer.line2, LCD_LINE_SIZE + 1,	"20%02d-%02d-%02d% 02d:%02d", data.time.year, data.time.month,
				data.time.day, data.time.hour, data.time.minute);
		if (MEASUREALL == data.mode) {
			snprintf(screenBuffer.line3, LCD_LINE_SIZE + 1,
					"X=%+-#1.2f  Y=%+-#1.2f", (float) data.coordinates.X / 100,
					(float) data.coordinates.Y / 100);
			snprintf(screenBuffer.line4, LCD_LINE_SIZE + 1,
					"Z=%+-#1.2f  R=%+-#1.2f", (float) data.coordinates.Z / 100,
					(float) data.coordinates.R / 100);
			DBG("LCD - MESUREALL\n");
		} else if (ZERROR1 == data.mode) {
			snprintf(screenBuffer.line3, LCD_LINE_SIZE + 1, "X=....   Y=.... ");
			snprintf(screenBuffer.line4, LCD_LINE_SIZE + 1, "Z=....   R=.... ");
			DBG("LCD - ZERROR1\n");
		} else if (ZERROR2 == data.mode) {
			snprintf(screenBuffer.line3, LCD_LINE_SIZE + 1,
					"X=%+-#1.2f  Y=%+-#1.2f", (float) data.coordinates.X / 100,
					(float) data.coordinates.Y / 100);
			snprintf(screenBuffer.line4, LCD_LINE_SIZE + 1,
					"Z=....  R=%+-#1.2f", (float) data.coordinates.R / 100);
			DBG("LCD - ZERROR2\n");
		} else if (ZONLY == data.mode) {
			snprintf(screenBuffer.line3, LCD_LINE_SIZE + 1, "X=....   Y=.... ");
			snprintf(screenBuffer.line4, LCD_LINE_SIZE + 1,
					"Z=%+-#1.2f  R=.... ", (float) data.coordinates.Z / 100);
			DBG("LCD - ZONLY\n");
		} else {
			snprintf(screenBuffer.line2, LCD_LINE_SIZE + 1, "                ");
			snprintf(screenBuffer.line3, LCD_LINE_SIZE + 1, "   No Data...!  ");
			snprintf(screenBuffer.line4, LCD_LINE_SIZE + 1, "                ");
			DBG("LCD - No data\n");
		}
	}
	else
	{
		snprintf(screenBuffer.line2, LCD_LINE_SIZE + 1, "....-..-.. ..:..");
		snprintf(screenBuffer.line3, LCD_LINE_SIZE + 1, "X=....   Y=.... ");
		snprintf(screenBuffer.line4, LCD_LINE_SIZE + 1, "Z=....   R=.... ");
	}
	LCD_Puts(0, 0, screenBuffer.line1);
	LCD_Puts(0, 1, screenBuffer.line2);
	LCD_Puts(0, 2, screenBuffer.line3);
	LCD_Puts(0, 3, screenBuffer.line4);
}

void screen_DataMeasureType2(dataMeasure data, uint8_t setCalib,
		uint8_t measIndex, uint8_t showHisFlag) {
	LCD_Clear();
	if (CALIBSET == setCalib) {
		if(!showHisFlag)
			snprintf(screenBuffer.line1, LCD_LINE_SIZE + 1, "MEASUREMENT %01d",
							measIndex);
		else
			snprintf(screenBuffer.line1, LCD_LINE_SIZE + 1, "MEAS.%01d HISTORY",
								measIndex);

		snprintf(screenBuffer.line2, LCD_LINE_SIZE + 1,
				"20%02d-%02d-%02d %02d:%02d", data.time.year, data.time.month,
				data.time.day, data.time.hour, data.time.minute);
		if (MEASUREALL == data.mode || data.mode == ZERROR2) {
			snprintf(screenBuffer.line3, LCD_LINE_SIZE + 1, "A=%+-#2.1f",
					(float) data.coordinates.aX / 10);
			snprintf(screenBuffer.line4, LCD_LINE_SIZE + 1, "B=%+-#2.1f",
					(float) data.coordinates.aY / 10);
			screenBuffer.line3[7] = 0xDF;
			screenBuffer.line4[7] = 0xDF;
		} else if (ZERROR1 == data.mode || data.mode == ZONLY) {

			snprintf(screenBuffer.line3, LCD_LINE_SIZE + 1, "A=....");
			snprintf(screenBuffer.line4, LCD_LINE_SIZE + 1, "B=....");
		} else {
            snprintf(screenBuffer.line2,LCD_LINE_SIZE+1,"                ");
            snprintf(screenBuffer.line3,LCD_LINE_SIZE+1,"   No Data...!  ");
            snprintf(screenBuffer.line4,LCD_LINE_SIZE+1,"                ");
		}
	}
	else
	{
		snprintf(screenBuffer.line1, LCD_LINE_SIZE + 1, " ");
		snprintf(screenBuffer.line2, LCD_LINE_SIZE + 1, "....-..-.. ..:..");
		snprintf(screenBuffer.line3, LCD_LINE_SIZE + 1, "A=....");
		snprintf(screenBuffer.line4, LCD_LINE_SIZE + 1, "B=....");
	}
	LCD_Puts(0, 0, screenBuffer.line1);
	LCD_Puts(0, 1, screenBuffer.line2);
	LCD_Puts(0, 2, screenBuffer.line3);
	LCD_Puts(0, 3, screenBuffer.line4);
}

void screen_Time(Time time) {
	snprintf(screenBuffer.line2, LCD_LINE_SIZE + 1, "  20%02d-%02d-%02d",
			time.year, time.month, time.day);
	snprintf(screenBuffer.line3, LCD_LINE_SIZE + 1, "    %02d:%02d",
			time.hour, time.minute);

	LCD_Puts(0, 0, "  TIME SETTING");
	LCD_Puts(0, 1, screenBuffer.line2);
	LCD_Puts(0, 2, screenBuffer.line3);
}

void screen_setDateTime(Time time, CycleTime cycle) {
	LCD_Clear();
	switch (cycle) {
	case SET_YEAR:
		snprintf(screenBuffer.line2, LCD_LINE_SIZE + 1,
				"[20%02d]-%02d-%02d", time.year, time.month, time.day);
		snprintf(screenBuffer.line3, LCD_LINE_SIZE + 1, "   %02d:%02d",
				time.hour, time.minute);
		break;
	case SET_MONTH:
		snprintf(screenBuffer.line2, LCD_LINE_SIZE + 1,
				"20%02d-[%02d]-%02d", time.year, time.month, time.day);
		snprintf(screenBuffer.line3, LCD_LINE_SIZE + 1, "   %02d:%02d",
				time.hour, time.minute);
		break;
	case SET_DAY:
		snprintf(screenBuffer.line2, LCD_LINE_SIZE + 1,
				"20%02d-%02d-[%02d]", time.year, time.month, time.day);
		snprintf(screenBuffer.line3, LCD_LINE_SIZE + 1, "   %02d:%02d",
				time.hour, time.minute);
		break;
	case SET_HOUR:
		snprintf(screenBuffer.line2, LCD_LINE_SIZE + 1,
				"20%02d-%02d-%02d", time.year, time.month, time.day);
		snprintf(screenBuffer.line3, LCD_LINE_SIZE + 1, "   [%02d]:%02d",
				time.hour, time.minute);
		break;
	case SET_MINUTE:
		snprintf(screenBuffer.line2, LCD_LINE_SIZE + 1,
				"20%02d-%02d-%02d", time.year, time.month, time.day);
		snprintf(screenBuffer.line3, LCD_LINE_SIZE + 1, "   %02d:[%02d]",
				time.hour, time.minute);
		break;
	}
	LCD_Puts(0, 0, "  TIME SETTING  ");
	LCD_Puts(0, 1, screenBuffer.line2);
	LCD_Puts(0, 2, screenBuffer.line3);
}

void screen_OptionMenu(optionScreen_e_t *optionIndex) {
	if (*optionIndex == measurement1HisList || *optionIndex == measurement2HisList)
		;
	else if (*optionIndex <= minNoneOption)
		*optionIndex = minNoneOption + 1;
	else if	(*optionIndex >= maxNoneOption)
		*optionIndex = maxNoneOption - 1;
	else
		;

	switch (*optionIndex) {
	case measurement1Setting:
		snprintf(screenBuffer.line2, LCD_LINE_SIZE + 1,"MEASUREMENT 1");
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
		snprintf(screenBuffer.line2, LCD_LINE_SIZE + 1,"V;D;L;R;Z INPUT");
		snprintf(screenBuffer.line3, LCD_LINE_SIZE + 1, "%s", "");
		break;
	case timeSetting:
		snprintf(screenBuffer.line2, LCD_LINE_SIZE + 1,"TIME SETTING");
		snprintf(screenBuffer.line3, LCD_LINE_SIZE + 1, "%s", "");
		break;
	case showIP:
		snprintf(screenBuffer.line2, LCD_LINE_SIZE + 1, "IP ADDRESS");
		snprintf(screenBuffer.line3, LCD_LINE_SIZE + 1, "%s", "");
		break;
	default:
		break;
	}

	LCD_Clear();
	LCD_Puts(0, 1, screenBuffer.line2);
	LCD_Puts(0, 2, screenBuffer.line3);
}

void screen_showIP(wiz_NetInfo *netInfo)
{
	snprintf(screenBuffer.line1, LCD_LINE_SIZE + 1, "IP ADDRESS");
	snprintf(screenBuffer.line2, LCD_LINE_SIZE + 1, "%03d.%03d.%02d.%02d",netInfo->ip[0], netInfo->ip[1], netInfo->ip[2], netInfo->ip[3]);
	LCD_Clear();
	LCD_Puts(0, 0, screenBuffer.line1);
	LCD_Puts(0, 1, screenBuffer.line2);
}

void screen_setVDRLZ(VDRLZ_Input VDRLZ, VDRLZ_CycleSet cycle) {
	switch (cycle) {
	case V_set:
		snprintf(screenBuffer.line2, LCD_LINE_SIZE +1, "V=[%2.1f] D=%2.1f", VDRLZ.V, VDRLZ.D);
		snprintf(screenBuffer.line3, LCD_LINE_SIZE +1, "L=%2.1f", VDRLZ.L);
		snprintf(screenBuffer.line4, LCD_LINE_SIZE +1, "R=%2.1f Z=%2.1f", VDRLZ.R, VDRLZ.Z);
		break;
	case D_set:
		snprintf(screenBuffer.line2, LCD_LINE_SIZE +1, "V=%2.1f D=[%2.1f]", VDRLZ.V, VDRLZ.D);
		snprintf(screenBuffer.line3, LCD_LINE_SIZE +1, "L=%2.1f", VDRLZ.L);
		snprintf(screenBuffer.line4, LCD_LINE_SIZE +1, "R=%2.1f Z=%2.1f", VDRLZ.R, VDRLZ.Z);
		break;
	case L_set:
		snprintf(screenBuffer.line2, LCD_LINE_SIZE +1, "V=%2.1f D=%2.1f", VDRLZ.V, VDRLZ.D);
		snprintf(screenBuffer.line3, LCD_LINE_SIZE +1, "L=[%2.1f]", VDRLZ.L);
		snprintf(screenBuffer.line4, LCD_LINE_SIZE +1, "R=%2.1f Z=%2.1f", VDRLZ.R, VDRLZ.Z);
		break;
	case R_set:
		snprintf(screenBuffer.line2, LCD_LINE_SIZE +1, "V=%2.1f D=%2.1f", VDRLZ.V, VDRLZ.D);
		snprintf(screenBuffer.line3, LCD_LINE_SIZE +1, "L=%2.1f", VDRLZ.L);
		snprintf(screenBuffer.line4, LCD_LINE_SIZE +1, "[R=%2.1f] Z=%2.1f", VDRLZ.R, VDRLZ.Z);
		break;
	case Z_set:
		snprintf(screenBuffer.line2, LCD_LINE_SIZE +1, "V=%2.1f D=%2.1f", VDRLZ.V, VDRLZ.D);
		snprintf(screenBuffer.line3, LCD_LINE_SIZE +1, "L=%2.1f", VDRLZ.L);
		snprintf(screenBuffer.line4, LCD_LINE_SIZE +1, "R=%2.1f Z=[%2.1f]", VDRLZ.R, VDRLZ.Z);
		break;
	default:
		break;
	}
	LCD_Clear();
	LCD_Puts(0,0," ");
	LCD_Puts(0, 1, screenBuffer.line2);
	LCD_Puts(0, 2, screenBuffer.line3);
	LCD_Puts(0, 3, screenBuffer.line4);
}
