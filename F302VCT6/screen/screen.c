/*
 * screen.c
 *
 *  Created on: Jun 27, 2023
 *      Author: Admin
 */
#include "stdio.h"
#include "stdint.h"
#include "lcd.h"
#include "screen.h"
#include "stdlib.h"
#ifdef CDC_DEBUG
#include "main.h" ///use for cdc debug
#endif //CDC_DEBUG
#include "rtc.h"


#define SHOW_AB_FLAG		1
#define NOT_SHOW_AB_FLAG	0

#define LCD_SNPRINTF(dst, format, ...)                                                                \
	do                                                                                                \
	{                                                                                                 \
		char __temp_buffer[32];                                                                       \
		int __len = snprintf(__temp_buffer, sizeof(__temp_buffer), format, ##__VA_ARGS__);            \
		if (__len > _LCD_COLS)                                                                        \
		{                                                                                             \
			snprintf(dst, _LCD_COLS + 1, "LCD SIZE ERROR"); /* Fallback string, null-terminated */   \
		}                                                                                             \
		else                                                                                          \
		{                                                                                             \
			snprintf(dst, _LCD_COLS + 1, format, ##__VA_ARGS__);                                      \
		}                                                                                             \
	} while (0)

static dataMeasure capData(dataMeasure input, uint8_t isShowAB);

void screen_DataMeasureType1(dataMeasure data, uint8_t setCalib,
							 uint8_t measIndex, uint8_t showHisFlag)
{
	LCD_Clear();
	screenData screenBuffer;

	if (!showHisFlag)
		LCD_SNPRINTF(screenBuffer.line1, "MEASUREMENT %01d",
					 measIndex);
	else
		LCD_SNPRINTF(screenBuffer.line1, "MEAS.%01d HISTORY",
					 measIndex);
	if (CALIBSET == setCalib)
	{
		data = capData(data, NOT_SHOW_AB_FLAG);
		LCD_SNPRINTF(screenBuffer.line2,
					 "20%02d/%02d/%02d %02d:%02d", data.time.year, data.time.month,
					 data.time.day, data.time.hour, data.time.minute);
		if (MEASUREALL == data.mode)
		{
			LCD_SNPRINTF(screenBuffer.line3,
						 "X=%s%01d.%02d  Y=%s%01d.%02d",
						 (data.coordinates.X >= 0) ? "+" : "-",
						 (int16_t)(abs(data.coordinates.X) / 100), abs(data.coordinates.X) % 100,
						 (data.coordinates.Y >= 0) ? "+" : "-",
						 (int16_t)(abs(data.coordinates.Y) / 100), abs(data.coordinates.Y) % 100);
			LCD_SNPRINTF(screenBuffer.line4,
						 "Z=%s%01d.%02d  R=%s%01d.%02d",
						 (data.coordinates.Z >= 0) ? "+" : "-",
						 (int16_t)(abs(data.coordinates.Z) / 100), abs(data.coordinates.Z) % 100,
						 (data.coordinates.R >= 0) ? "+" : "-",
						 (int16_t)(abs(data.coordinates.R) / 100), abs(data.coordinates.R) % 100);
			DBG("LCD - MEASUREALL\n");
		}
		else if (ZERROR1 == data.mode)
		{
			LCD_SNPRINTF(screenBuffer.line3, "X=.....  Y=.....");
			LCD_SNPRINTF(screenBuffer.line4, "Z=.....  R=.....");
			DBG("LCD - ZERROR1\n");
		}
		else if (ZERROR2 == data.mode)
		{
			LCD_SNPRINTF(screenBuffer.line3,
						 "X=%s%01d.%02d  Y=%s%01d.%02d",
						 (data.coordinates.X >= 0) ? "+" : "-",
						 (int16_t)(abs(data.coordinates.X) / 100), abs(data.coordinates.X) % 100,
						 (data.coordinates.Y >= 0) ? "+" : "-",
						 (int16_t)(abs(data.coordinates.Y) / 100), abs(data.coordinates.Y) % 100);
			LCD_SNPRINTF(screenBuffer.line4,
						 "Z=.....  R=%s%01d.%02d", (data.coordinates.R >= 0) ? "+" : "-",
						 (int16_t)(abs(data.coordinates.R) / 100), abs(data.coordinates.R) % 100);
			DBG("LCD - ZERROR2\n");
		}
		else if (ZONLY == data.mode)
		{
			LCD_SNPRINTF(screenBuffer.line3, "X=.....  Y=.....");
			LCD_SNPRINTF(screenBuffer.line4,
						 "Z=%s%01d.%02d  R=.....", (data.coordinates.Z >= 0) ? "+" : "-",
						 (int16_t)(abs(data.coordinates.Z) / 100), abs(data.coordinates.Z) % 100);
			DBG("LCD - ZONLY\n");
		}
		else
		{
			LCD_SNPRINTF(screenBuffer.line2, "              ");
			LCD_SNPRINTF(screenBuffer.line3, "   No Data...!");
			LCD_SNPRINTF(screenBuffer.line4, "              ");
			DBG("LCD - No data\n");
		}
	}
	else
	{
		data.time = rtc_Now();
		LCD_SNPRINTF(screenBuffer.line2,
					 "20%02d/%02d/%02d %02d:%02d", data.time.year, data.time.month,
					 data.time.day, data.time.hour, data.time.minute);
		LCD_SNPRINTF(screenBuffer.line3, "X=.....  Y=.....");
		LCD_SNPRINTF(screenBuffer.line4, "Z=.....  R=.....");
	}

	LCD_Puts(0, 0, screenBuffer.line1);
	LCD_Puts(0, 1, screenBuffer.line2);
	LCD_Puts(0, 2, screenBuffer.line3);
	LCD_Puts(0, 3, screenBuffer.line4);
}

void screen_DataMeasureType2(dataMeasure data, uint8_t setCalib,
							 uint8_t measIndex, uint8_t showHisFlag)
{
	LCD_Clear();
	static screenData screenBuffer;
	if (!showHisFlag)
		LCD_SNPRINTF(screenBuffer.line1, "MEASUREMENT %01d",
					 measIndex);
	else
		LCD_SNPRINTF(screenBuffer.line1, "MEAS.%01d HISTORY",
					 measIndex);
	if (CALIBSET == setCalib)
	{

		LCD_SNPRINTF(screenBuffer.line2,
					 "20%02d/%02d/%02d %02d:%02d", data.time.year, data.time.month,
					 data.time.day, data.time.hour, data.time.minute);
		if (MEASUREALL == data.mode || data.mode == ZERROR2)
		{
			data = capData(data, SHOW_AB_FLAG);
			LCD_SNPRINTF(screenBuffer.line3, "   A=%s%2d.%01d",
						 (data.coordinates.aX >= 0) ? "+" : "-",
						 (int16_t)(abs(data.coordinates.aX) / 10), abs(data.coordinates.aX) % 10);
			LCD_SNPRINTF(screenBuffer.line4, "   B=%s%2d.%01d",
						 (data.coordinates.aY >= 0) ? "+" : "-",
						 (int16_t)(abs(data.coordinates.aY) / 10), abs(data.coordinates.aY) % 10);
			screenBuffer.line3[10] = 0xDF;
			screenBuffer.line4[10] = 0xDF;
		}
		else if (ZERROR1 == data.mode || data.mode == ZONLY)
		{

			LCD_SNPRINTF(screenBuffer.line3, "    A=.....");
			LCD_SNPRINTF(screenBuffer.line4, "    B=.....");
		}
		else
		{
			LCD_SNPRINTF(screenBuffer.line2, "                ");
			LCD_SNPRINTF(screenBuffer.line3, "   No Data...!  ");
			LCD_SNPRINTF(screenBuffer.line4, "                ");
		}
	}
	else
	{
		data.time = rtc_Now();
		LCD_SNPRINTF(screenBuffer.line2,
					 "20%02d/%02d/%02d %02d:%02d", data.time.year, data.time.month,
					 data.time.day, data.time.hour, data.time.minute);
		LCD_SNPRINTF(screenBuffer.line3, "    A=.....");
		LCD_SNPRINTF(screenBuffer.line4, "    B=.....");
	}
	LCD_Puts(0, 0, screenBuffer.line1);
	LCD_Puts(0, 1, screenBuffer.line2);
	LCD_Puts(0, 2, screenBuffer.line3);
	LCD_Puts(0, 3, screenBuffer.line4);
}

void screen_Time(Time time)
{
	screenData screenBuffer;
	LCD_SNPRINTF(screenBuffer.line2, "  20%02d/%02d/%02d",
				 time.year, time.month, time.day);
	LCD_SNPRINTF(screenBuffer.line3, "    %02d:%02d", time.hour,
				 time.minute);

	LCD_Puts(0, 0, "  TIME SETTING");
	LCD_Puts(0, 1, screenBuffer.line2);
	LCD_Puts(0, 2, screenBuffer.line3);
	LCD_Puts(0, 3, " ");
}

void screen_setDateTime(Time time, CycleTime cycle)
{
	LCD_Clear();
	screenData screenBuffer;
	switch (cycle)
	{
	case SET_YEAR:
		LCD_SNPRINTF(screenBuffer.line2, "[20%02d]/ %02d / %02d",
					 time.year, time.month, time.day);
		LCD_SNPRINTF(screenBuffer.line3, "    %02d : %02d",
					 time.hour, time.minute);
		break;
	case SET_MONTH:
		LCD_SNPRINTF(screenBuffer.line2, " 20%02d /[%02d]/ %02d",
					 time.year, time.month, time.day);
		LCD_SNPRINTF(screenBuffer.line3, "    %02d : %02d",
					 time.hour, time.minute);
		break;
	case SET_DAY:
		LCD_SNPRINTF(screenBuffer.line2, " 20%02d / %02d /[%02d]",
					 time.year, time.month, time.day);
		LCD_SNPRINTF(screenBuffer.line3, "    %02d : %02d",
					 time.hour, time.minute);
		break;
	case SET_HOUR:
		LCD_SNPRINTF(screenBuffer.line2, " 20%02d / %02d / %02d",
					 time.year, time.month, time.day);
		LCD_SNPRINTF(screenBuffer.line3, "   [%02d]: %02d",
					 time.hour, time.minute);
		break;
	case SET_MINUTE:
		LCD_SNPRINTF(screenBuffer.line2, " 20%02d / %02d / %02d",
					 time.year, time.month, time.day);
		LCD_SNPRINTF(screenBuffer.line3, "    %02d :[%02d]",
					 time.hour, time.minute);
		break;
	}
	LCD_Puts(0, 0, "  TIME SETTING  ");
	LCD_Puts(0, 1, screenBuffer.line2);
	LCD_Puts(0, 2, screenBuffer.line3);
	LCD_Puts(0, 3, " ");
}

void screen_OptionMenu(optionScreen_e_t *optionIndex)
{
	screenData screenBuffer;
	if (*optionIndex == measurement1HisList || *optionIndex == measurement2HisList)
		;
	else if (*optionIndex <= minNoneOption)
		*optionIndex = minNoneOption + 1;
	else if (*optionIndex >= maxNoneOption)
		*optionIndex = maxNoneOption - 1;
	else
		;

	switch (*optionIndex)
	{
	case measurement1Setting:
		LCD_SNPRINTF(screenBuffer.line2, "MEASUREMENT 1");
		LCD_SNPRINTF(screenBuffer.line3, "SETTING");
		break;
	case measurement2Setting:
		LCD_SNPRINTF(screenBuffer.line2, "MEASUREMENT 2");
		LCD_SNPRINTF(screenBuffer.line3, "SETTING");
		break;
	case measurementHis:
		LCD_SNPRINTF(screenBuffer.line2, "MEASUREMENT");
		LCD_SNPRINTF(screenBuffer.line3, "HISTORY LIST");
		break;
	case measurement1HisList:
		LCD_SNPRINTF(screenBuffer.line2, "MEASUREMENT 1");
		LCD_SNPRINTF(screenBuffer.line3, "HISTORY LIST");
		break;
	case measurement2HisList:
		LCD_SNPRINTF(screenBuffer.line2, "MEASUREMENT 2");
		LCD_SNPRINTF(screenBuffer.line3, "HISTORY LIST");
		break;
	case VDLRZinput:
		LCD_SNPRINTF(screenBuffer.line2, "V;D;L;R;Z INPUT");
		LCD_SNPRINTF(screenBuffer.line3, "");
		break;
	case timeSetting:
		LCD_SNPRINTF(screenBuffer.line2, "TIME SETTING");
		LCD_SNPRINTF(screenBuffer.line3, "");
		break;
	case showIP:
		LCD_SNPRINTF(screenBuffer.line2, "IP ADDRESS");
		LCD_SNPRINTF(screenBuffer.line3, "");
		break;
	default:
		break;
	}

	LCD_Clear();
	LCD_Puts(0, 0, " ");
	LCD_Puts(0, 1, screenBuffer.line2);
	LCD_Puts(0, 2, screenBuffer.line3);
	LCD_Puts(0, 3, " ");
}

void screen_showIP(wiz_NetInfo *netInfo)
{
	screenData screenBuffer;
	LCD_SNPRINTF(screenBuffer.line1, "IP ADDRESS");
	LCD_SNPRINTF(screenBuffer.line2, "%03d.%03d.%02d.%02d",
				 netInfo->ip[0], netInfo->ip[1], netInfo->ip[2], netInfo->ip[3]);
	LCD_Clear();
	LCD_Puts(0, 0, screenBuffer.line1);
	LCD_Puts(0, 1, screenBuffer.line2);
	LCD_Puts(0, 2, " ");
	LCD_Puts(0, 3, " ");
}

void screen_setVDRLZ(VDRLZ_Input VDRLZ, VDRLZ_CycleSet cycle)
{
	screenData screenBuffer;
	uint32_t R = VDRLZ.R * 10;
	uint32_t Z = VDRLZ.Z * 10;

	switch (cycle)
	{
	case V_set:
		LCD_SNPRINTF(screenBuffer.line2, "V=[%02lu]   D= %02lu",
					 VDRLZ.V, VDRLZ.D);
		LCD_SNPRINTF(screenBuffer.line3, "L= %02lu", VDRLZ.L);
		LCD_SNPRINTF(screenBuffer.line4,
					 "R= %01lu.%01lu   Z= %01lu.%01lu", R / 10, R % 10, Z / 10, Z % 10);
		break;
	case D_set:
		LCD_SNPRINTF(screenBuffer.line2, "V= %02lu    D=[%02lu]",
					 VDRLZ.V, VDRLZ.D);
		LCD_SNPRINTF(screenBuffer.line3, "L= %02lu", VDRLZ.L);
		LCD_SNPRINTF(screenBuffer.line4,
					 "R= %01lu.%01lu   Z= %01lu.%01lu", R / 10, R % 10, Z / 10, Z % 10);
		break;
	case L_set:
		LCD_SNPRINTF(screenBuffer.line2, "V= %02lu    D= %02lu",
					 VDRLZ.V, VDRLZ.D);
		LCD_SNPRINTF(screenBuffer.line3, "L=[%02lu]", VDRLZ.L);
		LCD_SNPRINTF(screenBuffer.line4,
					 "R= %01lu.%01lu   Z= %01lu.%01lu", R / 10, R % 10, Z / 10, Z % 10);
		break;
	case R_set:
		LCD_SNPRINTF(screenBuffer.line2, "V= %02lu    D= %02lu",
					 VDRLZ.V, VDRLZ.D);
		LCD_SNPRINTF(screenBuffer.line3, "L= %02lu", VDRLZ.L);
		LCD_SNPRINTF(screenBuffer.line4,
					 "R=[%01lu.%01lu]  Z= %01lu.%01lu", R / 10, R % 10, Z / 10,
					 Z % 10);
		break;
	case Z_set:
		LCD_SNPRINTF(screenBuffer.line2, "V= %02lu    D= %02lu",
					 VDRLZ.V, VDRLZ.D);
		LCD_SNPRINTF(screenBuffer.line3, "L= %02lu", VDRLZ.L);
		LCD_SNPRINTF(screenBuffer.line4,
					 "R= %01lu.%01lu   Z=[%01lu.%01lu]", R / 10, R % 10, Z / 10,
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

static dataMeasure capData(dataMeasure input, uint8_t isShowAB)
{
	if (!isShowAB)
	{
		input.coordinates.X =
			(input.coordinates.X < -999) ? -999 : (input.coordinates.X > 999) ? 999
																			  : input.coordinates.X;
		input.coordinates.Y =
			(input.coordinates.Y < -999) ? -999 : (input.coordinates.Y > 999) ? 999
																			  : input.coordinates.Y;
		input.coordinates.Z =
			(input.coordinates.Z < -999) ? -999 : (input.coordinates.Z > 999) ? 999
																			  : input.coordinates.Z;
		input.coordinates.R =
			(input.coordinates.R < -999) ? -999 : (input.coordinates.R > 999) ? 999
																			  : input.coordinates.R;
	}
	else
	{
		input.coordinates.aX =
			(input.coordinates.aX < -999) ? -999 : (input.coordinates.aX > 999) ? 999
																				: input.coordinates.aX;
		input.coordinates.aY =
			(input.coordinates.aY < -999) ? -999 : (input.coordinates.aY > 999) ? 999
																				: input.coordinates.aY;
	}
	return input;
}

void screen_waitMeasurement(uint8_t measIndex)
{
	screenData screenBuffer;
	Time __time = rtc_Now();
	LCD_Clear();
	LCD_SNPRINTF(screenBuffer.line1, "MEASUREMENT %01d",
				 measIndex);
	LCD_SNPRINTF(screenBuffer.line2,
				 "20%02d/%02d/%02d %02d:%02d", __time.year, __time.month,
				 __time.day, __time.hour, __time.minute);
	LCD_SNPRINTF(screenBuffer.line3, "X=.....  Y=.....");
	LCD_SNPRINTF(screenBuffer.line4, "Z=.....  R=.....");
	LCD_Puts(0, 0, screenBuffer.line1);
	LCD_Puts(0, 1, screenBuffer.line2);
	LCD_Puts(0, 2, screenBuffer.line3);
	LCD_Puts(0, 3, screenBuffer.line4);
}

void screen_errorXY(uint8_t measIndex)
{
	screenData screenBuffer;
	Time __time = rtc_Now();
	if (__time.year > 99 || __time.month > 12 || __time.day > 31 || __time.hour > 23 || __time.minute > 59)
	{
		__time.year = 11;
		__time.month = 1;
		__time.day = 1;
		__time.hour = 11;
		__time.minute = 11;
	}

	LCD_Clear();
	LCD_SNPRINTF(screenBuffer.line1, "MEASUREMENT %01d",
				 measIndex);
	LCD_SNPRINTF(screenBuffer.line2,
				 "20%02d/%02d/%02d %02d:%02d", __time.year, __time.month, __time.day,
				 __time.hour, __time.minute);
	LCD_SNPRINTF(screenBuffer.line3, "   SENSOR X/Y");
	LCD_SNPRINTF(screenBuffer.line4, "     ERROR!");
	LCD_Puts(0, 0, screenBuffer.line1);
	LCD_Puts(0, 1, screenBuffer.line2);
	LCD_Puts(0, 2, screenBuffer.line3);
	LCD_Puts(0, 3, screenBuffer.line4);
}
