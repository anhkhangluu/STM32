/*
 * rtc.c
 *
 *  Created on: Jun 27, 2023
 *      Author: Admin
 */
#include "rtc.h"

void rtc_SetDateTime(Time time) {
	RTC_TimeTypeDef sTime;
	RTC_DateTypeDef sDate;
	sTime.Hours = time.hour; // set hours
	sTime.Minutes = time.minute; // set minutes
	sTime.Seconds = time.second; // set seconds
	sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	sTime.StoreOperation = RTC_STOREOPERATION_RESET;
	if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK) {
//		_Error_Handler(__FILE__, __LINE__);
		return;
	}
	sDate.WeekDay = RTC_WEEKDAY_MONDAY; //weekday - don't care
	sDate.Month = time.month; // month
	sDate.Date = time.day; // date
	sDate.Year = time.year; // year
	if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK) {
//		_Error_Handler(__FILE__, __LINE__);
		return;
	}
	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0x32F2); // backup register


	/* Using in main for check the RTC is still correct or not
	   if(HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1) != 0x32F2)
      {
      //   Set the time
        set_time();
      }
	 * */
}

Time rtc_Now() {
	Time time;
	RTC_DateTypeDef gDate;
	RTC_TimeTypeDef gTime;
	/* Get the RTC current Time */
	HAL_RTC_GetTime(&hrtc, &gTime, RTC_FORMAT_BIN);
	/* Get the RTC current Date */
	HAL_RTC_GetDate(&hrtc, &gDate, RTC_FORMAT_BIN);
	time.day = gDate.Date;
	time.month = gDate.Month;
	time.year = gDate.Year;

	time.hour = gTime.Hours;
	time.minute = gTime.Minutes;
	time.second = gTime.Seconds;
	return time;
}
