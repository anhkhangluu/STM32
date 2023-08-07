/*
 * rtc.h
 *
 *  Created on: Jun 27, 2023
 *      Author: Admin
 */

#ifndef RTC_H_
#define RTC_H_

#include "structer.h"
#include "main.h"

Time rtc_Now(void);
void rtc_SetDateTime(Time time);


#endif /* RTC_H_ */
