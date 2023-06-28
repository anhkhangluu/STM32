/*
 * screen.h
 *
 *  Created on: Jun 27, 2023
 *      Author: Admin
 */

#ifndef SCREEN_H_
#define SCREEN_H_

#include "structer.h"

void screen_DataMeasure(dataMeasure data, uint8_t setCalib);
void screen_Time(Time time);

void screen_setDateTime(Time time, CycleTime cycle);




#endif /* SCREEN_H_ */
