/*
 * unittest.c
 *
 *  Created on: Jul 28, 2023
 *      Author: Admin
 */


#include "unittest.h"



void unitTestZ()
{
	mmeasureValue.Z = 26500;
//	mcalibValue.Z = 110;

	app_CalculatorValue(WAITMEASUREX1Y1, ZONLY, 1);
}
