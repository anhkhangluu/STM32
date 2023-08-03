/*
 * unittest.c
 *
 *  Created on: Jul 28, 2023
 *      Author: Admin
 */


#include "unittest.h"



void unitTestZ()
{
	dataMeasure data = {0};

	mmeasureValue.Z = 249249;
	mcalibValue.Z = 249603;

	app_CalculatorValue(WAITMEASUREX1Y1, ZONLY, 1);
}
