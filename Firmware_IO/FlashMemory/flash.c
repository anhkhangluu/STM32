/*
 * eeprom_emulate.c
 *
 *  Created on: Jun 21, 2023
 *      Author: PC
 */

#include "flash.h"
#define TOTAL_BYTE	241

static void updateHistoryDataMeasure(uint16_t *ReadBuf, dataMeasure *data,
		uint16_t hisIndex);
static void updateCurrentIndex(uint16_t *ReadBuf, uint16_t index);
static void FLASH_ReadEntireMeasurement1(uint16_t *ReadBuf);
static void FLASH_ReadEntireMeasurement2(uint16_t *ReadBuf);
static void FLASH_Write(uint16_t *ReadBuf, uint32_t Addr);
static void updateDataCalib(uint16_t *ReadBuf, MeasureValue *data);
static void updateDataCurrent(uint16_t *ReadBuf, MeasureValue *data);


void FLASH_WriteCurrentIndex(uint16_t index, uint8_t measurementIndex) {
	uint32_t Addr = 0;
	uint16_t ReadBuf[(TOTAL_BYTE + 1) / 2];

	if (measurementIndex == 1) {
		Addr = MEASUREMENT_1_START_ADDR + FLASH_PAGE_CURRENT_INDEX;
		FLASH_ReadEntireMeasurement1(ReadBuf);
	} else {
		Addr = MEASUREMENT_2_START_ADDR + FLASH_PAGE_CURRENT_INDEX;
		FLASH_ReadEntireMeasurement2(ReadBuf);
	}
	updateCurrentIndex(ReadBuf, index);

	FLASH_Write(ReadBuf, Addr);
}

uint16_t FLASH_ReadCurrentIndex(uint8_t measurementIndex) {
	uint16_t index = 0;
	if (measurementIndex == 1) {
		index = *(volatile uint16_t*) (MEASUREMENT_1_START_ADDR
				+ FLASH_PAGE_CURRENT_INDEX);
	} else {
		index = *(volatile uint16_t*) (MEASUREMENT_2_START_ADDR
				+ FLASH_PAGE_CURRENT_INDEX);
	}
	return index;
}

void FLASH_WriteHistoryDataMeasure(dataMeasure *data, uint8_t measurementIndex,
		uint8_t hisIndex) {
	uint32_t Addr = 0;
	uint16_t ReadBuf[(TOTAL_BYTE + 1) / 2];

	if (measurementIndex == 1) {
		Addr = MEASUREMENT_1_START_ADDR + FLASH_PAGE_HISTORY;
		FLASH_ReadEntireMeasurement1(ReadBuf);
	} else {
		Addr = MEASUREMENT_2_START_ADDR + FLASH_PAGE_HISTORY;
		FLASH_ReadEntireMeasurement2(ReadBuf);
	}
	updateHistoryDataMeasure(ReadBuf, data, hisIndex);
	FLASH_Write(ReadBuf, Addr);
}

dataMeasure FLASH_ReadDataMeasure(uint32_t Addr) { ///////////////////// rewrite
	return *(dataMeasure*) Addr;
}

static void FLASH_ReadEntireMeasurement1(uint16_t *ReadBuf) {
	volatile uint16_t *words = (volatile uint16_t*) MEASUREMENT_1_START_ADDR;

	for (uint16_t i = 0; i < (TOTAL_BYTE + 1) / 2; i++) {
		ReadBuf[i] = words[i];
	}
}

static void FLASH_ReadEntireMeasurement2(uint16_t *ReadBuf) {
	volatile uint16_t *words = (volatile uint16_t*) MEASUREMENT_2_START_ADDR;

	for (uint16_t i = 0; i < (TOTAL_BYTE + 1) / 2; i++) {
		ReadBuf[i] = words[i];
	}
}

static void updateHistoryDataMeasure(uint16_t *ReadBuf, dataMeasure *data,
		uint16_t hisIndex) {
	uint16_t *temp;
	temp = (uint16_t*) data;
	for (uint8_t i = 0; i < sizeof(dataMeasure) / 2; i++) {
		ReadBuf[hisIndex * sizeof(dataMeasure) / 2 + FLASH_PAGE_HISTORY / 2 + i] =
				temp[i];
	}
}

static void updateCurrentIndex(uint16_t *ReadBuf, uint16_t index) {
	ReadBuf[FLASH_PAGE_CURRENT_INDEX] = index;
}

static void updateDataCalib(uint16_t *ReadBuf, MeasureValue *data) {
	uint16_t *temp;
	temp = (uint16_t*) data;
	for (uint8_t i = 0; i < sizeof(dataMeasure) / 2; i++) {
		ReadBuf[FLASH_PAGE_DATA_CALIB / 2 + i] = temp[i];
	}
}

static void updateDataCurrent(uint16_t *ReadBuf, MeasureValue *data) {
	uint16_t *temp;
	temp = (uint16_t*) data;
	for (uint8_t i = 0; i < sizeof(dataMeasure) / 2; i++) {
		ReadBuf[FLASH_PAGE_CURRENT_DATA/ 2 + i] = temp[i];
	}
}

static void FLASH_Write(uint16_t *ReadBuf, uint32_t Addr) {
	static FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t PAGEError;

	/* Unlock the Flash to enable the flash control register access *************/
	HAL_FLASH_Unlock();

	/* Fill EraseInit structure*/
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.PageAddress = Addr;
	EraseInitStruct.NbPages = 1;

	if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK) {
		/*Error occurred while page erase.*/
		return;
	}

	for (uint16_t i = 0; i < (TOTAL_BYTE + 1) / 2; i++)
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, Addr + i * 2, ReadBuf[i]);
	HAL_FLASH_Lock();
}

void FLASH_WriteDataCalib(MeasureValue *data, uint8_t measurementIndex)
{
	uint32_t Addr = 0;
	uint16_t ReadBuf[(TOTAL_BYTE + 1) / 2];

	if (measurementIndex == 1) {
		Addr = MEASUREMENT_1_START_ADDR + FLASH_PAGE_HISTORY;
		FLASH_ReadEntireMeasurement1(ReadBuf);
	} else {
		Addr = MEASUREMENT_2_START_ADDR + FLASH_PAGE_HISTORY;
		FLASH_ReadEntireMeasurement2(ReadBuf);
	}

	updateDataCalib(ReadBuf, data);
	FLASH_Write(ReadBuf, Addr);
}

void FLASH_WriteDataCurrent(MeasureValue *data, uint8_t measurementIndex)
{
	uint32_t Addr = 0;
	uint16_t ReadBuf[(TOTAL_BYTE + 1) / 2];

	if (measurementIndex == 1) {
		Addr = MEASUREMENT_1_START_ADDR + FLASH_PAGE_HISTORY;
		FLASH_ReadEntireMeasurement1(ReadBuf);
	} else {
		Addr = MEASUREMENT_2_START_ADDR + FLASH_PAGE_HISTORY;
		FLASH_ReadEntireMeasurement2(ReadBuf);
	}

	updateDataCurrent(ReadBuf, data);
	FLASH_Write(ReadBuf, Addr);
}
