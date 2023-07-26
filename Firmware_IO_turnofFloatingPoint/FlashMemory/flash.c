/*
 * eeprom_emulate.c
 *
 *  Created on: Jun 21, 2023
 *      Author: PC
 */

#include "flash.h"
#include "string.h"

#define TOTAL_BYTE	60


static void FLASH_ReadEntireMemory(uint16_t *ReadBuf);
static void FLASH_Write(uint16_t *ReadBuf, uint32_t Addr);
static void updateDataCalib(uint16_t *ReadBuf, MeasureValue *data, uint8_t measurementIndex);
static void updateVDRLZ(uint16_t *ReadBuf, VDRLZ_Input *data);


static void FLASH_ReadEntireMemory(uint16_t *ReadBuf) {
	volatile uint16_t *words = (volatile uint16_t*) MEMORY_ADDR;

	for (uint16_t i = 0; i < TOTAL_BYTE/2; i++) {
		ReadBuf[i] = words[i];
	}
}

static void updateDataCalib(uint16_t *ReadBuf, MeasureValue *data, uint8_t measurementIndex) {
	uint16_t *temp;
	temp = (uint16_t*) data;
	uint8_t indexArray = 0;
	if (measurementIndex == 1)
	{
		indexArray = INDEX_DATA_CALIB_1 / 2;
	}
	else
	{
		indexArray = INDEX_DATA_CALIB_2 /2 ;
	}
	for (uint8_t i = 0; i < sizeof(MeasureValue) / 2; i++) {
		ReadBuf[indexArray + i] = temp[i];
	}
}

static void updateVDRLZ(uint16_t *ReadBuf, VDRLZ_Input *data) {
	uint16_t *temp;
	temp = (uint16_t*) data;

	for (uint8_t i = 0; i < sizeof(MeasureValue) / 2; i++) {
		ReadBuf[INDEX_VDRLZ/2 + i] = temp[i];
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

	if(data == NULL)
	{
		return;
	}

	Addr = MEMORY_ADDR;
	FLASH_ReadEntireMemory(ReadBuf);
	updateDataCalib(ReadBuf, data, measurementIndex);
	FLASH_Write(ReadBuf, Addr);
}


MeasureValue FLASH_ReadDataCalib(uint8_t measurementIndex)
{
	uint32_t Addr = 0;

	if (measurementIndex == 1) {
		Addr = MEMORY_ADDR + INDEX_DATA_CALIB_1;
	} else {
		Addr = MEMORY_ADDR + INDEX_DATA_CALIB_2;
	}
	return *(MeasureValue*) Addr;
}


void FLASH_WriteVDRLZ(VDRLZ_Input *data)
{
	uint32_t Addr = 0;
	uint16_t ReadBuf[(TOTAL_BYTE + 1) / 2];

	if(data == NULL)
	{
		return;
	}

	Addr = MEMORY_ADDR;
	FLASH_ReadEntireMemory(ReadBuf);
	updateVDRLZ(ReadBuf, data);
	FLASH_Write(ReadBuf, Addr);
}

VDRLZ_Input FLASH_ReadVDRLZ()
{
//	memcpy(readStruct, (VDRLZ_Input*)(MEMORY_ADDR + INDEX_VDRLZ), sizeof(VDRLZ_Input));
	return *(VDRLZ_Input*)(MEMORY_ADDR + INDEX_VDRLZ);
}
