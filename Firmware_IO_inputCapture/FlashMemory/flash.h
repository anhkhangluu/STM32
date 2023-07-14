#ifndef INC_EEPROM_EMULATE_H_
#define INC_EEPROM_EMULATE_H_

#include "stm32f0xx_hal.h"
#include "structer.h"


#define PAGE_NUM_ADDR(pageNum)	((uint32_t )FLASH_BASE + PAGESIZE * pageNum)

#define MEASUREMENT_1_START_ADDR		((uint32_t )0x0801F000)
#define MEASUREMENT_2_START_ADDR		((uint32_t )0x0801F800)

#define FLASH_PAGE_CURRENT_INDEX		0		//
#define FLASH_PAGE_CURRENT_DATA			2
#define FLASH_PAGE_DATA_CALIB			22
#define FLASH_PAGE_HISTORY				42

void FLASH_WriteCurrentIndex(uint16_t index, uint8_t measurementIndex);
void FLASH_WriteDataCurrent(MeasureValue *data, uint8_t measurementIndex);
void FLASH_WriteDataCalib(MeasureValue *data, uint8_t measurementIndex);
void FLASH_WriteDataMeasure(dataMeasure *data, uint8_t measurementIndex);

uint16_t FLASH_ReadCurrentIndex(uint8_t measurementIndex);
dataMeasure FLASH_ReadDataMeasure(uint8_t measurementIndex, uint8_t hisIndex);
MeasureValue FLASH_ReadDataCalib(uint8_t measurementIndex);
MeasureValue FLASH_ReadDataCurrent(uint8_t measurementIndex);

void FLASH_ReadVDRLZ(VDRLZ_Input *readStruct);
void FLASH_WriteVDRLZ(VDRLZ_Input VDRLZ);

#endif /* INC_EEPROM_EMULATE_H_ */
