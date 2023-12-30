#ifndef INC_EEPROM_EMULATE_H_
#define INC_EEPROM_EMULATE_H_

#include "stm32f3xx_hal.h"
#include "structer.h"

#define MEMORY_ADDR		((uint32_t )0x0801F800)
#define INDEX_DATA_CALIB_1	0U
#define INDEX_DATA_CALIB_2  20U
#define INDEX_VDRLZ			40U

void FLASH_WriteDataCalib(MeasureValue *data, uint8_t measurementIndex);
MeasureValue FLASH_ReadDataCalib(uint8_t measurementIndex);


VDRLZ_Input FLASH_ReadVDRLZ();
void FLASH_WriteVDRLZ(VDRLZ_Input *data);
#endif /* INC_EEPROM_EMULATE_H_ */
