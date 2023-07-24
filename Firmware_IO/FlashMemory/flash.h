#ifndef INC_EEPROM_EMULATE_H_
#define INC_EEPROM_EMULATE_H_

#include "stm32f0xx_hal.h"
#include "structer.h"

void FLASH_WriteDataCalib(MeasureValue *data, uint8_t measurementIndex);
MeasureValue FLASH_ReadDataCalib(uint8_t measurementIndex);


VDRLZ_Input FLASH_ReadVDRLZ();
void FLASH_WriteVDRLZ(VDRLZ_Input *data);

#endif /* INC_EEPROM_EMULATE_H_ */
