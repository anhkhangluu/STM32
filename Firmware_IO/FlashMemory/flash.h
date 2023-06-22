#ifndef INC_EEPROM_EMULATE_H_
#define INC_EEPROM_EMULATE_H_

#include "main.h"
#include "structer.h"
#define FLASH_SECTOR_ADDR		((uint32_t)0x081C0000)

#define FLASH_ADDRESS                 ((uint32_t)FLASH_SECTOR_ADDR+EEP_ADDRESS)
#define FLASH_CURRENT_INDEX_ADDRESS   ((uint32_t)FLASH_SECTOR_ADDR+EEP_CURRENT_INDEX_ADDRESS  )
#define FLASH_CALIB_ADDRESS           ((uint32_t)FLASH_SECTOR_ADDR+EEP_CALIB_ADDRESS )
#define FLASH_CURREN_VALUE_ADDRESS    ((uint32_t)FLASH_SECTOR_ADDR+EEP_CURREN_VALUE_ADDRESS )
#define FLASH_START_ADDRESS           ((uint32_t)FLASH_SECTOR_ADDR+EEP_START_ADDRESS )//(EEP_CURREN_VALUE_ADDRESS + sizeof(MeasureValue))
//#define FLASH_MAX_DATA                10U

uint8_t FLASH_ReadCurrentIndex(void);
int FLASH_ReadDataMeasure(uint8_t index, dataMeasure *data);
//void Flashwrite

#endif /* INC_EEPROM_EMULATE_H_ */
