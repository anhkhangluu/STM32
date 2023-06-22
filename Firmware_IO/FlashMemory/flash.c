/*
 * eeprom_emulate.c
 *
 *  Created on: Jun 21, 2023
 *      Author: PC
 */

#include "flash.h"

uint8_t FLASH_ReadCurrentIndex(void)
{
	return (uint8_t *)FLASH_ADDRESS;
}

