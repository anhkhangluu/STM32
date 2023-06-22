/*
 * W5500_config.h
 *
 *  Created on: Jun 16, 2023
 *      Author: Admin
 */

#ifndef INC_W5500_CONFIG_H_
#define INC_W5500_CONFIG_H_

#include "main.h"

void W5500_Select();
void W5500_Unselect();
uint8_t W5500_ReadByte(void);
void W5500_WriteByte(uint8_t b);
void W5500_ReadBuff(uint8_t *buff, uint16_t len);
void W5500_WriteBuff(uint8_t *buff, uint16_t len);

#endif /* INC_W5500_CONFIG_H_ */
