/*
 * W5500_config.c
 *
 *  Created on: Jun 16, 2023
 *      Author: Admin
 */
#include "W5500_config.h"



void W5500_Select() {
	HAL_GPIO_WritePin(W5500_CS_GPIO_Port, W5500_CS_Pin, GPIO_PIN_RESET); //CS LOW
}

void W5500_Unselect() {
	HAL_GPIO_WritePin(W5500_CS_GPIO_Port, W5500_CS_Pin, GPIO_PIN_SET); //CS HIGH
}

uint8_t W5500_ReadByte(void) {
	uint8_t rbuf;
	HAL_SPI_Receive(HSPI_W5500, &rbuf, 1, 0xFFFFFFFF);
	return rbuf;
}

void W5500_WriteByte(uint8_t b) {
	HAL_SPI_Transmit(HSPI_W5500, &b, 1, 0xFFFFFFFF);
}

void W5500_ReadBuff(uint8_t *buff, uint16_t len) {
	HAL_SPI_Receive(HSPI_W5500, buff, len, HAL_MAX_DELAY);
}
void W5500_WriteBuff(uint8_t *buff, uint16_t len) {
	HAL_SPI_Transmit(HSPI_W5500, buff, len, HAL_MAX_DELAY);
}

