#include <stdint.h>
#include "lcd.h"

#define SET_IF(expr)  ((expr) ? GPIO_PIN_SET : GPIO_PIN_RESET)
char display_settings;

static uint8_t read4Bits(void)
{
    HAL_GPIO_WritePin(E_Port, E_Pin, GPIO_PIN_SET);
    uint32_t data5 = HAL_GPIO_ReadPin(DATA5_Port, DATA5_Pin);
    uint32_t data6 = HAL_GPIO_ReadPin(DATA6_Port, DATA6_Pin);
    uint32_t data7 = HAL_GPIO_ReadPin(DATA7_Port, DATA7_Pin);
    uint32_t data8 = HAL_GPIO_ReadPin(DATA8_Port, DATA8_Pin);
    uint8_t data = (data8 << 3) | (data7 << 2) | (data6 << 1) | data5;
    HAL_GPIO_WritePin(E_Port, E_Pin, GPIO_PIN_RESET);
    return data;
}

static void waitBusy(void)
{
    HAL_GPIO_WritePin(RS_Port, RS_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RW_Port, RW_Pin, GPIO_PIN_SET);

    uint8_t busyFlag;
    do {
        busyFlag = (read4Bits() & 0x08);
        read4Bits();
    } while (busyFlag);

    HAL_GPIO_WritePin(RW_Port, RW_Pin, GPIO_PIN_RESET);
}

static void fallingEdge(void)
{
    HAL_GPIO_WritePin(E_Port, E_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(E_Port, E_Pin, GPIO_PIN_RESET);
}

static void send4Bits(char data)
{
    waitBusy();
    HAL_GPIO_WritePin(DATA5_Port, DATA5_Pin, SET_IF(data & 0x01));
    HAL_GPIO_WritePin(DATA6_Port, DATA6_Pin, SET_IF(data & 0x02));
    HAL_GPIO_WritePin(DATA7_Port, DATA7_Pin, SET_IF(data & 0x04));
    HAL_GPIO_WritePin(DATA8_Port, DATA8_Pin, SET_IF(data & 0x08));
    fallingEdge();
}

static void sendCommand(char cmd)
{
    waitBusy();
    HAL_GPIO_WritePin(RS_Port, RS_Pin, GPIO_PIN_RESET);
    send4Bits(cmd >> 4);
    send4Bits(cmd & 0x0F);
}

static void sendData(char data)
{
    waitBusy();
    HAL_GPIO_WritePin(RS_Port, RS_Pin, GPIO_PIN_SET);
    send4Bits(data >> 4);
    send4Bits(data & 0x0F);
}

void LCD_Clear(void)
{
    sendCommand(LCD_CLEARDISPLAY);
}

void putLCD(char c)
{
    sendData(c);
}

void writeLCD(char *str)
{
    for (; *str != 0; ++str)
    {
        sendData(*str);
    }
}

void LCD_Init(void)
{
    HAL_GPIO_WritePin(E_Port, E_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RS_Port, RS_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RW_Port, RW_Pin, GPIO_PIN_RESET);

    HAL_Delay(50);
    display_settings = LCD_4BITMODE | LCD_2LINE | LCD_5x8DOTS;
    send4Bits(0x03); HAL_Delay(5);
    send4Bits(0x03); HAL_Delay(5);
    send4Bits(0x03); HAL_Delay(2);
    send4Bits(0x02);

    sendCommand(LCD_FUNCTIONSET | display_settings);
    display_settings = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
    sendCommand(LCD_DISPLAYCONTROL | display_settings);

    LCD_Clear();
    display_settings = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
    sendCommand(LCD_ENTRYMODESET | display_settings);
}

void setCursor(uint8_t x, uint8_t y)
{
    const uint8_t row_offsets[] = {0x00, 0x40, 0x10, 0x50};

    if (y >= _LCD_ROWS)
        y = 0;

    sendCommand(0x80 | (row_offsets[y] + x));
}

void cursorOn(void)
{
    sendCommand(0x08 | 0x04 | 0x02);
}

void blinkOn(void)
{
    sendCommand(0x08 | 0x04 | 0x01);
}

void clearDisp(void)
{
    sendCommand(0x08 | 0x04 | 0x00);
}

void setDisplay(lcdDispSetting_t dispSetting)
{
    sendCommand(0x08 | (dispSetting & 0x07));
}

void LCD_Puts(uint8_t x, uint8_t y, char *str)
{
    setCursor(x, y);
    writeLCD(str);
}