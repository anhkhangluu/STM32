
#ifndef LCD_CONFIG_H
#define LCD_CONFIG_H 

#include "main.h"

#define _LCD_COLS         16
#define _LCD_ROWS         4

#define _LCD_RS_PORT      GPIOE
#define _LCD_RS_PIN       GPIO_PIN_5

#define _LCD_E_PORT       GPIOE
#define _LCD_E_PIN        GPIO_PIN_2

#define _LCD_RW_PORT      GPIOE
#define _LCD_RW_PIN       GPIO_PIN_4

#define _LCD_D4_PORT      GPIOE
#define _LCD_D4_PIN		  GPIO_PIN_1

#define _LCD_D5_PORT      GPIOE
#define _LCD_D5_PIN       GPIO_PIN_0

#define _LCD_D6_PORT      GPIOB
#define _LCD_D6_PIN       GPIO_PIN_9

#define _LCD_D7_PORT      GPIOB
#define _LCD_D7_PIN       GPIO_PIN_8

#endif

