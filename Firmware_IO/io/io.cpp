#include "..\common\structer.h"
#include "..\common\definition.h"
#include "io.h"


static int SET     = 58;
static int RESET   = 59;
static int HIS     = 60;
static int NEXT    = 61;
static int PREV    = 62;

static int SENSOR0 =  6; //PC24
static int SENSOR1 =  5; //PC25

static int IN0     = 4; //PC26
static int IN1     = 3; //PC28
static int IN2     = 2; //PB25

static int OUT0    =  7;
static int OUT1    =  8;
static int OUT2    =  9;
static int OUT3    = 10;

static int RELAY0  = 11;
static int RELAY1  = 12;

static int LED1    = 56; //A2
static int LED2    = 55; //A1
static int LED3    = 54; //A0
static int LED4    = 57; //A3

static void io_TestOutput(void);


button io_getButton(void)
{
    button lbutton;
    uint8_t i=0;
    for(i=0; i<10; i++)
    {
        delay(5);
        lbutton.set   = (HAL_GPIO_ReadPin(SET)>0?_OFF:_ON);
        lbutton.reset = (HAL_GPIO_ReadPin(RESET)>0?_OFF:_ON);
        lbutton.his   = (HAL_GPIO_ReadPin(HIS)>0?_OFF:_ON);
        lbutton.next  = (HAL_GPIO_ReadPin(NEXT)>0?_OFF:_ON);
        lbutton.prev  = (HAL_GPIO_ReadPin(PREV)>0?_OFF:_ON);
    }
    return lbutton;
}

void io_setOutput(output out)
{
    out.out0>0?HAL_GPIO_WritePin(OUT0,_ON):HAL_GPIO_WritePin(OUT0,_OFF);
    out.out1>0?HAL_GPIO_WritePin(OUT1,_ON):HAL_GPIO_WritePin(OUT1,_OFF);
    out.out2>0?HAL_GPIO_WritePin(OUT2,_ON):HAL_GPIO_WritePin(OUT2,_OFF);
    out.out3>0?HAL_GPIO_WritePin(OUT3,_ON):HAL_GPIO_WritePin(OUT3,_OFF);
    out.rl1>0?HAL_GPIO_WritePin(RELAY0,_ON):HAL_GPIO_WritePin(RELAY0,_OFF);
    out.rl2>0?HAL_GPIO_WritePin(RELAY1,_ON):HAL_GPIO_WritePin(RELAY1,_OFF);
}

void io_setLedStatus(ledStatus led)
{
    led.led0>0?HAL_GPIO_WritePin(LED1,_ON):HAL_GPIO_WritePin(LED1,_OFF);
    led.led1>0?HAL_GPIO_WritePin(LED2,_ON):HAL_GPIO_WritePin(LED2,_OFF);
    led.led2>0?HAL_GPIO_WritePin(LED3,_ON):HAL_GPIO_WritePin(LED3,_OFF);
    led.led3>0?HAL_GPIO_WritePin(LED4,_ON):HAL_GPIO_WritePin(LED4,_OFF);
}

/*Test Function*/
static void io_TestOutput(void)
{
    while(1)
    {
        HAL_GPIO_WritePin(OUT0,_ON);
        HAL_GPIO_WritePin(OUT1,_ON);
        HAL_GPIO_WritePin(OUT2,_ON);
        HAL_GPIO_WritePin(OUT3,_ON);
        HAL_GPIO_WritePin(RELAY0,_ON);
        HAL_GPIO_WritePin(RELAY1,_ON);
        HAL_GPIO_WritePin(LED1,_ON);
        HAL_GPIO_WritePin(LED2,_ON);
        HAL_GPIO_WritePin(LED3,_ON);
        HAL_GPIO_WritePin(LED4,_ON);
        delay(2000);
        HAL_GPIO_WritePin(OUT0,_OFF);
        HAL_GPIO_WritePin(OUT1,_OFF);
        HAL_GPIO_WritePin(OUT2,_OFF);
        HAL_GPIO_WritePin(OUT3,_OFF);
        HAL_GPIO_WritePin(RELAY0,_OFF);
        HAL_GPIO_WritePin(RELAY1,_OFF);
        HAL_GPIO_WritePin(LED1,_OFF);
        HAL_GPIO_WritePin(LED2,_OFF);
        HAL_GPIO_WritePin(LED3,_OFF);
        HAL_GPIO_WritePin(LED4,_OFF);
        delay(2000);
    }
}

