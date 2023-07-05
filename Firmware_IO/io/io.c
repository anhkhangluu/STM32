#include "..\common\structer.h"
#include "..\common\definition.h"
#include "io.h"

////// check define main.h
//static int SENSOR0 =  6; //PC24
//static int SENSOR1 =  5; //PC25



static void io_TestOutput(void);


button io_getButton(void)
{
    button lbutton;
    uint8_t i=0;
    for(i=0; i<10; i++)
    {
        HAL_Delay(5);
        lbutton.set   = (HAL_GPIO_ReadPin(BT_SET_GPIO_Port,BT_SET_Pin)>0?_OFF:_ON);
        lbutton.reset = (HAL_GPIO_ReadPin(BT_RESET_GPIO_Port,BT_RESET_Pin)>0?_OFF:_ON);
        lbutton.menu   = (HAL_GPIO_ReadPin(BT_MENU_GPIO_Port,BT_MENU_Pin)>0?_OFF:_ON);
        lbutton.next  = (HAL_GPIO_ReadPin(BT_NEXT_GPIO_Port,BT_NEXT_Pin)>0?_OFF:_ON);
        lbutton.prev  = (HAL_GPIO_ReadPin(BT_PREV_GPIO_Port,BT_PREV_Pin)>0?_OFF:_ON);
    }
    return lbutton;
}

void io_setOutput(output out)
{
    out.out0>0?HAL_GPIO_WritePin(OUT0_GPIO_Port,OUT0_Pin,_ON):HAL_GPIO_WritePin(OUT0_GPIO_Port,OUT0_Pin,_OFF);
    out.out1>0?HAL_GPIO_WritePin(OUT1_GPIO_Port,OUT1_Pin,_ON):HAL_GPIO_WritePin(OUT1_GPIO_Port,OUT1_Pin,_OFF);
    out.out2>0?HAL_GPIO_WritePin(OUT2_GPIO_Port,OUT2_Pin,_ON):HAL_GPIO_WritePin(OUT2_GPIO_Port,OUT2_Pin,_OFF);
    out.out3>0?HAL_GPIO_WritePin(OUT3_GPIO_Port,OUT3_Pin,_ON):HAL_GPIO_WritePin(OUT3_GPIO_Port,OUT3_Pin,_OFF);
#if 0
    out.rl1>0?HAL_GPIO_WritePin(RELAY0_GPIO_Port,RELAY0_Pin,_ON):HAL_GPIO_WritePin(RELAY0_GPIO_Port,RELAY0_Pin,_OFF);
    out.rl2>0?HAL_GPIO_WritePin(RELAY1_GPIO_Port,RELAY1_Pin,_ON):HAL_GPIO_WritePin(RELAY1_GPIO_Port,RELAY1_Pin,_OFF);
#endif
}

void io_setLedStatus(ledStatus led)
{
    led.led0>0?HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,_ON):HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,_OFF);
    led.led1>0?HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,_ON):HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,_OFF);
    led.led2>0?HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,_ON):HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,_OFF);
    led.led3>0?HAL_GPIO_WritePin(LED4_GPIO_Port,LED4_Pin,_ON):HAL_GPIO_WritePin(LED4_GPIO_Port,LED4_Pin,_OFF);
}

/*Test Function*/
static void io_TestOutput(void)
{
    while(1)
    {
        HAL_GPIO_WritePin(OUT0_GPIO_Port,OUT0_Pin,_ON);
        HAL_GPIO_WritePin(OUT1_GPIO_Port,OUT1_Pin,_ON);
        HAL_GPIO_WritePin(OUT2_GPIO_Port,OUT2_Pin,_ON);
        HAL_GPIO_WritePin(OUT3_GPIO_Port,OUT3_Pin,_ON);
#if 0 //TODO
        HAL_GPIO_WritePin(RELAY0_GPIO_Port,RELAY0_Pin,_ON);
        HAL_GPIO_WritePin(RELAY1_GPIO_Port,RELAY1_Pin,_ON);
#endif
        HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,_ON);
        HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,_ON);
        HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,_ON);
        HAL_GPIO_WritePin(LED4_GPIO_Port,LED4_Pin,_ON);
        HAL_Delay(2000);
        HAL_GPIO_WritePin(OUT0_GPIO_Port,OUT0_Pin,_OFF);
        HAL_GPIO_WritePin(OUT1_GPIO_Port,OUT1_Pin,_OFF);
        HAL_GPIO_WritePin(OUT2_GPIO_Port,OUT2_Pin,_OFF);
        HAL_GPIO_WritePin(OUT3_GPIO_Port,OUT3_Pin,_OFF);
#if 0 //TODO
        HAL_GPIO_WritePin(RELAY0_GPIO_Port,RELAY0_Pin,_OFF);
        HAL_GPIO_WritePin(RELAY1_GPIO_Port,RELAY1_Pin,_OFF);
#endif
        HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,_OFF);
        HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,_OFF);
        HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,_OFF);
        HAL_GPIO_WritePin(LED4_GPIO_Port,LED4_Pin,_OFF);
        delay(2000);
    }
}

