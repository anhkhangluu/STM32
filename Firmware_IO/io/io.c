#include "..\common\structer.h"
#include "..\common\definition.h"
#include "io.h"


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

void io_setOutput(output out, uint8_t *MBReg)
{
	MBReg[0] = out.out0;
	MBReg[1] = out.out1;
	MBReg[2] = out.out2;
	MBReg[3] = out.out3;
	MBReg[4] = out.out4;
	MBReg[5] = out.out5;
	MBReg[6] = out.out6;
	MBReg[7] = out.out7;


	MBReg[0]>0?HAL_GPIO_WritePin(OUT0_GPIO_Port,OUT0_Pin,_ON):HAL_GPIO_WritePin(OUT0_GPIO_Port,OUT0_Pin,_OFF);
    MBReg[1]>0?HAL_GPIO_WritePin(OUT1_GPIO_Port,OUT1_Pin,_ON):HAL_GPIO_WritePin(OUT1_GPIO_Port,OUT1_Pin,_OFF);
    MBReg[2]>0?HAL_GPIO_WritePin(OUT2_GPIO_Port,OUT2_Pin,_ON):HAL_GPIO_WritePin(OUT2_GPIO_Port,OUT2_Pin,_OFF);
    MBReg[3]>0?HAL_GPIO_WritePin(OUT3_GPIO_Port,OUT3_Pin,_ON):HAL_GPIO_WritePin(OUT3_GPIO_Port,OUT3_Pin,_OFF);
    MBReg[4]>0?HAL_GPIO_WritePin(OUT4_GPIO_Port,OUT4_Pin,_ON):HAL_GPIO_WritePin(OUT4_GPIO_Port,OUT4_Pin,_OFF);
    MBReg[5]>0?HAL_GPIO_WritePin(OUT5_GPIO_Port,OUT5_Pin,_ON):HAL_GPIO_WritePin(OUT5_GPIO_Port,OUT5_Pin,_OFF);
    MBReg[6]>0?HAL_GPIO_WritePin(OUT6_GPIO_Port,OUT6_Pin,_ON):HAL_GPIO_WritePin(OUT6_GPIO_Port,OUT6_Pin,_OFF);
    MBReg[7]>0?HAL_GPIO_WritePin(OUT7_GPIO_Port,OUT7_Pin,_ON):HAL_GPIO_WritePin(OUT7_GPIO_Port,OUT7_Pin,_OFF);
}

void io_setLedStatus(ledStatus led, uint8_t *MBReg)
{
	MBReg[8] = led.led1;
	MBReg[9] = led.led2;
	MBReg[10] = led.led3;
//    led.led0>0?HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,_ON):HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,_OFF);
    MBReg[8]==0?HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,_ON):HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,_OFF);
    MBReg[9]==0?HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,_ON):HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,_OFF);
    MBReg[10]==0?HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,_ON):HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,_OFF);
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
        HAL_GPIO_WritePin(OUT4_GPIO_Port,OUT4_Pin,_ON);
        HAL_GPIO_WritePin(OUT5_GPIO_Port,OUT5_Pin,_ON);
        HAL_GPIO_WritePin(OUT6_GPIO_Port,OUT6_Pin,_ON);
        HAL_GPIO_WritePin(OUT7_GPIO_Port,OUT7_Pin,_ON);

        HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,_ON);
        HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,_ON);
        HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,_ON);

        HAL_Delay(2000);
        HAL_GPIO_WritePin(OUT0_GPIO_Port,OUT0_Pin,_OFF);
        HAL_GPIO_WritePin(OUT1_GPIO_Port,OUT1_Pin,_OFF);
        HAL_GPIO_WritePin(OUT2_GPIO_Port,OUT2_Pin,_OFF);
        HAL_GPIO_WritePin(OUT3_GPIO_Port,OUT3_Pin,_OFF);
        HAL_GPIO_WritePin(OUT4_GPIO_Port,OUT4_Pin,_OFF);
        HAL_GPIO_WritePin(OUT5_GPIO_Port,OUT5_Pin,_OFF);
        HAL_GPIO_WritePin(OUT6_GPIO_Port,OUT6_Pin,_OFF);
        HAL_GPIO_WritePin(OUT7_GPIO_Port,OUT7_Pin,_OFF);

        HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,_OFF);
        HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,_OFF);
        HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin,_OFF);
        HAL_Delay(2000);
    }
}

