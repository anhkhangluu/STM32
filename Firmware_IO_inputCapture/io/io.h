#ifndef _IO_H_
#define _IO_H_

#include "main.h"
#include "stm32f072xb.h"

/*------------------------------INPUT----------------------------*/
#define GET_IN0     (((IN0_GPIO_Port->IDR) & IN0_Pin) >> 11)
//#define GET_IN2		(((IN2_GPIO_Port->IDR) & IN2_Pin) >> 9)
#define GET_IN1     (((IN1_GPIO_Port->IDR) & IN1_Pin) >> 10)
#define GET_IN3     (((IN3_GPIO_Port->IDR) & IN3_Pin) >> 8)
#define GET_IN4     (((IN4_GPIO_Port->IDR) & IN4_Pin) >> 15)
#define GET_IN5     (((IN5_GPIO_Port->IDR) & IN5_Pin) >> 14)
#define GET_IN6     (((IN6_GPIO_Port->IDR) & IN6_Pin) >> 13)
#define GET_IN7     (((IN7_GPIO_Port->IDR) & IN7_Pin) >> 12)

/*----------------------------------------------------------------*/
void io_Init(void);
button io_getButton(void);
void io_setOutput(output out, uint8_t *MBReg);
void io_setLedStatus(ledStatus led, uint8_t *MBReg);

static inline input io_getInput(void)
{
    input linput;
    linput.in0 = (GET_IN0 > 0) ? _OFF:_ON;
    linput.in1 = (GET_IN1 > 0) ? _OFF:_ON;
//    linput.in2 = (GET_IN2 > 0) ? _OFF:_ON;
    return linput;
}



#endif
