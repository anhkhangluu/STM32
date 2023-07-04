#ifndef _IO_H_
#define _IO_H_

#include "main.h"
#include "stm32f072xb.h"


#define GET_IN0     IN0_GPIO_Port->IDR & IN0_Pin
#define GET_IN1     IN1_GPIO_Port->IDR & IN1_Pin
//#define GET_IN2     (PIOB->PIO_PDSR & PIO_IFSR_P25)

#define GET_SENSOR0 SENSOR0_GPIO_Port->IDR & SENSOR0_Pin
#define GET_SENSOR1 SENSOR1_GPIO_Port->IDR & SENSOR1_Pin

void io_Init(void);
button io_getButton(void);
void io_setOutput(output out);
void io_setLedStatus(ledStatus led);

inline input io_getInput(void)
{
    input linput;
    linput.in0 = ((GET_IN0) > 0 ? _OFF:_ON);
    linput.in1 = ((GET_IN1) > 0 ? _OFF:_ON);
//    linput.in2 = ((GET_IN2) > 0 ? _OFF:_ON);
    return linput;
}

inline sensor io_getSensor(void)
{
    sensor lsensor;
    lsensor.s0 = ((GET_SENSOR0) > 0 ? _OFF:_ON);
    lsensor.s1 = ((GET_SENSOR1) > 0 ? _OFF:_ON);
    return lsensor;
}

#endif
