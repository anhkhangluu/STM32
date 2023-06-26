#ifndef _IO_H_
#define _IO_H_

#define GET_IN0     (PIOC->PIO_PDSR & PIO_IFSR_P26)
#define GET_IN1     (PIOC->PIO_PDSR & PIO_IFSR_P28)
#define GET_IN2     (PIOB->PIO_PDSR & PIO_IFSR_P25)

#define GET_SENSOR0 (PIOC->PIO_PDSR & PIO_IFSR_P24)
#define GET_SENSOR1 (PIOC->PIO_PDSR & PIO_IFSR_P25)

void io_Init(void);
button io_getButton(void);
void io_setOutput(output out);
void io_setLedStatus(ledStatus led);

inline input io_getInput(void)
{
    input linput;
    linput.in0 = ((GET_IN0) > 0 ? _OFF:_ON);
    linput.in1 = ((GET_IN1) > 0 ? _OFF:_ON);
    linput.in2 = ((GET_IN2) > 0 ? _OFF:_ON);
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