#include <Wire.h>
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

void io_Init(void)
{
    /*Config Button*/
    pinMode(SET,   INPUT_PULLUP);
    pinMode(RESET, INPUT_PULLUP);
    pinMode(HIS,   INPUT_PULLUP);
    pinMode(NEXT,  INPUT_PULLUP);
    pinMode(PREV,  INPUT_PULLUP);

    /*Config Input*/
    pinMode(SENSOR0,   INPUT_PULLUP);
    pinMode(SENSOR1,   INPUT_PULLUP);
    pinMode(IN0,       INPUT_PULLUP);
    pinMode(IN1,       INPUT_PULLUP);
    pinMode(IN2,       INPUT_PULLUP);

    /*Config Out*/
    pinMode(OUT0,      OUTPUT);
    pinMode(OUT1,      OUTPUT);
    pinMode(OUT2,      OUTPUT);
    pinMode(OUT3,      OUTPUT);
    pinMode(RELAY0,    OUTPUT);
    pinMode(RELAY1,    OUTPUT);

    pinMode(LED1,      OUTPUT);
    pinMode(LED2,      OUTPUT);
    pinMode(LED3,      OUTPUT);
    pinMode(LED4,      OUTPUT);

    //io_TestOutput();
}

button io_getButton(void)
{
    button lbutton;
    uint8_t i=0;
    for(i=0; i<10; i++)
    {
        delay(5);
        lbutton.set   = (digitalRead(SET)>0?_OFF:_ON);
        lbutton.reset = (digitalRead(RESET)>0?_OFF:_ON);
        lbutton.his   = (digitalRead(HIS)>0?_OFF:_ON);
        lbutton.next  = (digitalRead(NEXT)>0?_OFF:_ON);
        lbutton.prev  = (digitalRead(PREV)>0?_OFF:_ON);
    }
    return lbutton;
}

void io_setOutput(output out)
{
    out.out0>0?digitalWrite(OUT0,_ON):digitalWrite(OUT0,_OFF);
    out.out1>0?digitalWrite(OUT1,_ON):digitalWrite(OUT1,_OFF);
    out.out2>0?digitalWrite(OUT2,_ON):digitalWrite(OUT2,_OFF);
    out.out3>0?digitalWrite(OUT3,_ON):digitalWrite(OUT3,_OFF);
    out.rl1>0?digitalWrite(RELAY0,_ON):digitalWrite(RELAY0,_OFF);
    out.rl2>0?digitalWrite(RELAY1,_ON):digitalWrite(RELAY1,_OFF);
}

void io_setLedStatus(ledStatus led)
{
    led.led0>0?digitalWrite(LED1,_ON):digitalWrite(LED1,_OFF);
    led.led1>0?digitalWrite(LED2,_ON):digitalWrite(LED2,_OFF);
    led.led2>0?digitalWrite(LED3,_ON):digitalWrite(LED3,_OFF);
    led.led3>0?digitalWrite(LED4,_ON):digitalWrite(LED4,_OFF);
}

/*Test Function*/
static void io_TestOutput(void)
{
    while(1)
    {
        digitalWrite(OUT0,_ON);
        digitalWrite(OUT1,_ON);
        digitalWrite(OUT2,_ON);
        digitalWrite(OUT3,_ON);
        digitalWrite(RELAY0,_ON);
        digitalWrite(RELAY1,_ON);
        digitalWrite(LED1,_ON);
        digitalWrite(LED2,_ON);
        digitalWrite(LED3,_ON);
        digitalWrite(LED4,_ON);
        delay(2000);
        digitalWrite(OUT0,_OFF);
        digitalWrite(OUT1,_OFF);
        digitalWrite(OUT2,_OFF);
        digitalWrite(OUT3,_OFF);
        digitalWrite(RELAY0,_OFF);
        digitalWrite(RELAY1,_OFF);
        digitalWrite(LED1,_OFF);
        digitalWrite(LED2,_OFF);
        digitalWrite(LED3,_OFF);
        digitalWrite(LED4,_OFF);
        delay(2000);
    }
}
