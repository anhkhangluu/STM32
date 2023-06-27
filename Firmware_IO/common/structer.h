#ifndef _STRUCTER_H_
#define _STRUCTER_H_
#include <stdint.h>
#include "definition.h"
typedef enum _CycleTime
{
    SET_YEAR = 0,
    SET_MONTH,
    SET_DAY,
    SET_HOUR,
    SET_MINUTE,
    EXIT
}CycleTime;

typedef enum
{
    NONE = 0,
    ZONLY,
    ZERROR1,
    ZERROR2,
    MEASUREALL
}modeMeasure;

typedef enum
{
    CALIBSET = 0,
    CALIBRESET
}setCalibValue;

typedef enum _CycleMeasure
{
    START = 0,
    STOP,
    CLEARSENSOR,
    WAITMEASUREZ,
    WAITRBSTABLEZ,
    MEASUREZ,
    CHECKZVALUE,
    Z_OK,
    Z_NOT_OK,
    SETVALUEZ,
    WAITMEASUREX1Y1,
    WAITRBSTABLEX1Y1,
    MEASUREX1Y1,
    WAITMEASUREX2Y2,
    WAITRBSTABLEX2Y2,
    MEASUREX2Y2,
    WAITSETVALUE,
    CALCULATORVALUE,
    _ERROR,
    FINISH
}CycleMeasure;

typedef enum _CycleMeasureSensor
{
    SEN_START = 0,
    SEN_STOP,
    SEN_FINISH
}CycleMeasureSensor;

typedef enum _GetInputType
{
    GET_BUTTON = 0,
    GET_SENSOR
}GetInputType;

typedef enum
{
    SENSORCHANGE = 0,
    SENSORNOCHANGE
}SensorChange;

#pragma pack(1)
typedef struct _Time 
{
	uint8_t year; /*0:99*/
    uint8_t month; /*1:12*/
    uint8_t day; /*1:31*/
    uint8_t hour; /*0:23*/
    uint8_t minute; /*0:59*/
    uint8_t second; /*0:59*/
}Time;

typedef struct _Coordinates
{
    int16_t X;
    int16_t Y;
    int16_t Z;
    int16_t R;
    int16_t aX;
    int16_t aY;
}Coordinates;

typedef struct _MeasureValue
{
    int32_t X1;
    int32_t Y1;
    int32_t X2;
    int32_t Y2;
    int32_t Z;
}MeasureValue;

typedef struct _dataMeasure
{
    Coordinates coordinates;
    Time time;
    uint8_t mode;
}dataMeasure;

typedef struct _screenData
{
    char line1[LCD_LINE_SIZE+1];
    char line2[LCD_LINE_SIZE+1];
    char line3[LCD_LINE_SIZE+1];
    char line4[LCD_LINE_SIZE+1];
}screenData;

typedef struct _button
{
    uint8_t set;
    uint8_t reset;
    uint8_t menu;
    uint8_t next;
    uint8_t prev;
}button;

typedef struct _sensor
{
    uint8_t s0;
    uint8_t s1;
}sensor;

typedef struct _input
{
    uint8_t in0;
    uint8_t in1;
    uint8_t in2;
}input;

typedef struct _output
{
    uint8_t out0;
    uint8_t out1;
    uint8_t out2;
    uint8_t out3;
    uint8_t rl1;
    uint8_t rl2;
}output;

typedef struct _ledStatus
{
    uint8_t led0;
    uint8_t led1;
    uint8_t led2;
    uint8_t led3;
}ledStatus;

typedef struct _timer
{
    uint8_t flat;
    uint8_t status;
    uint32_t count;
    uint32_t inc;
}timer;
#pragma pack()
#endif
