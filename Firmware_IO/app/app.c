#include <Wire.h>
#include <math.h>
#include <stdint.h>
#include "..\common\definition.h"
#include "..\common\structer.h"
#include "..\rtc\rtc.h"
#include "..\lcd\lcd.h"
#include "..\screen\screen.h"
#include "..\io\io.h"
#include "..\eeprom\eeprom.h"
#include "..\timer\timer.h"
#include "app.h"

#define TIME_WAIT 100
static dataMeasure mdata =
{
  {0,0,0,0,0,0},
  {0,0,0,0,0,0}
};

static Time mtime = 
{
  2022,05,21,06,00,00
};

static button mbutton;
static input minput;
static sensor msensor;
static output moutput;
static ledStatus mledStatus;
static MeasureValue mcalibValue;
static MeasureValue mCurrentMeasureValue;
static MeasureValue mmeasureValue;
static setCalibValue msetCalibValue;

static void app_SettingRtc(void);
static void app_SettingData(void);
static void app_GetEeprom(void);
static void app_Measurement(void);
static void app_SetCalibValue(void);
static void app_GetCalibValue(void);
static void app_CalculatorValue(CycleMeasure lcycleMeasure);
static void app_HisValue(void);
static void app_ClearAllOutput(void);
static void app_GotoMainScreen(uint8_t option);
static void app_SetCurrentMeasureValue(void);
static void app_GetCurrentMeasureValue(void);
static void app_TrigerOutputON(void);
static void app_TrigerOutputOFF(void);



/*private function*/
static void app_TrigerOutputOFF(void)
{
    moutput.rl2 = _OFF;
    io_setOutput(moutput);
    HAL_Delay(100);
}

static void app_TrigerOutputON(void)
{
    HAL_Delay(100);
    moutput.rl2 = _ON;
    io_setOutput(moutput);
}

static void app_GotoMainScreen(uint8_t option)
{
    uint8_t index;
    dataMeasure data;
    index = epp_ReadCurrentIndex();
    if(10 <= index)
    {
        index = 9;
    }
    else if(0 < index)
    {
        index--; 
    }
    (void)eep_ReadDataMeasure(index, &data);
    screen_Clear();
    screen_DataMeasure(data, option);
}

static void app_ClearAllOutput(void)
{
    moutput.out0 = _OFF;
    moutput.out1 = _OFF;
    moutput.rl1 = _OFF;
    io_setOutput(moutput);
}
static void app_HisValue(void)
{
    uint8_t index=0;
    uint8_t exit = 0;
    dataMeasure ldata;
    uint8_t u8_Led3 = mledStatus.led3;
    screen_Clear();
    eep_ReadDataMeasure(index, &ldata);
    screen_DataMeasure(ldata, CALIBSET);
    do
    {
        mbutton = io_getButton();
        if(TIME_RUN != timer_Status(TIMER_CLEARSENSOR))
        {
            timer_Start(TIMER_CLEARSENSOR,10000);
            mledStatus.led3 ^=0x01;
            io_setLedStatus(mledStatus);
        }
        if(_ON == mbutton.next)
        {
            mbutton.next = _OFF;
            while(_ON == io_getButton().next);
            index++;
            if(index >= EEP_MAX_DATA)
            {
                index = 0;
            }
            eep_ReadDataMeasure(index, &ldata);
            screen_DataMeasure(ldata, CALIBSET);
        }
        if(_ON == mbutton.prev)
        {
            mbutton.prev = _OFF;
            while(_ON == io_getButton().prev);
            if(index > 0)
            {
                index--;
            }
            else
            {
                index = EEP_MAX_DATA - 1;
            }
            eep_ReadDataMeasure(index, &ldata);
            screen_DataMeasure(ldata , CALIBSET);
        }
        if(_ON == mbutton.reset)
        {
            mbutton.reset = _OFF;
            while(_ON == io_getButton().reset);
            screen_Clear();
            exit = 1;
        }
    }
    while(exit == 0);
    time_Stop(TIMER_CLEARSENSOR);
    mledStatus.led3 = u8_Led3;
    io_setLedStatus(mledStatus);
    if(CALIBSET == msetCalibValue)
    {
        app_GotoMainScreen(CALIBSET);
    }
    else
    {
        app_GotoMainScreen(CALIBRESET);
    }
}

static void app_SettingRtc(void)
{
    CycleTime cycle = SET_YEAR;
    uint8_t exit=0;
    uint8_t tmp=0;
    screen_Clear();
    do
    {
        /*set year*/
        if(SET_YEAR == cycle)
        {
            mtime = rtc_Now();
            screen_setDateTime(mtime,SET_YEAR);
            do
            {
                mbutton = io_getButton();
                if(_ON == mbutton.set)
                {
                    tmp = (mtime.year >=2000)?(mtime.year-2000):(byte)mtime.year;
                    tmp += 1;
                    if(99 < tmp) tmp = 0;
                    mtime.year = tmp + 2000;
                    rtc_SetYear(mtime.year);
                    screen_setDateTime(mtime,SET_YEAR);
                    HAL_Delay(TIME_WAIT);
                }
                if(_ON == mbutton.next)
                {
                    cycle = SET_MONTH;
                    while(_ON == io_getButton().next);
                }
                if(_ON == mbutton.reset)
                {
                    while(_ON == io_getButton().reset);
                    exit = 1;
                    break;
                }
            } while (SET_YEAR == cycle);
        }
        /*set month*/
        if(SET_MONTH == cycle)
        {
            mtime = rtc_Now();
            screen_setDateTime(mtime,SET_MONTH);
            do
            {
                mbutton = io_getButton();
                if(_ON == mbutton.set)
                {
                    mtime.month += 1;
                    if(12 < mtime.month) mtime.month = 1;
                    rtc_SetMonth(mtime.month);
                    screen_setDateTime(mtime,SET_MONTH);
                    HAL_Delay(TIME_WAIT);
                }
                if(_ON == mbutton.next)
                {
                    cycle = SET_DAY;
                    while(_ON == io_getButton().next);
                }
                if(_ON == mbutton.reset)
                {
                    while(_ON == io_getButton().reset);
                    exit = 1;
                    break;
                }
            } while (SET_MONTH == cycle);
        }
        /*set day*/
        if(SET_DAY == cycle)
        {
            mtime = rtc_Now();
            screen_setDateTime(mtime,SET_DAY);
            do
            {
                mbutton = io_getButton();
                if(1U == mbutton.set)
                {
                    mtime.day += 1;
                    if(31 < mtime.day) mtime.day = 1;
                    rtc_SetDay(mtime.day);
                    screen_setDateTime(mtime,SET_DAY);
                    HAL_Delay(TIME_WAIT);
                }
                if(_ON == mbutton.next)
                {
                    cycle = SET_HOUR;
                    while(_ON == io_getButton().next);
                }
                if(_ON == mbutton.reset)
                {
                    while(_ON == io_getButton().reset);
                    exit = 1;
                    break;
                }
            } while (SET_DAY == cycle);
        }
        /*set hour*/
        if(SET_HOUR == cycle)
        {
            mtime = rtc_Now();
            screen_setDateTime(mtime,SET_HOUR);
            do
            {
                mbutton = io_getButton();
                if(_ON == mbutton.set)
                {
                    mtime.hour += 1;
                    if(59 < mtime.hour) mtime.hour = 1;
                    rtc_SetHour(mtime.hour);
                    rtc_SetSecond(0);
                    screen_setDateTime(mtime,SET_HOUR);
                    HAL_Delay(TIME_WAIT);
                }
                if(_ON == mbutton.next)
                {
                    cycle = SET_MINUTE;
                    while(_ON == io_getButton().next);
                }
                if(_ON == mbutton.reset)
                {
                    while(0U == io_getButton().reset);
                    exit = 1;
                    break;
                }
            } while (SET_HOUR == cycle);
        }
        /*set minute*/
        if(SET_MINUTE == cycle)
        {
            mtime = rtc_Now();
            screen_setDateTime(mtime,SET_MINUTE);
            do
            {
                mbutton = io_getButton();
                if(_ON == mbutton.set)
                {
                    mtime.minute+=1;
                    if(59 < mtime.minute) mtime.minute = 1;
                    rtc_SetMinute(mtime.minute);
                    rtc_SetSecond(0);
                    screen_setDateTime(mtime,SET_MINUTE);
                    HAL_Delay(TIME_WAIT);
                }
                if(_ON == mbutton.next)
                {
                    cycle = SET_YEAR;
                    while(_ON == io_getButton().next);
                }
                if(_ON == mbutton.reset)
                {
                    while(_ON == io_getButton().reset);
                    exit = 1;
                    break;
                }
            } while (SET_MINUTE == cycle);
        }
    } while (0 == exit);
    if(CALIBSET == msetCalibValue)
    {
        app_GotoMainScreen(CALIBSET);
    }
    else
    {
        app_GotoMainScreen(CALIBRESET);
    }
}

static void app_SetCalibValue(void)
{
    mcalibValue.X1 = mmeasureValue.X1;
    mcalibValue.Y1 = mmeasureValue.Y1;
    mcalibValue.X2 = mmeasureValue.X2;
    mcalibValue.Y2 = mmeasureValue.Y2;
    mcalibValue.Z  = mmeasureValue.Z;
    (void)eep_WriteDataCalib(&mcalibValue);
    DBG("mcalibValue.X1 = %d\n",mcalibValue.X1);
    DBG("mcalibValue.Y1 = %d\n",mcalibValue.Y1);
    DBG("mcalibValue.X2 = %d\n",mcalibValue.X2);
    DBG("mcalibValue.Y2 = %d\n",mcalibValue.Y2);
    DBG("mcalibValue.Z = %d\n",mcalibValue.Z);
}

static void app_GetCalibValue(void)
{
    (void)eep_ReadDataCalib(&mcalibValue);
    DBG("mcalibValue.X1 = %d\n",mcalibValue.X1);
    DBG("mcalibValue.Y1 = %d\n",mcalibValue.Y1);
    DBG("mcalibValue.X2 = %d\n",mcalibValue.X2);
    DBG("mcalibValue.Y2 = %d\n",mcalibValue.Y2);
    DBG("mcalibValue.Z = %d\n",mcalibValue.Z);
}

static void app_SetCurrentMeasureValue(void)
{
    mCurrentMeasureValue.X1 = mmeasureValue.X1;
    mCurrentMeasureValue.Y1 = mmeasureValue.Y1;
    mCurrentMeasureValue.X2 = mmeasureValue.X2;
    mCurrentMeasureValue.Y2 = mmeasureValue.Y2;
    mCurrentMeasureValue.Z  = mmeasureValue.Z;
    (void)eep_WriteDataCurrent(&mCurrentMeasureValue);
    DBG("mCurrentMeasureValue.X1 = %d\n",mCurrentMeasureValue.X1);
    DBG("mCurrentMeasureValue.Y1 = %d\n",mCurrentMeasureValue.Y1);
    DBG("mCurrentMeasureValue.X2 = %d\n",mCurrentMeasureValue.X2);
    DBG("mCurrentMeasureValue.Y2 = %d\n",mCurrentMeasureValue.Y2);
    DBG("mCurrentMeasureValue.Z = %d\n",mCurrentMeasureValue.Z);
}

static void app_GetCurrentMeasureValue(void)
{
    (void)eep_ReadDataCurrent(&mCurrentMeasureValue);
    DBG("mCurrentMeasureValue.X1 = %d\n",mCurrentMeasureValue.X1);
    DBG("mCurrentMeasureValue.Y1 = %d\n",mCurrentMeasureValue.Y1);
    DBG("mCurrentMeasureValue.X2 = %d\n",mCurrentMeasureValue.X2);
    DBG("mCurrentMeasureValue.Y2 = %d\n",mCurrentMeasureValue.Y2);
    DBG("mCurrentMeasureValue.Z = %d\n",mCurrentMeasureValue.Z);
}

static void app_CalculatorValue(CycleMeasure lcycleMeasures, uint8_t mode)
{
    double db_DetaTX1 = 0;
    double db_DetaTY1 = 0;
    double db_DetaTX2 = 0;
    double db_DetaTY2 = 0;
    double db_DetaTZ = 0;

    double  db_anpha1 = 0;
    double  db_beta1 = 0;
    double  db_anpha2 = 0;
    double  db_beta2 = 0;

    double db_Vrb1 = 20.0; /* mm/s */
    double db_Vrb2 = 20.0; /* mm/s */
    double db_R = 20.0;    /* mm */
    double db_D = 20.0;    /* mm */
    double db_a = 1.2;   /* mm */
    double db_b = 1.2;   /* mm */

    double db_DetaXSS1 = 0;
    double db_DetaYSS1 = 0;
    double db_DetaXSS2 = 0;
    double db_DetaYSS2 = 0;
    double db_DetaXRB1 = 0;
    double db_DetaYRB1 = 0;
    double db_DetaXRB2 = 0;
    double db_DetaYRB2 = 0;
    double db_DetaZ = 0;
    double db_r1 = 0;
    double db_r2 = 0;
    double db_LXSS = 0;
    double db_LYSS = 0;
    double db_LXRB = 0;
    double db_LYRB = 0;

    mdata.time = rtc_Now();
    if(CALIBSET == msetCalibValue)
    {
        DBG("**********************************************************\n")
        /*calculator Z*/
        if((ZONLY == mode) || (MEASUREALL == mode))
        {
            db_DetaTZ  = ((double)(mmeasureValue.Z  - mcalibValue.Z))*100/1000000.0;  /* us/1000000 => s */
            DBG("db_DetaTZ = %lf\n",db_DetaTZ);
            db_DetaZ = db_Vrb2 * db_DetaTZ;
            DBG("db_DetaZ = %lf\n",db_DetaZ);
            mdata.coordinates.Z = (int16_t)(db_DetaZ * 100.0);
            DBG("Z (db_DetaZ)= %lf\n",db_DetaZ);

            if((db_DetaZ > db_b) || (db_DetaZ < (-db_b)))
            {
                moutput.out3 = _ON;
            }
            else
            {
                moutput.out3 = _OFF;
            }
        }


        /*calculator X Y*/
        if (FINISH == lcycleMeasures)
        {
            db_DetaTX1 = ((double)(mmeasureValue.X1 - mcalibValue.X1))*100/1000000.0; /* us/1000000 => s */
            db_DetaTY1 = ((double)(mmeasureValue.Y1 - mcalibValue.Y1))*100/1000000.0; /* us/1000000 => s */
            db_DetaTX2 = ((double)(mmeasureValue.X2 - mcalibValue.X2))*100/1000000.0; /* us/1000000 => s */
            db_DetaTY2 = ((double)(mmeasureValue.Y2 - mcalibValue.Y2))*100/1000000.0; /* us/1000000 => s */
            DBG("db_DetaTX1 = %lf\n",db_DetaTX1);
            DBG("db_DetaTY1 = %lf\n",db_DetaTY1);
            DBG("db_DetaTX2 = %lf\n",db_DetaTX2);
            DBG("db_DetaTY2 = %lf\n",db_DetaTY2);

            db_anpha1 = (db_Vrb1*db_DetaTX1)/db_R;
            db_beta1  = (db_Vrb1*db_DetaTY1)/db_R;
            db_anpha2 = (db_Vrb2*db_DetaTX2)/db_R;
            db_beta2  = (db_Vrb2*db_DetaTY2)/db_R;
            DBG("db_anpha1 = %lf\n",db_anpha1);
            DBG("db_beta1 = %lf\n",db_beta1);
            DBG("db_anpha2 = %lf\n",db_anpha2);
            DBG("db_beta2 = %lf\n",db_beta2);

            db_DetaXSS1 = (2*db_R) * sin(db_anpha1/2) * cos(db_anpha1/2);
            db_DetaYSS1 = (2*db_R) * sin(db_beta1/2) * cos(db_beta1/2);
            db_DetaXRB1 = db_DetaXSS1 * cos(3.142/4);
            db_DetaYRB1 = db_DetaYSS1 * cos(3.142/4);
            DBG("db_DetaXSS1 = %lf\n",db_DetaXSS1);
            DBG("db_DetaYSS1 = %lf\n",db_DetaYSS1);
            DBG("db_DetaXRB1 = %lf\n",db_DetaXRB1);
            DBG("db_DetaYRB1 = %lf\n",db_DetaYRB1);

            db_DetaXSS2 = (2*db_R) * sin(db_anpha2/2) * cos(db_anpha2/2);
            db_DetaYSS2 = (2*db_R) * sin(db_beta2/2) * cos(db_beta2/2);
            db_DetaXRB2 = db_DetaXSS2 * cos(3.142/4);
            db_DetaYRB2 = db_DetaYSS2 * cos(3.142/4);
            DBG("db_DetaXSS2 = %lf\n",db_DetaXSS2);
            DBG("db_DetaYSS2 = %lf\n",db_DetaYSS2);
            DBG("db_DetaXRB2 = %lf\n",db_DetaXRB2);
            DBG("db_DetaYRB2 = %lf\n",db_DetaYRB2);

            db_r1 = sqrt((db_DetaYSS1 * db_DetaYSS1) + (db_DetaXSS1 * db_DetaXSS1));
            db_r2 = sqrt((db_DetaYSS2 * db_DetaYSS2) + (db_DetaXSS2 * db_DetaXSS2));
            DBG("db_r1 = %lf\n",db_r1);
            DBG("db_r2 = %lf\n",db_r2);

            db_LXSS = (atan((db_DetaXSS2 - db_DetaXSS1)/db_D)*180)/3.142;
            db_LYSS = (atan((db_DetaYSS2 - db_DetaYSS1)/db_D)*180)/3.142;
            db_LXRB = (atan((db_DetaXRB2 - db_DetaXRB1)/db_D)*180)/3.142;
            db_LYRB = (atan((db_DetaYRB2 - db_DetaYRB1)/db_D)*180)/3.142;
            DBG("db_LXSS = %lf\n",db_LXSS);
            DBG("db_LYSS = %lf\n",db_LYSS);
            DBG("db_LXRB = %lf\n",db_LXRB);
            DBG("db_LYRB = %lf\n",db_LYRB);

            mdata.coordinates.X = (int16_t)(db_DetaXRB2 * 100.0);
            mdata.coordinates.Y = (int16_t)(db_DetaYRB2 * 100.0);

            mdata.coordinates.R = (int16_t)(db_r2 * 100.0);
            mdata.coordinates.aX = (int16_t)(db_LXRB * 10.0);
            mdata.coordinates.aY = (int16_t)(db_LYRB * 10.0);
            DBG("X (db_DetaXRB2) = %lf\n",db_DetaXRB2);
            DBG("Y (db_DetaYRB2)= %lf\n",db_DetaYRB2);
            DBG("R (db_r2)= %lf\n",db_r2);
            DBG("A (db_LXRB) = %lf\n",db_LXRB);
            DBG("B (db_LYRB)= %lf\n",db_LYRB);
            DBG("X1 (db_DetaXRB1)= %lf\n",db_DetaXRB1);
            DBG("Y1 (db_DetaYRB1)= %lf\n",db_DetaYRB1);
            if(db_r2 > db_a)
            {
                moutput.out2 = _ON;
            }
            else
            {
                moutput.out2 = _OFF;
            }
        }
        io_setOutput(moutput);
    }
}

static void app_Measurement(void)
{
    CycleMeasure cycleMeasure = STOP;
    CycleMeasureSensor cycleMeasureX = SEN_STOP;
    CycleMeasureSensor cycleMeasureY = SEN_STOP;
    CycleMeasureSensor cycleMeasureZ = SEN_STOP;
    SensorChange XStatus = SENSORNOCHANGE;
    SensorChange YStatus = SENSORNOCHANGE;
    GetInputType getInput = GET_SENSOR;

    mdata.coordinates.X = 0;
    mdata.coordinates.Y = 0;
    mdata.coordinates.Z = 0;
    mdata.coordinates.aX = 0;
    mdata.coordinates.aY = 0;
    mdata.mode = NONE;

    mmeasureValue.X1 = 0;
    mmeasureValue.Y1 = 0;
    mmeasureValue.X2 = 0;
    mmeasureValue.Y2 = 0;
    mmeasureValue.Z = 0;

    do
    {
        if(GET_BUTTON == getInput)
        {
            mbutton = io_getButton();
            minput = io_getInput();
        }
        else
        {
            minput = io_getInput();
            msensor = io_getSensor();
        }
        /********************************************## 1 ##*******************************************/
        /*Robot di chuyển tới vị trí P0 Robot xuất tín hiệu cho X10 cho phép quá trình bắt đầu*/
        if((STOP == cycleMeasure) && (_ON == minput.in0)) // X10=ON
        {
            app_ClearAllOutput();
            minput.in0 = _OFF;
            cycleMeasure = CLEARSENSOR;
            getInput = GET_SENSOR;
            timer_Start(TIMER_CLEARSENSOR, TIMERCLEARSENSOR);
            moutput.out0 = _OFF;
            moutput.out1 = _OFF;
            moutput.rl1 = _ON;
            io_setOutput(moutput);
            DBG("cycleMeasure = CLEARSENSOR\n");
        }
        /********************************************## 2 ##*******************************************/
        /********************************************## 3 ##*******************************************/
        /*Waiting clear Sensor*/
        if((CLEARSENSOR == cycleMeasure) && (TIME_FINISH == timer_Status(TIMER_CLEARSENSOR)))
        {
            if((_OFF == msensor.s0) && (_OFF == msensor.s1))
            {
                moutput.rl1 = _OFF;
                moutput.out0 = _ON;
                moutput.out1 = _OFF;
                io_setOutput(moutput);
                msensor.s0 = _OFF;
                msensor.s1 = _OFF;
                getInput = GET_SENSOR;
                cycleMeasure = WAITMEASUREZ;
                DBG("SENSOR OK\n");
            }
            else
            {
                moutput.rl1 = _OFF;
                moutput.out0 = _OFF;
                moutput.out1 = _OFF;
                io_setOutput(moutput);
                cycleMeasure  = ERROR;
                DBG("SENSOR NOT OK\n");
            }
            getInput = GET_SENSOR;
        }
        /********************************************## 4 ##*******************************************/
        while ((WAITMEASUREZ == cycleMeasure) && (0 == GET_IN0))
        {
            minput = io_getInput();
            if(_ON == minput.in1) //C=1
            {
                /*Start couter*/
                timer_Start(TIMER_CLEARSENSOR, TIMEWAITX11);
                minput.in1 = _OFF;
                msensor.s0 = _OFF;
                msensor.s1 = _OFF;
                getInput = GET_SENSOR;
                cycleMeasureZ = SEN_START;
                cycleMeasure = WAITRBSTABLEZ;
                DBG("C=1 START TIME MEASURE Z\n");
            }
        };
        while ((WAITRBSTABLEZ == cycleMeasure) && (0 == GET_IN0)) // Waiting 200ms
        {
            if(TIME_FINISH == timer_Status(TIMER_CLEARSENSOR))
            {
                /*Start couter*/
                timer_Start(TIMER_Z, TIMERMAXVALUE);
                minput.in1 = _OFF;
                msensor.s0 = _OFF;
                msensor.s1 = _OFF;
                getInput = GET_SENSOR;
                cycleMeasureZ = SEN_START;
                cycleMeasure = MEASUREZ;
                DBG("MEASURE Z\n");
            }
        };
        /*Measure Z*/
        while ((MEASUREZ == cycleMeasure) && (0 == GET_IN0))
        {
            msensor = io_getSensor();
            minput = io_getInput();
            if((SEN_START == cycleMeasureZ) && (_ON == msensor.s0))
            {
                /*Stop couter X*/
                mmeasureValue.Z = time_Stop(TIMER_Z);
                cycleMeasureZ = SEN_FINISH;
                getInput = GET_SENSOR;
                cycleMeasure = CHECKZVALUE;
                XStatus = SENSORCHANGE;
                DBG("SENSOR Z = ON\n");
            }
            if(_ON == minput.in1) // C=2
            {
                if(SEN_START == cycleMeasureZ)
                {
                    mmeasureValue.Z = 0;
                }
                cycleMeasureZ = SEN_FINISH;
                getInput = GET_SENSOR;
                cycleMeasure = CHECKZVALUE;
                DBG("C=2 END MEASURE Z\n");
            }
        };
        /********************************************## 5 ##*******************************************/

        while (((CHECKZVALUE == cycleMeasure) || (Z_OK == cycleMeasure) || (Z_NOT_OK == cycleMeasure)) && (0 == GET_IN0))
        {
            msensor = io_getSensor();
            minput = io_getInput();
            if((CHECKZVALUE == cycleMeasure) && (_ON == minput.in1) && (_ON == msensor.s0) && (_ON == msensor.s1)) //C=2
            {
                minput.in1 = _OFF;
                moutput.out1 = _ON;
                io_setOutput(moutput);
                app_GetCurrentMeasureValue();
                (void)time_Stop(TIMER_Z);
                mmeasureValue.X1 =  mCurrentMeasureValue.X1;
                mmeasureValue.Y1 =  mCurrentMeasureValue.Y1;
                mmeasureValue.X2 =  mCurrentMeasureValue.X2;
                mmeasureValue.Y2 =  mCurrentMeasureValue.Y2;
                app_SetCurrentMeasureValue();
                mdata.mode = ZONLY;
                app_CalculatorValue(cycleMeasure, mdata.mode);
                screen_DataMeasure(mdata, msetCalibValue);
                cycleMeasure = Z_OK;
                getInput = GET_BUTTON;
                XStatus = SENSORCHANGE;
                YStatus = SENSORCHANGE;
                DBG("C=2 MEASURE Z OK\n");
            }
            else if((CHECKZVALUE == cycleMeasure) && (_ON == minput.in1) && ((_OFF == msensor.s0) || (_OFF == msensor.s1))) //C=2
            {
                minput.in1 = _OFF;
                moutput.out1 = _OFF;
                io_setOutput(moutput);
                (void)time_Stop(TIMER_Z);
                cycleMeasure = Z_NOT_OK;
                getInput = GET_SENSOR;
                mdata.mode = ZERROR1;
                screen_DataMeasure(mdata, msetCalibValue);
                //HAL_Delay(100);
                DBG("C=2 MEASURE Z NOT OK\n");
            }
            if(((Z_OK == cycleMeasure) || (Z_NOT_OK == cycleMeasure)) && (_OFF == minput.in1))
            {
                cycleMeasure = WAITMEASUREX1Y1;
                DBG("C=2 cycleMeasure = WAITMEASUREX1Y1\n");
            }
        }

        if((SETVALUEZ == cycleMeasure) && (_ON == mbutton.set) && (_ON == moutput.out1))
        {
            mledStatus.led3 = _ON;
            io_setLedStatus(mledStatus);
            cycleMeasure = WAITMEASUREX1Y1;
            getInput = GET_SENSOR;
            DBG("SET Z\n");
            app_SetCalibValue();
            app_GetCalibValue();
        }
        /********************************************## 6 ##*******************************************/
        /*Robot di chuyển tới vị trí P5 Robot xuất tín hiệu cho X11*/
        while (((WAITMEASUREX1Y1 == cycleMeasure) || (SETVALUEZ == cycleMeasure)) && (0 == GET_IN0))
        {
            minput = io_getInput();
            if(_ON == minput.in1) //C=3
            {
                /*Start couter*/
                timer_Start(TIMER_CLEARSENSOR, TIMEWAITX11);
                minput.in1 = _OFF;
                moutput.out0 = _OFF;
                moutput.out1 = _OFF;
                io_setOutput(moutput);
                cycleMeasure = WAITRBSTABLEX1Y1;
                getInput = GET_SENSOR;
                cycleMeasureX = SEN_START;
                cycleMeasureY = SEN_START;
                DBG("cycleMeasure = MEASUREX1Y1 C=3\n");
            }
        };
        while ((WAITRBSTABLEX1Y1 == cycleMeasure) && (0 == GET_IN0))
        {
            if(TIME_FINISH == timer_Status(TIMER_CLEARSENSOR))
            {
                /*Start couter*/
                timer_Start(TIMER_X, TIMERMAXVALUE);
                timer_Start(TIMER_Y, TIMERMAXVALUE);
                minput.in1 = _OFF;
                cycleMeasure = MEASUREX1Y1;
                getInput = GET_SENSOR;
                cycleMeasureX = SEN_START;
                cycleMeasureY = SEN_START;
                DBG("Start couter X1, Y1\n");
            }
        };
        while ((MEASUREX1Y1 == cycleMeasure) && (0 == GET_IN0))
        {
            msensor = io_getSensor();
            minput = io_getInput();
            if((SEN_START == cycleMeasureX) && (_ON == msensor.s0))
            {
                /*Stop couter X*/
                mmeasureValue.X1 = time_Stop(TIMER_X);
                msensor.s0 = _OFF;
                cycleMeasureX = SEN_FINISH;
                getInput = GET_SENSOR;
                XStatus = SENSORCHANGE;
                DBG("X1 = SEN_FINISH\n");
                // DBG("mmeasureValue.X1 = %d\n",mmeasureValue.X1);
            }
            if((SEN_START == cycleMeasureY) && (_ON == msensor.s1))
            {
                /*Stop couter X*/
                mmeasureValue.Y1 = time_Stop(TIMER_Y);
                msensor.s1 = _OFF;
                cycleMeasureY = SEN_FINISH;
                getInput = GET_SENSOR;
                YStatus = SENSORCHANGE;
                DBG("Y1 = SEN_FINISH\n");
                // DBG("mmeasureValue.Y1 = %d\n",mmeasureValue.Y1);
            }
            if((SEN_FINISH == cycleMeasureX) && (SEN_FINISH == cycleMeasureY))
            {
                cycleMeasureX = SEN_STOP;
                cycleMeasureY = SEN_STOP;
                cycleMeasure = WAITMEASUREX2Y2;
                getInput = GET_SENSOR;
                DBG("cycleMeasure = WAITMEASUREX2Y2\n");
            }
            //if(_ON == minput.in1) //C=4
            //{
            //    if(SEN_START == cycleMeasureX)
            //    {
            //        mmeasureValue.X1 = 0;
            //    }
            //    if(SEN_START == cycleMeasureY)
            //    {
            //        mmeasureValue.Y1 = 0;
            //    }
            //    cycleMeasureX = SEN_STOP;
            //    cycleMeasureY = SEN_STOP;
            //    cycleMeasure = WAITMEASUREX2Y2;
            //    getInput = GET_SENSOR;
            //    (void)time_Stop(TIMER_X);
            //    (void)time_Stop(TIMER_Y);
            //    DBG("MEASUREX1Y1 == cycleMeasure C=4\n");
            //}
        };
        /********************************************## 7 ##*******************************************/
        /*Robot di chuyển tới vị trí P10 Robot xuất tín hiệu cho X11*/
        while ((WAITMEASUREX2Y2 == cycleMeasure) && (0 == GET_IN0))
        {
            minput = io_getInput();
            if(_ON == minput.in1) // C=4
            {
                /*Start couter*/
                timer_Start(TIMER_CLEARSENSOR, TIMEWAITX11);
                minput.in1 = _OFF;
                cycleMeasure = WAITRBSTABLEX2Y2;
                getInput = GET_SENSOR;
                cycleMeasureX = SEN_START;
                cycleMeasureY = SEN_START;
                DBG("cycleMeasure = WAITMEASUREX2Y2 C=4\n");
            }
        };
        while ((WAITRBSTABLEX2Y2 == cycleMeasure) && (0 == GET_IN0))
        {
            if(TIME_FINISH == timer_Status(TIMER_CLEARSENSOR))
            {
                /*Start couter*/
                timer_Start(TIMER_X, TIMERMAXVALUE);
                timer_Start(TIMER_Y, TIMERMAXVALUE);
                minput.in1 = _OFF;
                cycleMeasure = MEASUREX2Y2;
                getInput = GET_SENSOR;
                cycleMeasureX = SEN_START;
                cycleMeasureY = SEN_START;
                DBG("Start couter X2, Y2\n");
            }
        };
        while ((MEASUREX2Y2 == cycleMeasure) && (0 == GET_IN0))
        {
            msensor = io_getSensor();
            if((SEN_START == cycleMeasureX) && (_ON == msensor.s0))
            {
                /*Stop couter X*/
                mmeasureValue.X2 = time_Stop(TIMER_X);
                msensor.s0 = _OFF;
                cycleMeasureX = SEN_FINISH;
                getInput = GET_SENSOR;
                XStatus = SENSORCHANGE;
                DBG("X2 = SEN_FINISH\n");
            }
            if((SEN_START == cycleMeasureY) && (_ON == msensor.s1))
            {
                /*Stop couter X*/
                mmeasureValue.Y2 = time_Stop(TIMER_Y);
                msensor.s1 = _OFF;
                cycleMeasureY = SEN_FINISH;
                getInput = GET_SENSOR;
                YStatus = SENSORCHANGE;
                DBG("Y2 = SEN_FINISH\n");
            }
            if((SEN_FINISH == cycleMeasureX) && (SEN_FINISH == cycleMeasureY))
            {
                /*Save D_X1, D_Y1*/
                cycleMeasureX = SEN_STOP;
                cycleMeasureY = SEN_STOP;
                cycleMeasure = WAITSETVALUE;
                getInput = GET_BUTTON;
                DBG("WAITSETVALUE == cycleMeasure\n");
            }
            // if(_ON == minput.in1) //C=5
            // {
            //     if(SEN_START == cycleMeasureX)
            //     {
            //         mmeasureValue.X2 = 0;
            //     }
            //     if(SEN_START == cycleMeasureY)
            //     {
            //         mmeasureValue.Y2 = 0;
            //     }
            //     cycleMeasureX = SEN_STOP;
            //     cycleMeasureY = SEN_STOP;
            //     cycleMeasure = CALCULATORVALUE;
            //     getInput = GET_SENSOR;
            //     (void)time_Stop(TIMER_X);
            //     (void)time_Stop(TIMER_Y);
            // }
        };

        /*Robot di chuyển tới vị trí P18*/
        if((WAITSETVALUE == cycleMeasure) && (_ON == mbutton.set))
        {
            app_GetCalibValue();
            if ((0 == mcalibValue.X1) && (0 == mcalibValue.X2) && (0 == mcalibValue.Y1) && (0 == mcalibValue.Y2) && (0 == mcalibValue.Z) && (ZERROR1 != mdata.mode))
            {
                app_SetCalibValue();
                app_GetCalibValue();
                mledStatus.led3 = _ON;
                msetCalibValue = CALIBSET;
                io_setLedStatus(mledStatus);
            }
            getInput = GET_SENSOR;
            cycleMeasure = CALCULATORVALUE;
            DBG("cycleMeasure = CALCULATORVALUE\n");
        }
        /********************************************## 8 ##*******************************************/
        if(((CALCULATORVALUE == cycleMeasure) || (WAITSETVALUE == cycleMeasure)) && (_ON == minput.in1)) // C=5
        {
            minput.in1 = _OFF;
            cycleMeasure = FINISH;
            getInput = GET_BUTTON;
            /*Calculator Value - Printf to screen*/
            app_SetCurrentMeasureValue();
            if(ZONLY == mdata.mode)
            {
                mdata.mode = MEASUREALL;
            }
            else if(ZERROR1 == mdata.mode)
            {
                mdata.mode = ZERROR2;
            }
            app_CalculatorValue(cycleMeasure,  mdata.mode);
            screen_DataMeasure(mdata, msetCalibValue);
            DBG("cycleMeasure = FINISH C=5\n");
        }
    } while (0 == GET_IN0);
    app_TrigerOutputOFF();
    moutput.out0 = _OFF;
    moutput.out1 = _OFF;
    moutput.rl1  = _OFF;
    moutput.rl2  = _OFF;

    if((SENSORNOCHANGE == XStatus) || (SENSORNOCHANGE == YStatus))
    {
        moutput.out1 = _ON;
        DBG("SENSOR IS NOT CHANGE\n");
    }
    else
    {
        moutput.out1 = _OFF;
        DBG("SENSOR CHANGE\n");
    }
    io_setOutput(moutput);
    app_TrigerOutputON();
    if((NONE != mdata.mode) && (CALIBSET == msetCalibValue))
    {
        eep_WriteDataMeasure(&mdata);
    }
}

/*public function*/
void app_Init(void)
{
    Serial.begin(57600);
    Wire.begin();
    io_Init();
    lcd_Int();
    timer_Init();
    lcd_Backlight();
    rtc_Init();
    eep_Init();
    app_GetCalibValue();
    if((mcalibValue.X1 == 0) && (mcalibValue.X2 == 0) && (mcalibValue.Y1 == 0) && (mcalibValue.Y2 == 0) && (mcalibValue.Z == 0))
    {
        mledStatus.led3 = _OFF;
        msetCalibValue = CALIBRESET;
    }
    else
    {
        mledStatus.led3 = _ON;
        msetCalibValue = CALIBSET;
    }
    io_setLedStatus(mledStatus);

    screen_Clear();

    if(2000 == rtc_Now().year)
    {
        rtc_SetDateTime(mtime);
    }
    if(CALIBSET == msetCalibValue)
    {
        app_GotoMainScreen(CALIBSET);
    }
    else
    {
        app_GotoMainScreen(CALIBRESET);
    }

    app_TrigerOutputON();
}

void app_loop(void)
{
    uint32_t time = 0;
    mbutton = io_getButton();
    minput = io_getInput();
    if((_ON == mbutton.next) && (_ON == mbutton.set))
    {
        do
        {
            mbutton = io_getButton();
        } while((_ON == mbutton.next) && (_ON == mbutton.set));
        app_SettingRtc();
    }
    if(_ON == mbutton.his)
    {
        while(_ON == io_getButton().his);
        app_HisValue();
    }
    if(_ON == minput.in0)
    {
        minput.in0 = _OFF;
        app_Measurement();
    }
    if(_ON == mbutton.reset)
    {
        mbutton.reset = _OFF;
        timer_Start(TIMER_CLEARCALIB,TIMERCLEARCALIB);
        do
        {
            /* code */
        } while (_ON == io_getButton().reset);
        time = time_Stop(TIMER_CLEARCALIB);

        if((time >= 10000) && (time <= 30000))
        {
            app_TrigerOutputOFF();
            moutput.out2 = _OFF;
            moutput.out3 = _OFF;
            moutput.out1 = _OFF;
            moutput.out0 = _OFF;
            moutput.rl1 = _OFF;
            moutput.rl2 = _OFF;
            io_setOutput(moutput);
            app_TrigerOutputON();
        }
        else if((time >= 50000))
        {
            MeasureValue vl={0,0,0,0,0};
            eep_WriteDataCalib(&vl);
            eep_ReadDataCalib(&mcalibValue);
            DBG("eep_WriteDataCalib(&vl)");
            mledStatus.led3 = _OFF;
            msetCalibValue = CALIBRESET;
            io_setLedStatus(mledStatus);
            app_GotoMainScreen(CALIBRESET);
        }
    }
}
