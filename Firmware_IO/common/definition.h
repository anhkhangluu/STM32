#ifndef _DEFINITION_H_
#define _DEFINITION_H_
//#include "structer.h"

#define EEP_ADDRESS                 0x57
#define EEP_CURRENT_INDEX_ADDRESS   5U
#define EEP_CALIB_ADDRESS           10U
#define EEP_CURREN_VALUE_ADDRESS    30U
#define EEP_START_ADDRESS           50U //(EEP_CURREN_VALUE_ADDRESS + sizeof(MeasureValue))
#define EEP_MAX_DATA                10U

#define LCD_LINE_SIZE               16U
#define _ON                         1U
#define _OFF                        0U

#define TIMER_X                     0U
#define TIMER_Y                     1U
#define TIMER_Z                     0U
#define TIMER_CLEARSENSOR           2U
//#define TIMER_ERRORSENSOR			3U
#define TIMER_CLEARCALIB            2U
#define TIMERCLEARSENSOR            2000U * 10U *10
#define TIMERERRORSENSOR			5000U * 10U *10
#define TIMERCLEARCALIB             6000U * 10U *10
#define TIMERY0ON                   1500U * 10U *10
#define TIMEWAITX11                 200U * 10U *10
#define TIMEWAITSTABLE				200U * 10U *10
#define TIMERMAXVALUE               0xFFFFFFFFU

#define DEBUG 1
#if(DEBUG == 1)
//#define DBG(x...) {fprintf(stderr,"%s<%d> ",__FUNCTION__,__LINE__); fprintf(stderr,x);fflush(stderr);}
//#define WARN(x...) {fprintf(stderr,"(W) %s<%d> ",__FUNCTION__,__LINE__); fprintf(stderr,x);fflush(stderr);}
//#define ERR(x...) {fprintf(stderr,"(E) %s<%d> ",__FUNCTION__,__LINE__); fprintf(stderr,x);fflush(stderr);}
#else
#define DBG(x...) {}
#define WARN(x...) {}
#define ERR(x...) {}
#endif
#endif
