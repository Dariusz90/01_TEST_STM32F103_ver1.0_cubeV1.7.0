/*
 * common_fun.h
 *
 *  Created on: 20 cze 2018
 *      Author: DBabraj
 */

#ifndef COMMON_FUN_H_
#define COMMON_FUN_H_

/* Includes (zalaczone biblioteki) -----------------------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "dma.h"
#include "usart.h"
#include "rtc.h"
#include "spi.h"
#include "gpio.h"

#include "DB_soft_timer.h"
#include "DB_GPS_NMEA_Simple.h"
#include "DB_GSM_SIM800.h"
#include "DB_RFM98W.h"

#include <stdbool.h>	// dla bool itp.


/* Preprocessor definition (definicje preprocesora) ------------------------------------------------------------------*/

/* Types definition (definicje typow) --------------------------------------------------------------------------------*/
/**************************************************
 * SOFT TIMER DATA
 **************************************************/
typedef enum{
	ST_MAIN_LOOP,

	ST_SIZE	// MAX SIZE OF SOFT TIMER DATA
}soft_timer_t;

/* Const declarations (deklaracje stalych) ---------------------------------------------------------------------------*/

/* Variable declarations (deklaracje zmiennych) ----------------------------------------------------------------------*/
extern RING_BUFFER_DATA uart2_buffer_gps;
extern RING_BUFFER_DATA uart3_buffer_gsm;


/**************************************************
 * SOFT TIMER DATA
 **************************************************/
extern volatile uint16_t soft_timer[ST_SIZE];

/**************************************************
 * RFM98W DATA
 **************************************************/
extern RFM98_data_t RFM98_data;

/* Function declarations (deklaracje funkcji) ------------------------------------------------------------------------*/
void GPS_GSM_Tracker_init(void);
bool GPS_GSM_TrackerTask(void);

void RTC_SetInterval(int Hours);
bool RTC_WakeUp(void);
void RTC_GoSleep(void);

/* Function definition (definicje funkcji) ---------------------------------------------------------------------------*/

#endif /* COMMON_FUN_H_ */
