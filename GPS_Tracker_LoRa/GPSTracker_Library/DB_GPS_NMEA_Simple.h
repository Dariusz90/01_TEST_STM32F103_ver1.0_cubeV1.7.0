/*
 * DB_GPS_NMEA_Simple.h
 *
 *  Created on: 22 cze 2018
 *      Author: DBabraj
 */

#ifndef DB_GPS_NMEA_SIMPLE_H_
#define DB_GPS_NMEA_SIMPLE_H_

/* Includes (zalaczone biblioteki) -----------------------------------------------------------------------------------*/
#ifdef USE_HAL_DRIVER
#include "DB_stm_HAL_drivers.h"	// do obslugi funkcji z bibliotek STM32 HAL
#endif /* USE_HAL_DRIVER */

#include "DB_ring_buffer.h"	// dla RING_BUFFER_DATA


/* Preprocessor definition (definicje preprocesora) ------------------------------------------------------------------*/

/* Types definition (definicje typow) --------------------------------------------------------------------------------*/

typedef enum{
	GPS_START_CHAR_STATE,
	GPS_DATA_STATE,
	GPS_CHECSUM_STATE
}gps_nmea_state_t;

//	Latitude and longitude, with time of position fix and status
typedef struct{
	RING_BUFFER_DATA *buffer;
	gps_nmea_state_t nmea_state;
	int16_t fpos;
	uint8_t crc;
	uint8_t gpsInit_flag;

	char *lat;		//	Latitude (degrees & minutes),
	char *NS;		//	North/South indicator
	char *lon;		//	Longitude (degrees & minutes)
	char *EW;		//	East/West indicator
	char *time;		//	UTC time
	char *status;	//	V = Data invalid or receiver warning, A = Data valid.
	char *GPGLL;	//	GPGLL message

	char nmeaByte[72];
	char nmeaByte_mem[72];
}GPS_NMEA_GLL_t;

/* Const declarations (deklaracje stalych) ---------------------------------------------------------------------------*/
/* Variable declarations (deklaracje zmiennych) ----------------------------------------------------------------------*/

/* Function declarations (deklaracje funkcji) ------------------------------------------------------------------------*/
void GPS_NMEA_init_simple(GPS_NMEA_GLL_t *_data, RING_BUFFER_DATA *_buffer);

uint8_t GPS_NMEA_readFrame_simple(GPS_NMEA_GLL_t *_data);

/* Function definition (definicje funkcji) ---------------------------------------------------------------------------*/



#endif /* DB_GPS_NMEA_SIMPLE_H_ */
