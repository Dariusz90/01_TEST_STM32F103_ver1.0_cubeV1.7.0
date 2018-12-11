/*
 * DB_GPS_NMEA_Simple.c
 *
 *  Created on: 22 cze 2018
 *      Author: DBabraj
 */

/* Includes (zalaczone biblioteki) -----------------------------------------------------------------------------------*/
#include "DB_GPS_NMEA_Simple.h"
#include <string.h>	// dla memset()
#include <stdlib.h> // dla atoi()
#include <stdbool.h>	// dla bool itp.

#ifdef FREERTOS
#include "cmsis_os.h"
#endif

/* Preprocessor definition (definicje preprocesora) ------------------------------------------------------------------*/
#ifdef FREERTOS
#define GPS_DELAY_MS(_ms)	osDelay(_ms)
#else
#define GPS_DELAY_MS(_ms)	HAL_Delay(_ms)
#endif

#define NMEA_START_CHAR		'$'
#define NMEA_END_CHAR_0		'\r'
#define NMEA_END_CHAR_1		'\n'
#define NMEA_CRC_CHAR		'*'
#define NMEA_MAX_LENGTH 	70

/* KONFIGURACJA: DEBUG UART */
//#define DEBUG_MODE_UART

#define UART_TRANSMIT_TIMEOUT	100	// [ms]


/* Types definition (definicje typow) --------------------------------------------------------------------------------*/

/* Variable declarations (deklaracje zmiennych) ----------------------------------------------------------------------*/


/* Function declarations (deklaracje funkcji) ------------------------------------------------------------------------*/
static uint8_t processGPS(GPS_NMEA_GLL_t *_data, uint8_t c);

/* Function definition (definicje funkcji) ---------------------------------------------------------------------------*/

/**
  * @brief  Set GPS message type to output.
  * @param  *_data:	Pointer to GPS_NMEA_GLL_t data structure.
  * @param  *_buffer:	Pointer to RING_BUFFER_DATA data structure.
  * @retval none.
  */

#define sizeof_member(type,member) sizeof(((type*)0)->member)

static bool safe_strncpy(void * dst, const void * src, size_t bufsize)
{
    if (bufsize == 0)
        return false;

    while ( (bufsize > 1) && *(uint8_t*)src != 0)
    {
        *(uint8_t*)dst++ = *(uint8_t*)src++;
        bufsize --;
    }
    *(uint8_t*)dst = 0;
    return true;
}


void GPS_NMEA_init_simple(GPS_NMEA_GLL_t *_data, RING_BUFFER_DATA *_buffer)	//	GPS_SetMessageOutput
{
	_data->buffer = _buffer;
	_data->fpos = 0;
	_data->nmea_state = GPS_START_CHAR_STATE;
	_data->gpsInit_flag = true;
}


/**
  * @brief  Read frame from UART/ring buffer.
  * @param  *_data:	Pointer to GPS_NMEA_GLL_t data structure.
  * @retval Message Type - true/false.
  */
uint8_t GPS_NMEA_readFrame_simple(GPS_NMEA_GLL_t *_data)
{
	if( !_data->gpsInit_flag ) return false;
	uint8_t msgType = false;
	uint16_t size = ringBuffer_checkAvailable(_data->buffer);

	while ( size-- )
	{
		msgType = processGPS(_data, ringBuffer_read(_data->buffer));
#if 1
		if(msgType != false)
			break;	//	poprawnie odebrano cala ramke
#endif
	}
#if defined(HAL_UART_MODULE_ENABLED) && defined(DEBUG_MODE_UART)
	if( msgType )
	{
#if 0
		HAL_UART_Transmit(_data->buffer->huart, (uint8_t *)_data->lat, strlen(_data->lat), UART_TRANSMIT_TIMEOUT);
		HAL_UART_Transmit(_data->buffer->huart, (uint8_t *)_data->NS, strlen(_data->NS), UART_TRANSMIT_TIMEOUT);
		HAL_UART_Transmit(_data->buffer->huart, (uint8_t *)_data->lon, strlen(_data->lon), UART_TRANSMIT_TIMEOUT);
		HAL_UART_Transmit(_data->buffer->huart, (uint8_t *)_data->EW, strlen(_data->EW), UART_TRANSMIT_TIMEOUT);
		HAL_UART_Transmit(_data->buffer->huart, (uint8_t *)_data->time, strlen(_data->time), UART_TRANSMIT_TIMEOUT);
		HAL_UART_Transmit(_data->buffer->huart, (uint8_t *)_data->status, strlen(_data->status), UART_TRANSMIT_TIMEOUT);
		HAL_UART_Transmit(_data->buffer->huart, (uint8_t *)"\r\n", strlen("\r\n"), UART_TRANSMIT_TIMEOUT);
#else
		HAL_UART_Transmit(_data->buffer->huart, (uint8_t *)_data->GPGLL, strlen(_data->GPGLL), UART_TRANSMIT_TIMEOUT);
		HAL_UART_Transmit(_data->buffer->huart, (uint8_t *)"\r\n", strlen("\r\n"), UART_TRANSMIT_TIMEOUT);
		//	Google Maps: http://maps.google.com/maps?q=43.508083,-80.190653
#endif
	}
#endif /* HAL_UART_MODULE_ENABLED */

	return msgType;
}


/**
  * @brief  Processing incoming serial bytes.
  * @param  *_pvt: 	Pointer to frame message data structure.
  * @param  _pvtSize:	Size of message data structure.
  * @param  c:	Byte reading from UART/ring buffer.
  * @retval true/false.
  */
static uint8_t processGPS(GPS_NMEA_GLL_t *_data, uint8_t c)
{
	uint8_t read_flag = false;

	switch(_data->nmea_state)
	{
		case GPS_START_CHAR_STATE:
			_data->fpos = 0;
			_data->crc = 0;
			if(c == NMEA_START_CHAR)
			{
				_data->nmeaByte[_data->fpos] = (char)c;
				_data->fpos++;
				_data->nmea_state = GPS_DATA_STATE;
			}
			break;
		case GPS_DATA_STATE:
			_data->nmeaByte[_data->fpos] = (char)c;
			_data->fpos++;

			if( c != NMEA_CRC_CHAR )
			{
				// Is this the first value for the checksum?
				if(_data->crc == 0)
				{
					// Yes. Set the checksum to the value
					_data->crc = c;
				}
				else
				{
					// No. XOR the checksum with this character's value
					_data->crc = _data->crc ^ c;
				}
			}
			if(c == NMEA_CRC_CHAR)
			{
				_data->nmea_state = GPS_CHECSUM_STATE;
			}
			break;
		case GPS_CHECSUM_STATE:
			_data->nmeaByte[_data->fpos] = (char)c;
			_data->fpos++;
			if(c == NMEA_END_CHAR_1)
			{
				_data->nmea_state = GPS_START_CHAR_STATE;
#warning niebezpieczna funkcja strtol - nie sprawdza maksymalnej d³ugoœci bufora!
				uint8_t num_crc = (int)strtol(_data->nmeaByte + _data->fpos-4, NULL, 16);
				if( _data->crc == num_crc )
				{

					const char *gpgll = "$GPGLL";
					if( !memcmp((char*)_data->nmeaByte ,gpgll , strlen( gpgll )) )
					{
					    safe_strncpy(_data->nmeaByte_mem, _data->nmeaByte, sizeof_member(GPS_NMEA_GLL_t, nmeaByte) - 1);

//#warning skrajnie niebezpieczna funkcja memcpy - nie sprawdza maksymalnej d³ugoœci bufora!
					    // memcpy(_data->nmeaByte_mem, _data->nmeaByte, strlen(_data->nmeaByte));

					    // Parse text data
						// Returns first token
						if( _data->nmeaByte[_data->fpos-8] == 'A' )
						{
//							memcpy(_data->nmeaByte_mem, _data->nmeaByte, strlen(_data->nmeaByte));


#warning niebezpieczna funkcja strtok - nie sprawdza maksymalnej d³ugoœci bufora!
							strtok((char*)_data->nmeaByte_mem, ",");	//	$GPGLL,
							_data->lat = strtok(NULL, ",");
							_data->NS = strtok(NULL, ",");
							_data->lon = strtok(NULL, ",");
							_data->EW = strtok(NULL, ",");
							_data->time = strtok(NULL, ",");
							_data->status = strtok(NULL, ",");

						}
#warning niebezpieczna funkcja strtok - nie sprawdza maksymalnej d³ugoœci bufora!
						_data->GPGLL = strtok((char*)_data->nmeaByte, "\r\n");	//	$GPGLL,
						read_flag = true;
					}

				}
			}
			break;
		default:
			_data->nmea_state = GPS_START_CHAR_STATE;
			break;
	}

	return read_flag;
}
