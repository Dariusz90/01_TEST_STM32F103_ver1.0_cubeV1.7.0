/*
 * DB_GSM_SIM800.c
 *
 *  Created on: 26 cze 2018
 *      Author: DBabraj
 */

/* Includes (zalaczone biblioteki) -----------------------------------------------------------------------------------*/
#include "DB_GSM_SIM800.h"
#include <stdbool.h>	// dla bool itp.
#include <string.h>
#include <stdlib.h>	// dla strtof()

#include "DB_string.h"	//	dla ftoa_fast()

/* Preprocessor definition (definicje preprocesora) ------------------------------------------------------------------*/

/* KONFIGURACJA -> REPLY FROM GSM MODULE ON */
#define GSM_REPLY_FLAG_ENABLE
/* KONFIGURACJA: DEBUG UART */
//#define GSM_DEBUG_MODE_UART

#define UART_TRANSMIT_TIMEOUT	100	// [ms]

#define GSM_REPLY_TIME	1000


/* Types definition (definicje typow) --------------------------------------------------------------------------------*/

/* Const declarations (deklaracje stalych) ---------------------------------------------------------------------------*/

const char *gsmAtSmsCmd[] = {
		"AT\r\n",
		"AT+CMGF=1\r\n",				//	Set the format of messages to Text mode
		"AT+CMGS=\"18825271704\"\r"		// Send SMS to the corresponding phone		// "AT+CMGS=\"18825271704\"\r\n"
};

const char *gsmSmsCmd[] = {
		"\x1a",	//	0x1a : send			"\x1a\r\n"
		"\x1b",	//	0x1b : Cancel send	"\x1b\r\n"
};

//expected response
const char *gsmAtReply[] =
{
		"OK", 			// "AT\r\r\nOK\r\n"		AT z Echo
		"OK", 			// "\r\nOK\r\n"			AT,	AT+CMGF=1
		"OK", 			// "ATE0\r\r\nOK\r\n"	ATE0
		"SMS Ready",	// "\r\nOK\r\n\r\n+CPIN: READY\r\nCall Ready\r\n\r\nSMS Ready\r\n"	AT+CPIN=3363
		">",			// "\r\n> "				AT+CMGS="18825271704"
		"+CMGS"			// "\r\n+CMGS: "		SMStxt\x1a
};

const enum{
	GSM_REPLY_AT_ECHO,
	GSM_REPLY_AT_OR_ATCMGF,
	GSM_REPLY_ATE0,
	GSM_REPLY_ATCPIN,
	GSM_REPLY_ATCMGS,
	GSM_REPLY_SMSTXT
}gsmAtReply_state;

/* Variable declarations (deklaracje zmiennych) ----------------------------------------------------------------------*/

#ifdef GSM_DEBUG_MODE_UART
extern RING_BUFFER_DATA uart2_buffer_gps;
#endif

/* Function declarations (deklaracje funkcji) ------------------------------------------------------------------------*/

/* Function definition (definicje funkcji) ---------------------------------------------------------------------------*/

/**
 * @brief  GSM init.
 * @param  *_data:	Pointer to GSM_SIM800_t data.
 * @param  *_buffer:	Pointer to RING_BUFFER_DATA data.
 * @param  _initRepeat_cnt:	Repeat counter.
 * @param  _delay_ms:	Repeat delay.
 * @retval init_flag: true/false
 */
uint8_t gsm_init(GSM_SIM800_t *_data, RING_BUFFER_DATA *_buffer, uint8_t _initRepeat_cnt, uint32_t _delay_ms)
{
	// GSM Init
	//	Enable GSM Module
	HAL_GPIO_WritePin(QB_GSM_PWR_ON_GPIO_Port, QB_GSM_PWR_ON_Pin, GPIO_PIN_SET);
	HAL_Delay(3000);

	//	Start GSM - Key Push
	//HAL_GPIO_WritePin(QB_GSM_PWR_KEY_GPIO_Port, QB_GSM_PWR_KEY_Pin, GPIO_PIN_SET);
	HAL_Delay(3000);
	//HAL_GPIO_WritePin(QB_GSM_PWR_KEY_GPIO_Port, QB_GSM_PWR_KEY_Pin, GPIO_PIN_RESET);

	_data->buffer = _buffer;

#if 1
	do
	{
		if( gsm_sendCMD(_data, "AT\r\n", gsmAtReply[GSM_REPLY_AT_ECHO], GSM_REPLY_TIME) )				// first AT cmd
		{
			if( gsm_sendCMD(_data, "ATE0\r\n", gsmAtReply[GSM_REPLY_AT_OR_ATCMGF], GSM_REPLY_TIME)	)	// echo off
			{
				if( gsm_sendCMD(_data, "AT+CGATT?\r\n", "+CGATT: 0", 1000) || gsm_sendCMD(_data, "AT+CGATT?\r\n", "ERROR", 1000) )	//	Check if the MS is not connected to the GPRS network
				{
					if( gsm_sendCMD(_data, "AT+CPIN=3363\r\n", gsmAtReply[GSM_REPLY_ATCPIN], 6000) )		// enter PIN "AT+CPIN=3363\r\n"
					{
						_data->gsmInit_flag = true;
					}
				}
				else
				{
					_data->gsmInit_flag = true;
				}
			}

		}
		_initRepeat_cnt--;
		HAL_Delay(_delay_ms);

	}while( _initRepeat_cnt && !_data->gsmInit_flag);
#else
	gsm_sendCMD(_data);				// first AT cmd
	gsm_sendCMD(_data, "ATE0\r\n", gsmAtReply[GSM_REPLY_AT_OR_ATCMGF], GSM_REPLY_TIME);		// echo off
	gsm_sendCMD(_data, "AT+CPIN=3363\r\n", gsmAtReply[GSM_REPLY_ATCPIN], 6000);				// enter PIN	"AT+CPIN=3363\r\n"
	_data->gsmInit_flag = true;
#endif

	return _data->gsmInit_flag;
}


/**
 * @brief  GSM send SMS message.
 * @param  *_data:	Pointer to GSM_SIM800_t data.
 * @param  *_msg:	Pointer to text message.
 * @param  *_phoneNumber:	Pointer to text phone number.
 * @retval send_flag: true/false
 */
uint8_t gsm_sendSMS(GSM_SIM800_t *_data, char *_msg, char *_phoneNumber)
{
	if(!_data->gsmInit_flag) return false;
	uint8_t send_flag = false;
	uint8_t break_flag = false;
	char *txBuff = (char*)_data->buffer->TxData;

	do{
		switch(_data->gsmMsg_state)
		{
			case GSM_STATE_SEND_CMGF:
				strcpy( txBuff, "AT+CMGF=1\r\n");	// Set SMS to TEXT mode
			#ifdef GSM_REPLY_FLAG_ENABLE
				if( gsm_sendCMD(_data, txBuff, gsmAtReply[GSM_REPLY_AT_OR_ATCMGF], GSM_REPLY_TIME) )	//	// Set SMS to TEXT mode
				{
					_data->gsmMsg_state = GSM_STATE_SEND_CMGS;
				}
				else
				{
					break_flag = true;
				}
			#else
				gsm_sendCMD(txBuff, gsmAtReply[GSM_REPLY_AT_OR_ATCMGF], GSM_REPLY_TIME);
				_data->gsmMsg_state = GSM_STATE_SEND_CMGS;
			#endif

				break;
			case GSM_STATE_SEND_CMGS:
				strcpy( txBuff, "AT+CMGS=\"");		// Set the number of receiver
				strcat( txBuff + strlen(txBuff), _phoneNumber );
				strcat( txBuff + strlen(txBuff), "\"\r\n" );	// "\"\r\n"
			#ifdef GSM_REPLY_FLAG_ENABLE
				if( gsm_sendCMD(_data, txBuff, gsmAtReply[GSM_REPLY_ATCMGS], GSM_REPLY_TIME) )	// Set the number of receiver
				{
					_data->gsmMsg_state = GSM_STATE_SEND_MSG;
				}
				else
				{
					break_flag = true;
				}
			#else
				gsm_sendCMD(_data, txBuff, gsmAtReply[GSM_REPLY_ATCMGS], GSM_REPLY_TIME);
				_data->gsmMsg_state = GSM_STATE_SEND_MSG;
			#endif /* GSM_REPLY_FLAG_ENABLE */

				break;
			case GSM_STATE_SEND_MSG:
				strcpy( txBuff, _msg);
				strcat( txBuff + strlen(txBuff), gsmSmsCmd[0] );
			#ifdef GSM_REPLY_FLAG_ENABLE
				// if module reply "+CNGS: 174"
				if( gsm_sendCMD(_data, txBuff, gsmAtReply[GSM_REPLY_SMSTXT], 5000) )	// Send text message
				{
					send_flag = true;
				}
				break_flag = true;
			#else
				gsm_sendCMD(_data, txBuff, gsmAtReply[GSM_REPLY_SMSTXT], 5000);
				send_flag = true;
				break_flag = true;
				_data->gsmMsg_state = GSM_STATE_SEND_CMGF;
			#endif /* GSM_REPLY_FLAG_ENABLE */

				break;
			default:
				_data->gsmMsg_state = GSM_STATE_SEND_CMGF;
				break;
		}
		if( break_flag )
		{
			_data->gsmMsg_state = GSM_STATE_SEND_CMGF;
		}
	}while( !break_flag );

	if( !send_flag )
	{
		_data->gsmSendError_cnt++;
	}

	return send_flag;
}

/**
 * @brief  GSM send CMD.
 * @param  *_data:	Pointer to GSM_SIM800_t data.
 * @param  *_msg:	Pointer to text cmd.
 * @param  *_gsmAtReply:	Pointer to text gsmAtReply.
 * @param  _replyTime:	Timeout.
 * @retval cmdOK_flag: true/false
 */
uint8_t gsm_sendCMD(GSM_SIM800_t *_data, const char *_cmd, const char *_gsmAtReply, const uint32_t _replyTime)
{
	uint8_t cmdOK_flag = false;
	char rxBuff[128] = {0};

	#ifdef HAL_UART_MODULE_ENABLED
	HAL_UART_Transmit(_data->buffer->huart, (uint8_t *)_cmd, strlen(_cmd), UART_TRANSMIT_TIMEOUT);
	#endif

	HAL_Delay(_replyTime);

	if( strlen(_gsmAtReply) )
	{
		ringBuffer_readBytes( _data->buffer, (uint8_t*)rxBuff, ringBuffer_checkAvailable(_data->buffer) );
	#if 0
		if( !memcmp(rxBuff ,_gsmAtReply, strlen( _gsmAtReply )) )
		{
			cmdOK_flag = true;
		}
	#else
		char *s = strstr(rxBuff, _gsmAtReply);	// search for string "_gsmAtReply" in buff
		if(s != NULL)                     		// if successful then s now points at "_gsmAtReply"
		{
			cmdOK_flag = true;
		}
	#endif

	#if defined(GSM_DEBUG_MODE_UART) && defined(HAL_UART_MODULE_ENABLED)
		HAL_UART_Transmit(uart2_buffer_gps.huart, (uint8_t *)rxBuff, strlen(rxBuff), UART_TRANSMIT_TIMEOUT);
		HAL_UART_Transmit(uart2_buffer_gps.huart, (uint8_t *)"\r\n", strlen("\r\n"), UART_TRANSMIT_TIMEOUT);
	#endif
	}
	else
	{
		cmdOK_flag = true;
	}

	return cmdOK_flag;
}


/**
 * @brief  GSM send GPS location message.
 * @param  *_data:	Pointer to GSM_SIM800_t data.
 * @param  *_lat:	Latitude (degrees & minutes) text message.
 * @param  *_lon:	Longitude (degrees & minutes) text message.
 * @param  *_phoneNumber:	Pointer to text phone number.
 * @retval send_flag: true/false
 */
uint8_t gsm_sendGPS_NMEA_location(GSM_SIM800_t *_data, char *_lat, char *_lon, char *_phoneNumber)
{
	char txBuff[128] = {0};

	//	NMEA (GPS) sentence to a map location:
	//	https://stackoverflow.com/questions/12263599/nmea-gps-sentence-to-a-map-location
#if 1
	float latitude = atof(_lat);
	float longitude = atof(_lon);
#else
	float latitude = strtof(_lat, NULL);
	float longitude = strtof(_lon, NULL);
#endif
	int latitude_int = latitude / 100;
	int longitude_int = longitude / 100;
	latitude = (float)latitude_int + (((float)latitude - (float)latitude_int*100) / 60.0);
	longitude = (float)longitude_int + (((float)longitude - (float)longitude_int*100) / 60.0);

	//	Google Maps: http://maps.google.com/maps?q=43.508083,-80.190653
#if 0
	snprintf(txBuff, sizeof(txBuff), "http://maps.google.com/maps?q=%.6f,%.6f", latitude, longitude);	//	-u _printf_float
#else
	char res[20];
	strcpy( txBuff, "GPS -> http://maps.google.com/maps?q=");		// Set the number of receiver
	ftoa_fast(latitude, res, 6);
	strcat( txBuff + strlen(txBuff), res );
	strcat( txBuff + strlen(txBuff), "," );
	ftoa_fast(longitude, res, 6);
	strcat( txBuff + strlen(txBuff), res );
#endif


//	strcpy( txBuff, "http://maps.google.com/maps?q=");		// Set the number of receiver
//	strcat( txBuff + strlen(txBuff), _lat );
//	strcat( txBuff + strlen(txBuff), "," );
//	strcat( txBuff + strlen(txBuff), _lon );

#if defined(GSM_DEBUG_MODE_UART) && defined(HAL_UART_MODULE_ENABLED)
	HAL_UART_Transmit(uart2_buffer_gps.huart, (uint8_t *)txBuff, strlen(txBuff), UART_TRANSMIT_TIMEOUT);
	HAL_UART_Transmit(uart2_buffer_gps.huart, (uint8_t *)"\r\n", strlen("\r\n"), UART_TRANSMIT_TIMEOUT);
#endif

	return gsm_sendSMS(_data, txBuff, _phoneNumber);
}


/**
 * @brief  GSM send GPRS location message.
 * @param  *_data:	Pointer to GSM_SIM800_t data.
 * @param  *_phoneNumber:	Pointer to text phone number.
 * @retval send_flag: true/false
 */
uint8_t gsm_sendGPRS_location(GSM_SIM800_t *_data, char *_phoneNumber)
{
	// GSM Location Commands
	/*
	 * AT+SAPBR=3,1,"Contype","GPRS"
	 * OK
	 * AT+SAPBR=3,1,"APN","internet"
	 * OK
	 * AT+SAPBR=1,1
	 * OK
	 * AT+SAPBR=2,1
	 * +SAPBR: 1,1,"10.70.127.50"
	 * OK
	 * AT+CGATT?
	 * +CGATT: 1
	 * OK
	 * AT+CIPGSMLOC=1,1
	 * +CIPGSMLOC: 0,19.410540,53.065716,2017/11/04,12:25:38
	 * OK
	 */
	char txBuff[128] = {0};
	char rxBuff[128] = {0};
	char *lat = NULL;
	char *lon = NULL;
	uint8_t gprs_flag = false;

	if( gsm_sendCMD(_data, "AT+CGATT?\r\n", "+CGATT: 1", 1000) )	//	Check if the MS is connected to the GPRS network
		if( gsm_sendCMD(_data, "AT+SAPBR=3,1,\"Contype\",\"GPRS\"\r\n", "OK", 1000) )	//	Activate bearer profile
			if( gsm_sendCMD(_data, "AT+SAPBR=3,1,\"APN\",\"internet\"\r\n", "OK", 1000) )
				//if( gsm_sendCMD(_data, "AT+SAPBR=0,1\r\n", "", 1000) )	// ? OK
				if( gsm_sendCMD(_data, "AT+SAPBR=1,1\r\n", "", 1000) )	// ? OK
					if( gsm_sendCMD(_data, "AT+SAPBR=2,1\r\n", "OK", 2000) )
						if( gsm_sendCMD(_data, "AT+CGATT?\r\n", "+CGATT: 1", 1000) )	//	Check if the MS is connected to the GPRS network
							if( gsm_sendCMD(_data, "AT+CIPGSMLOC=1,1\r\n", "", 4000) )	//	Get location
							{
								ringBuffer_readBytes( _data->buffer, (uint8_t*)rxBuff, ringBuffer_checkAvailable(_data->buffer) );
								char *s = strstr(rxBuff, "+CIPGSMLOC");	// search for string "_gsmAtReply" in buff
								if(s != NULL)                     		// if successful then s now points at "_gsmAtReply"
								{
									gprs_flag = true;
									strtok((char*)rxBuff, ",");	//	+CIPGSMLOC: 0,
									lon = strtok(NULL, ",");
									lat = strtok(NULL, ",");
								}

								gsm_sendCMD(_data, "AT+SAPBR=0,1\r\n", "", 2000);	// Deactivate bearer profile
							}

	if( gprs_flag )
	{
		//	Google Maps: http://maps.google.com/maps?q=43.508083,-80.190653
		strcpy( txBuff, "GPRS -> http://maps.google.com/maps?q=");		// Set the number of receiver
		strcat( txBuff + strlen(txBuff), lat );
		strcat( txBuff + strlen(txBuff), "," );
		strcat( txBuff + strlen(txBuff), lon );

	#if defined(GSM_DEBUG_MODE_UART) && defined(HAL_UART_MODULE_ENABLED)
		HAL_UART_Transmit(uart2_buffer_gps.huart, (uint8_t *)txBuff, strlen(txBuff), UART_TRANSMIT_TIMEOUT);
		HAL_UART_Transmit(uart2_buffer_gps.huart, (uint8_t *)"\r\n", strlen("\r\n"), UART_TRANSMIT_TIMEOUT);
	#endif

		gprs_flag = gsm_sendSMS(_data, txBuff, _phoneNumber);
	}

	return gprs_flag;
}


