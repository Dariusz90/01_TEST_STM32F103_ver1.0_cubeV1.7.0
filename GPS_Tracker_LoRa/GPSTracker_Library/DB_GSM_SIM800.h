/*
 * DB_GSM_SIM800.h
 *
 *  Created on: 26 cze 2018
 *      Author: DBabraj
 */

#ifndef DB_GSM_SIM800_H_
#define DB_GSM_SIM800_H_

/* Includes (zalaczone biblioteki) -----------------------------------------------------------------------------------*/
#ifdef USE_HAL_DRIVER
#include "DB_stm_HAL_drivers.h"	// do obslugi funkcji z bibliotek STM32 HAL
#endif /* USE_HAL_DRIVER */

#include "DB_ring_buffer.h"


/* Preprocessor definition (definicje preprocesora) ------------------------------------------------------------------*/

/* Types definition (definicje typow) --------------------------------------------------------------------------------*/

typedef enum{
	GSM_STATE_SEND_CMGF,	//	Set the format of messages to Text mode
	GSM_STATE_SEND_CMGS,	// Send SMS to the corresponding phone
	GSM_STATE_SEND_MSG,		// Send SMS text
	GSM_STATE_SIZE
}gsmMsg_state_t;

typedef struct{
	RING_BUFFER_DATA *buffer;
	gsmMsg_state_t gsmMsg_state;
	uint8_t gsmInit_flag;
	uint8_t gsmSendError_cnt;
}GSM_SIM800_t;

/* Const declarations (deklaracje stalych) ---------------------------------------------------------------------------*/

/* Variable declarations (deklaracje zmiennych) ----------------------------------------------------------------------*/

/* Function declarations (deklaracje funkcji) ------------------------------------------------------------------------*/

uint8_t gsm_init(GSM_SIM800_t *_data, RING_BUFFER_DATA *_buffer, uint8_t _initRepeat_cnt, uint32_t _delay_ms);
uint8_t gsm_sendSMS(GSM_SIM800_t *_data, char *_msg, char *_phoneNumber);
uint8_t gsm_sendCMD(GSM_SIM800_t *_data, const char *_cmd, const char *_gsmAtReply, const uint32_t _replyTime);
uint8_t gsm_sendGPS_NMEA_location(GSM_SIM800_t *_data, char *_lat, char *_lon, char *_phoneNumber);
uint8_t gsm_sendGPRS_location(GSM_SIM800_t *_data, char *_phoneNumber);

/* Function definition (definicje funkcji) ---------------------------------------------------------------------------*/

#endif /* DB_GSM_SIM800_H_ */
