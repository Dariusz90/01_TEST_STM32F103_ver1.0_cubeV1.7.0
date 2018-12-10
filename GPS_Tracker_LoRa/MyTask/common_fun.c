/*
 * commonFunction.c
 *
 *  Created on: 20 cze 2018
 *      Author: DBabraj
 */

/* Includes (zalaczone biblioteki) -----------------------------------------------------------------------------------*/
#include "common_fun.h"
#include "sim_config.h"
#include <stdbool.h>
#include <string.h>

#include "DB_RFM98W_LoRa.h"

/* Preprocessor definition (definicje preprocesora) ------------------------------------------------------------------*/

/* Types definition (definicje typow) --------------------------------------------------------------------------------*/

/* Const declarations (deklaracje stalych) ---------------------------------------------------------------------------*/

/* Variable declarations (deklaracje zmiennych) ----------------------------------------------------------------------*/
/**************************************************
 * USART DATA
 **************************************************/
RING_BUFFER_DATA uart2_buffer_gps;
RING_BUFFER_DATA uart3_buffer_gsm;

/**************************************************
 * GPS DATA
 **************************************************/
GPS_NMEA_GLL_t gps_data;
uint8_t gpsRead_flag = false;

/**************************************************
 * GSM DATA
 **************************************************/
GSM_SIM800_t gsm_data;

/**************************************************
 * RFM98W DATA
 **************************************************/
RFM98_data_t RFM98_data;

float Frequency = 431.650e6;
float FrequencyOffset = 0.0; // TODO: Automatic Frequency Correction

enum
{
	RFM98_RegFifo,
	RFM98_RegOpMode,
	RFM98_RegBitrateMsb,
	RFM98_RegBitrateLsb,
	RFM98_RegFdevMsb,
	RFM98_RegFdevLsb,
	RFM98_RegFrfMsb,
	RFM98_RegFrfMid,
	RFM98_RegFrfLsb,
	RFM98_RegPaConfig,
	RFM98_RegPaRamp,
	RFM98_RegOcp,
	RFM98_RegLna,
	RFM98_SIZE
} rfmRegAddress;
uint8_t rfm98RegTable[RFM98_SIZE];

/**************************************************
 * SOFT TIMER DATA
 **************************************************/
volatile uint16_t soft_timer[ST_SIZE];

/* Function declarations (deklaracje funkcji) ------------------------------------------------------------------------*/
#ifdef SLEEP_MODE_ENABLE
static void
SystemPower_Config(void);
static void
SystemPower_Off(void);
#endif

/* Function definition (definicje funkcji) ---------------------------------------------------------------------------*/

/**
 * @brief  GPS_GSM Tracker init.
 * @param  none
 * @retval none
 */

void __attribute__ ((noinline)) FATAL_ERROR(char * msg)
{
	volatile char * ERROR_MSG = msg;
	while (1) {}
}

void DBG_PRINT(const char * arg)
{
	HAL_UART_Transmit(uart2_buffer_gps.huart, (void*) "DBG: ", 5, 100);
	HAL_UART_Transmit(uart2_buffer_gps.huart, (void*) arg, strlen(arg), 100);
	HAL_UART_Transmit(uart2_buffer_gps.huart, (void*) "\x18\r\n", 3, 100);
}

void DBG_PRINT_NO(unsigned int n)
{
	char Number[6] = "00000\0";
	char *Current = Number + sizeof(Number) - 1;
	int len = 0;

	do
	{
		len++;
		Current--;
		*(Current) = '0' + n % 10;
		n /= 10;
	} while (n && len < sizeof(Number));
	DBG_PRINT(Number);
}

void DBG_PRINT_TN(const char* arg,uint16_t num)
{
	char Number[6] = "00000\0";
	char *Current = Number + sizeof(Number) - 1;
	int len = 0;

	do
	{
		len++;
		Current--;
		*(Current) = '0' + num % 10;
		num /= 10;
	} while (num && len < sizeof(Number));

	HAL_UART_Transmit(uart2_buffer_gps.huart, (void*) "DBG: ", 5, 100);
	HAL_UART_Transmit(uart2_buffer_gps.huart, (void*) arg, strlen(arg), 100);
	HAL_UART_Transmit(uart2_buffer_gps.huart, (void*) Number, strlen(Number), 100);
	HAL_UART_Transmit(uart2_buffer_gps.huart, (void*) "\x18\r\n", 3, 100);
}


void LoRa_Panic(void)
{
	RFM98_LoRa_data_t RFM98_LoRa_data;

	RFM98_LoRa_data.Modulation = LORA; //Modulation mode is LoRa
	RFM98_LoRa_data.COB = RFM98; //Module is RFM98
	RFM98_LoRa_data.Frequency = 434000; // Target frequency is 434MHz
	RFM98_LoRa_data.OutputPower = 17; //Output power is 17dBm
	RFM98_LoRa_data.PreambleLength = 16; //Preamble length is 16 bytes
	RFM98_LoRa_data.FixedPktLength = false; //Message length is variable
	RFM98_LoRa_data.PayloadLength = 21; //Message length is 21 bytes
	RFM98_LoRa_data.CrcDisable = true; //Disable CRC for true
	RFM98_LoRa_data.SFSel = SF9; //Spread spectrum factor is 9
	RFM98_LoRa_data.BWSel = BW125K; //Spread spectrum transmitting bandwidth is 125 KHz.
	RFM98_LoRa_data.CRSel = CR4_5; //Coding rate is 4/5

	RFM98W_LoRa_init(&RFM98_LoRa_data, &hspi2, QB_SPI1_CS_GPIO_Port,
	QB_SPI1_CS_Pin,
	RST_RFM_GPIO_Port, RST_RFM_Pin);

	uint8_t i = 0;
	for (i = 0; i < RFM98_SIZE; ++i)
	{
		rfm98RegTable[i] = RFM98W_LoRa_ReadRegister(i);
	}

	char messageTx[] = "PANIC!";
	do
	{
		RFM98W_LoRa_bSendMessage(&RFM98_LoRa_data, (byte*) messageTx,
				strlen(messageTx)); //Transmit a packet of messageTx every one second
		HAL_Delay(50);
	} while (true);

#if 0
	RFM98W_init(&RFM98_data, &hspi2, QB_SPI1_CS_GPIO_Port, QB_SPI1_CS_Pin, RST_RFM_GPIO_Port, RST_RFM_Pin);

	uint8_t i = 0;
	for (i = 0; i < RFM98_SIZE; ++i)
	{
		rfm98RegTable[i] = RFM98W_ReadRegister(i);
	}

	RFM98W_setLoRaMode(&RFM98_data, Frequency + FrequencyOffset);
	RFM98W_startReceiving(&RFM98_data);
	do
	{
		RFM98W_sendData(&RFM98_data, "ACTION ", 7);
	}while (true);
#endif
}

typedef union
{
	struct
	{
		int16_t RTC_Interval;
		int16_t RTC_TimeLeft;
	} Split;
	uint32_t Merged;
} RTC_Backup;

RTC_Backup Backup;

void RTC_SetInterval(int Hours)
{

	Backup.Split.RTC_Interval = Hours;
	Backup.Split.RTC_TimeLeft = Hours;

	/*
	 HAL_PWR_EnableBkUpAccess();
	 HAL_RTCEx_BKUPWrite(&hrtc, 0, Backup.Merged);
	 HAL_PWR_DisableBkUpAccess();
	 */
}

bool RTC_WakeUp(void)
{
	bool Result = false;
	/*
	 RTC_Backup Backup;
	 HAL_PWR_EnableBkUpAccess();
	 Backup.Merged = HAL_RTCEx_BKUPRead(&hrtc, 0);
	 */
	Backup.Split.RTC_TimeLeft--;
	if (Backup.Split.RTC_TimeLeft <= 0)
	{
		if (Backup.Split.RTC_Interval == 0)
			Backup.Split.RTC_Interval = DEFAULT_SLEEP_TIME;
		Backup.Split.RTC_TimeLeft = Backup.Split.RTC_Interval;
		Result = true;
	}
	/*
	 HAL_RTCEx_BKUPWrite(&hrtc, 0, Backup.Merged);
	 HAL_PWR_DisableBkUpAccess();
	 */

	DBG_PRINT("BUDZE SIE");
	return Result;
}

bool Increment_bcd(uint8_t * val, uint8_t bcd_limit)
{
	(*val)++;
	if ((*val & 0x0f) > 0x09)
	{
		*val += 0x10 - 0x0a;
	}

	if (*val >= bcd_limit)
	{
		*val -= bcd_limit;
		return true;
	}
	return false;
}

void RTC_GoSleep(void)
{
	DBG_PRINT("IDE SPAC");

	RTC_TimeTypeDef Time;
	HAL_RTC_GetTime(&hrtc, &Time, RTC_FORMAT_BCD);

	RTC_AlarmTypeDef Alarm;
	Alarm.AlarmTime.Hours = Time.Hours;
	Alarm.AlarmTime.Seconds = 0;
	Alarm.Alarm = 0;

#ifdef SLEEP_IN_MINUTES
	// w trybie minutowym nie ma szans na zachowanie równych interwałów wywołania
	// za to robiony jest równy odstęp.
	// ten tryb jest używany tylko podczas testów, dokładność interwału jest mało istotna
	Alarm.AlarmTime.Minutes = Time.Minutes;
	if (Increment_bcd(&(Alarm.AlarmTime.Minutes), 0x60))
#else
		Alarm.AlarmTime.Minutes = BUILD_MIN_BCD;
#endif
		Increment_bcd(&(Alarm.AlarmTime.Hours), 0x24);

	HAL_RTC_SetAlarm_IT(&hrtc, &Alarm, RTC_FORMAT_BCD);
#ifdef SLEEP_MODE_ENABLE
	HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFE);
	HAL_GPIO_WritePin(QB_DEBUG_LED_GREEN_GPIO_Port, QB_DEBUG_LED_GREEN_Pin,GPIO_PIN_SET);
	HAL_Delay(50);
	HAL_GPIO_WritePin(QB_DEBUG_LED_GREEN_GPIO_Port, QB_DEBUG_LED_GREEN_Pin,GPIO_PIN_RESET);
	HAL_RTC_AlarmIRQHandler(&hrtc);
#endif
}

void GPS_GSM_Tracker_init(void)
{
	ringBuffer_init_DMA(&uart2_buffer_gps, &huart2);
	ringBuffer_init_DMA(&uart3_buffer_gsm, &huart3);

	int var = 0;
	for (var = 0; var < 12; ++var)
	{
		HAL_GPIO_TogglePin(QB_DEBUG_LED_GREEN_GPIO_Port,
		QB_DEBUG_LED_GREEN_Pin);
		HAL_Delay(50);
	}

	// GPS Init
	GPS_NMEA_init_simple(&gps_data, &uart2_buffer_gps);

#ifdef SLEEP_MODE_ENABLE
#ifndef WORK_BEFORE_SLEEP
	RTC_SetInterval(DEFAULT_SLEEP_TIME);
#endif
	SystemPower_Config();
#endif /* SLEEP_MODE_ENABLE */

}

/**
 * @brief  GPS_GSM Tracker.
 * @param  none
 * @retval none
 */

typedef enum
{
	TS_Standby, TS_Initializing, TS_Ready, TS_SendingMessage, TS_Error,
} GPRS_Task_Status;

typedef enum
{
	ATR_Unknown, ATR_OK, ATR_Error, ATR_Empty
} AT_Response;

typedef enum
{
	TJ_Standby = 0,
	TJ_PowerOnInit,
	TJ_ModemReset,
	TJ_EchoOff,
	TJ_InitSMS,
	TJ_NextTask,

	TJ_EnableGPRS,
	TJ_SetConnType,
#ifdef SIM_GPRS_APN
	TJ_SetAPN,
#endif
#ifdef SIM_GPRS_USER
	TJ_SetGPRSUser,
#endif
#ifdef SIM_GPRS_PASSWORD
	TJ_SetGPRSPassword,
#endif
	TJ_SetGPRSMode1,
	TJ_SetGPRSMode2,
	TJ_GetGPRSLocation,
	TJ_EndGPRS,

	//
	TJ_SendSMSPhoneNumber,
	TJ_SendSMSMessage,
	TJ_EndSendSMS,

	TJ_GetSMSCount,
	TJ_CountSMSReady,
	TJ_ReadSMS,
	TJ_ReadSMSReady,
	TJ_DeleteSMS,
	TJ_EndReadSMS,

	TJ_Ready,
	TJ_PowerOff,
	TJ_FatalError
} GPRS_Task_Job;

typedef enum
{
	JS_Init,
	JS_SendCommand,
	JS_WaitForTimeout,
	JS_CheckResponse,
	JS_Success,
	JS_Error

} GPRS_Job_State;

typedef enum
{
	SMS_SlotEmpty, SMS_Junk, SMS_Command, SMS_Error
} SMS_Type;

void __attribute__ ((noinline))
GPRS_Command(RING_BUFFER_DATA* GSM, GPRS_Job_State * State, int * Timeout,
		int DefaultTimeout, char * Command, char * Response,
		const char * Expect)
{
	switch (*State)
	{
	case JS_Init:
		*Timeout = DefaultTimeout;
		(*State)++;
		/* no break */

	case JS_SendCommand:
		ringBuffer_flush(GSM);
		HAL_UART_Transmit(GSM->huart, (void*) Command, strlen(Command), 100);
		(*State)++;
		break;

	case JS_CheckResponse:
		memset(Response, 0, 256);
		int Available = ringBuffer_checkAvailable(GSM);
#if UART_RX_BUFFER_SIZE > 256
		while (Available > 256)
		{
			int MaxRead = Available - 256;
			if (MaxRead > 256)
				MaxRead = 256;
			ringBuffer_readBytes(GSM, (void*) Response, MaxRead);
			Available -= MaxRead;
		}
#endif
		ringBuffer_readBytes(GSM, (void*) Response, Available);

		int RespLen = strlen(Response);
		int OkayLen = strlen(Expect);

		if ((RespLen >= OkayLen
				&& memcmp(Expect, &Response[RespLen - OkayLen], OkayLen) == 0)
				|| (RespLen == 0))
			*State = JS_Success;
		else
			*State = JS_Error;
		break;

	default:
		break;

	}

}

void itoa(uint32_t num, char ** buf, uint8_t* len)
{
	static char str[11];
	int idx = 10;
	int curr_len = 0;
	str[10] = 0;

	do
	{
		str[idx--] = '0' + (num % 10);
		num /= 10;
		curr_len++;
	} while (num);
	*buf = &str[idx + 1];

	if(len != NULL)
		*len = curr_len;
}


uint8_t __attribute__ ((noinline)) GetPendingSMS(char *RxMsg)
{
	if (RxMsg == NULL)
	{
		FATAL_ERROR("GetPendingSMS");
	}
	RxMsg = strstr(RxMsg, "+CPMS:");
	char * CMD_Start = NULL;
	uint8_t SmsCnt = 0;

	if (RxMsg == NULL)
		return SmsCnt;

	CMD_Start = strstr(RxMsg, "+CPMS: ");
	CMD_Start += 7;

	do
	{
		if (*CMD_Start < '0' || *CMD_Start > '9')
			break;

		SmsCnt = SmsCnt * 10 + *CMD_Start - '0';
		CMD_Start++;

		if (*CMD_Start == ',')	//	'\0'
		{
			// wartość kompletna i poprawna
			DBG_PRINT_TN("SMS OCZEKUJACYH: ",SmsCnt);
			break;
		}
	} while (1);

	return SmsCnt;
}

uint8_t __attribute__ ((noinline)) GetMaxSMSCount(char *RxMsg)
{
	if (RxMsg == NULL)
	{
		FATAL_ERROR("GetMaxSMSCount");
	}
	RxMsg = strstr(RxMsg, "+CPMS:");
	char * CMD_Start;
	uint8_t SmsCnt = 0;

	if (RxMsg == NULL)
		return SmsCnt;

	CMD_Start = strstr(RxMsg, ",");
	CMD_Start += 1;

	do
	{
		if (*CMD_Start < '0' || *CMD_Start > '9')
			break;

		SmsCnt = SmsCnt * 10 + *CMD_Start - '0';
		CMD_Start++;

		if (*CMD_Start == ',')	//	'\0'
		{
			// wartość kompletna i poprawna
			DBG_PRINT_TN("POJEMNOSC SKRZYNKI: ",SmsCnt);
			break;
		}
	} while (1);

	return SmsCnt;
}

#define GPRS_FLAG_GET_LOCATION (uint8_t) (1 << 0)
#define GPRS_FLAG_SEND_SMS     (uint8_t) (1 << 1)
#define GPRS_FLAG_READ_SMS     (uint8_t) (1 << 2)

#define AT_RESPONSE_OK "\r\nOK\r\n"
#define AT_RESPONSE_OK_SMS "\r\n> "

static volatile unsigned short SmsNo;
static unsigned short SmsFound;

GPRS_Task_Job  __attribute__ ((noinline)) GPRS_Task(bool Run, RING_BUFFER_DATA* GSM, char * RxMessage,
		char * TxMessage, uint8_t * Flags)
{

	static GPRS_Task_Job Job = 0;
	static GPRS_Job_State State = 0;
	static int Timeout;
	static int Retries;
	static int GlobalRetries;
	static char TxMessageBuffer[192];

	static unsigned short SMSToFind;
	static unsigned short SMSMaxCount;
	// kompilator uwaza, ze run jest zawsze nie-0
	if (!Run)
	{
		GlobalRetries = 0;
		if (Job != TJ_Standby)
		{
			HAL_GPIO_WritePin(QB_GSM_PWR_ON_GPIO_Port, QB_GSM_PWR_ON_Pin,
					GPIO_PIN_RESET);
			Job = TJ_Standby;
		}
		return Job;
	}

	switch (Job)
	{
	case TJ_Standby:
		Job = TJ_PowerOnInit;
		/* no break */
	case TJ_PowerOnInit:
		switch (State)
		{
		case JS_Init:
			HAL_GPIO_WritePin(QB_GSM_PWR_ON_GPIO_Port, QB_GSM_PWR_ON_Pin,
					GPIO_PIN_SET);
			Timeout = 80;
			Retries = 0;
			State = JS_WaitForTimeout;
			break;
		case JS_CheckResponse:
			State = JS_Success;
			break;
		default:
			break;
		}
		break;

	case TJ_ModemReset:
		GPRS_Command(GSM, &State, &Timeout, SIM_TIMEOUT_LONG, "ATZ\r\n",
				RxMessage, "");
		break;

	case TJ_EchoOff:
		GPRS_Command(GSM, &State, &Timeout, SIM_TIMEOUT_SHORT, "ATE0\r\n",
				RxMessage, AT_RESPONSE_OK);
		break;

	case TJ_InitSMS:
		GPRS_Command(GSM, &State, &Timeout, SIM_TIMEOUT_SHORT, "AT+CMGF=1\r\n",
				RxMessage, AT_RESPONSE_OK);
		break;

	case TJ_Ready:
	case TJ_NextTask:
		if ((*Flags) & GPRS_FLAG_GET_LOCATION)
		{
			Job = TJ_EnableGPRS;
		}
		else if ((*Flags) & GPRS_FLAG_SEND_SMS)
		{
			Job = TJ_SendSMSPhoneNumber;
		}
		else if ((*Flags) & GPRS_FLAG_READ_SMS)
		{
			SmsNo = 1;//1
			Job = TJ_GetSMSCount;
		}
		else
		{
			Job = TJ_Ready;
		}
		break;

	case TJ_EnableGPRS:
		GPRS_Command(GSM, &State, &Timeout, SIM_TIMEOUT_LONG, "AT+CGATT=1\r\n",
				RxMessage, AT_RESPONSE_OK);
		break;
	case TJ_SetConnType:
		GPRS_Command(GSM, &State, &Timeout, SIM_TIMEOUT_SHORT,
				"AT+SAPBR=3,1,\"Contype\",\"GPRS\"\r\n", RxMessage,
				AT_RESPONSE_OK);
		break;
#ifdef SIM_GPRS_APN
	case TJ_SetAPN:
		GPRS_Command(GSM, &State, &Timeout, SIM_TIMEOUT_SHORT,
				"AT+SAPBR=3,1,\"APN\",\"" SIM_GPRS_APN "\"\r\n", RxMessage,
				AT_RESPONSE_OK);
		break;
#endif
#ifdef SIM_GPRS_USER
	case TJ_SetGPRSUser:
		GPRS_Command(GSM, &State, &Timeout, SIM_TIMEOUT_SHORT,
				"AT+SAPBR=3,1,\"USER\",\"" SIM_GPRS_USER "\"\r\n", RxMessage,
				AT_RESPONSE_OK);
		break;
#endif
#ifdef SIM_GPRS_PASSWORD
	case TJ_SetGPRSPassword:
		GPRS_Command(GSM, &State, &Timeout, SIM_TIMEOUT_SHORT,
				"AT+SAPBR=3,1,\"PWD\",\"" SIM_GPRS_PASSWORD "\"\r\n", RxMessage,
				AT_RESPONSE_OK);
		break;
#endif
	case TJ_SetGPRSMode1:
		GPRS_Command(GSM, &State, &Timeout, SIM_TIMEOUT_LONG,
				"AT+SAPBR=1,1\r\n", RxMessage, AT_RESPONSE_OK);
		break;

	case TJ_SetGPRSMode2:
		GPRS_Command(GSM, &State, &Timeout, SIM_TIMEOUT_LONG,
				"AT+SAPBR=2,1\r\n", RxMessage, AT_RESPONSE_OK);
		break;

	case TJ_GetGPRSLocation:
		GPRS_Command(GSM, &State, &Timeout, SIM_TIMEOUT_LONG,
				"AT+CIPGSMLOC=1,1\r\n", RxMessage, AT_RESPONSE_OK);
		break;

	case TJ_EndGPRS:
		Job = TJ_NextTask;
		*Flags = (*Flags) & ~ GPRS_FLAG_GET_LOCATION;
		break;

	case TJ_FatalError:
		HAL_GPIO_WritePin(QB_GSM_PWR_ON_GPIO_Port, QB_GSM_PWR_ON_Pin,
				GPIO_PIN_RESET);
		break;

	case TJ_SendSMSPhoneNumber:
		GPRS_Command(GSM, &State, &Timeout, SIM_TIMEOUT_SHORT,
				"AT+CMGS=\"" SIM_DESTINATION_PHONE_NUMBER "\"\r\n", RxMessage,
				AT_RESPONSE_OK_SMS);
		break;

	case TJ_SendSMSMessage:
		if (State == JS_Init)
		{
			if (TxMessage == NULL)
			{
				FATAL_ERROR("GPRS_Task/TJ_SendSMSMessage/JS_Init");
			}
			strcpy(TxMessageBuffer, TxMessage);
			strcat(TxMessageBuffer, "\x1a");
		}
		GPRS_Command(GSM, &State, &Timeout, SIM_TIMEOUT_LONG, TxMessageBuffer,
				RxMessage, AT_RESPONSE_OK);
		break;

	case TJ_EndSendSMS:
		*Flags = (*Flags) & ~ GPRS_FLAG_SEND_SMS;
		Job = TJ_NextTask;
		break;

	//sprawdzamy ile SMS mamy do odczytu
	case TJ_GetSMSCount:
		if (State == JS_Init)
		{
			SmsFound = 0;
			SMSToFind = 0;
		}
		GPRS_Command(GSM, &State, &Timeout, SIM_TIMEOUT_LONG,
				"AT+CPMS=\"MT\",\"ME\",\"SM\"\r\n", RxMessage, "");
		break;

	case TJ_CountSMSReady:

		SMSToFind = GetPendingSMS(RxMessage);
		SMSMaxCount = GetMaxSMSCount(RxMessage);

		Job = TJ_ReadSMS;

		if (SMSToFind == 0)
		{
			SmsNo = 1; //1
			*Flags = (*Flags) & ~ GPRS_FLAG_READ_SMS;
			Job = TJ_EndReadSMS;
		}

		break;

	case TJ_ReadSMS:
		if (State == JS_Init)
		{
			uint8_t Offset = 0;
			strcpy(TxMessageBuffer, "AT+CMGR=");

			char * SmsNoStr;
			itoa((uint32_t) SmsNo, &SmsNoStr, &Offset);
			strcpy(TxMessageBuffer + 8, SmsNoStr);
			strcpy(TxMessageBuffer + 8 + Offset, "\r\n");

			DBG_PRINT_TN("CZYTAM SLOT SMS NUMER: ",SmsNo);
		}
		GPRS_Command(GSM, &State, &Timeout, SIM_TIMEOUT_SHORT, TxMessageBuffer,
				RxMessage, "");
		break;

	case TJ_ReadSMSReady:
		Job++;
		break;

	case TJ_DeleteSMS:
		if (State == JS_Init)
		{
			strcpy(TxMessageBuffer, "AT+CMGDA=\"DEL READ\"\r\n");
		}
		GPRS_Command(GSM, &State, &Timeout, SIM_TIMEOUT_SHORT, TxMessageBuffer,
				RxMessage, AT_RESPONSE_OK);


		break;

	case TJ_EndReadSMS:

		//SmsNo++;

		if ((SMSToFind != SmsFound) && (SmsNo <= SMSMaxCount))
		{
			Job = TJ_ReadSMS;
		}
		else
		{
			SmsNo = 1;
			*Flags = (*Flags) & ~ GPRS_FLAG_READ_SMS;
			DBG_PRINT("WSZYSTKIE SMS ODCZYTANE");
			Job = TJ_NextTask;
		}
		break;

	case TJ_PowerOff:
		switch (State)
		{
		case JS_Init:
			HAL_GPIO_WritePin(QB_GSM_PWR_ON_GPIO_Port, QB_GSM_PWR_ON_Pin,
					GPIO_PIN_RESET);
			Timeout = 40;
			State = JS_WaitForTimeout;
			break;
		case JS_CheckResponse:
			State = JS_Init;
			Job = TJ_PowerOnInit;
			break;
		default:
			break;
		}
	};

	switch (State)
	{
	case JS_Init:
		Retries = SIM_COMMAND_RETRIES;
		break;

	case JS_Success:
		State = JS_Init;
		Retries = SIM_COMMAND_RETRIES;
		Job++;
		break;

	case JS_Error:
		if (Retries)
		{
			Retries--;
			State = JS_Init;
		}
		else
		{
			GlobalRetries++;
			if (GlobalRetries >= SIM_GLOBAL_RETRIES)
				Job = TJ_FatalError;
			else
				Job = TJ_PowerOff;
			State = JS_Init;
			Retries = SIM_COMMAND_RETRIES;
		}
		break;

	case JS_WaitForTimeout:
		if (Timeout)
			Timeout--;
		else
			State = JS_CheckResponse;
		break;

	default:
		Job = TJ_PowerOff;
		State = JS_Init;

	}
	return Job;
}



SMS_Type ParseSMS(char *SMS)
{
	char *CMD_Start;
	int Time = 0;

	//sprawdz czy slot SMS pusty
	if ((strncmp(SMS, "\r\nOK\r\n", 6)) == 0)
	{
		return SMS_SlotEmpty;
	}

	//sprawdz czy potencjalna komenda
	if ((SMS = strstr(SMS, "\r\nACTION ")) != NULL)
	{
		SMS += 9;

		if (strncmp("STOP", SMS, 4) == 0)
		{
			DBG_PRINT("STOP command. Goodbye cruel world!");
			HAL_GPIO_WritePin(QB_GPS_PWR_ON_GPIO_Port, QB_GPS_PWR_ON_Pin,
					GPIO_PIN_SET);
			HAL_GPIO_WritePin(QB_DEBUG_LED_GREEN_GPIO_Port,
			QB_DEBUG_LED_GREEN_Pin, GPIO_PIN_SET);
			while (1)
			{
				HAL_UART_Transmit(uart3_buffer_gsm.huart, (void*) "AT+CMGDA=\"DEL READ\"\r\n", 25, 100);
				HAL_Delay(5000);
			}
			// polcecenie STOP. Umrzyj.
		}
		else if (strncmp("LORA", SMS, 4) == 0)
		{
			DBG_PRINT("LORA command. PANIC!");
			HAL_UART_Transmit(uart3_buffer_gsm.huart, (void*) "AT+CMGDA=\"DEL READ\"\r\n", 25, 100);
			HAL_Delay(5000);
			HAL_GPIO_WritePin(QB_GPS_PWR_ON_GPIO_Port, QB_GPS_PWR_ON_Pin,
					GPIO_PIN_SET);
			LoRa_Panic();
			// polecenie LORA. Wysyłaj panikę
		}

		else
		{
			CMD_Start = strstr(SMS, "START ");
			if (CMD_Start != NULL)
			{
				if (*CMD_Start == 'S')
					CMD_Start += 6;
				Time = 0;

				do
				{
					if (*CMD_Start < '0' || *CMD_Start > '9')
						return SMS_Error;

					Time = Time * 10 + *CMD_Start - '0';
					CMD_Start++;

					if (*CMD_Start == 'H')	//	'\0'
					{
						// wartość kompletna i poprawna
						DBG_PRINT_TN("NOWY CZAS BUDZENIA CO :",Time);
						RTC_SetInterval(Time);
						break;
					}

					if (*CMD_Start == '\0')	//	'\0'
						return SMS_Error;

				} while (1);

				return SMS_Command;
			}
		}
	}
	return SMS_Junk;
}

bool ParseSMSCommand(char * SMS)
{
	//"\r\nOK\r\n" - pusty SMS

	SMS = strstr(SMS, "\r\nACTION ");
	char * CMD_Start;
	int Time = 0;

	bool state = false;

	if (SMS == NULL)
		return state;

	SMS += 9; // strlen("\r\nACTION ");

	if (strncmp("STOP", SMS, 4) == 0)
	{
		DBG_PRINT("STOP command. Goodbye cruel world!");
		HAL_GPIO_WritePin(QB_GPS_PWR_ON_GPIO_Port, QB_GPS_PWR_ON_Pin,
				GPIO_PIN_SET);
		HAL_GPIO_WritePin(QB_DEBUG_LED_GREEN_GPIO_Port,
		QB_DEBUG_LED_GREEN_Pin, GPIO_PIN_SET);
		while (1);
		// polcecenie STOP. Umrzyj.
	}
	else if (strncmp("LORA", SMS, 4) == 0)
	{
		DBG_PRINT("LORA command. PANIC!");



		HAL_GPIO_WritePin(QB_GPS_PWR_ON_GPIO_Port, QB_GPS_PWR_ON_Pin,
				GPIO_PIN_SET);
		LoRa_Panic();
		// polecenie LORA. Wysyłaj panikę
	}
	else
	{
		CMD_Start = strstr(SMS, "START ");
		if (CMD_Start != NULL)
		{
			if (*CMD_Start == 'S')
				CMD_Start += 6;
			Time = 0;
			do
			{
				if (*CMD_Start < '0' || *CMD_Start > '9')
					break;

				Time = Time * 10 + *CMD_Start - '0';
				CMD_Start++;

				if (*CMD_Start == 'H')	//	'\0'
				{
					// wartość kompletna i poprawna
					DBG_PRINT("Repeat time: ");
					DBG_PRINT_NO(Time);
					RTC_SetInterval(Time);
					state = true;
					break;
				}
			} while (1);
			// polecenie budzenia się co n GODZIN
//            state = true;
		}
	}

	return state;
}


bool GPS_GSM_TrackerTask(void)
{
	static volatile bool GPRS_Run;
	static char GPRS_RxMessage[256];
	static char GPRS_Location[128];
	static uint8_t GPRS_To_Do = 0;
	static int GPRS_Mode;
	static int GPRS_Ticks;
	static uint8_t timeSet_flag = false;

	if (!ringBuffer_checkBounds(&uart3_buffer_gsm))
		FATAL_ERROR("BUFFER OVERFLOW");

	//MAIN STATE MACHINE
	if (!GPRS_Run)
	{
		GPRS_Ticks = 0;
		GPRS_Mode = 0;
		HAL_GPIO_WritePin(QB_GPS_PWR_ON_GPIO_Port, QB_GPS_PWR_ON_Pin,
						  GPIO_PIN_SET);
	}
	GPRS_Run = true;

	if (SOFT_TIMER_EXECUTE(soft_timer, ST_MAIN_LOOP, 100))
	{
		HAL_GPIO_TogglePin(QB_DEBUG_LED_GREEN_GPIO_Port,
						   QB_DEBUG_LED_GREEN_Pin);

		if (GPRS_Run)
		{
			GPRS_Ticks++;
			switch (GPRS_Mode)
			{
			case 0: // daj mu odpocz��, niech pozbiera SMS z sieci
				// ten case musi zosta�, bo nie sprawdzamy stanu zalogowania do sieci
				// mo�na skr�ci� czas oczekiwania w sim_config.h -- SIM_SMS_WAIT_TIME
				if (GPRS_Task(GPRS_Run, &uart3_buffer_gsm, GPRS_RxMessage, 0,
							  &GPRS_To_Do) == TJ_FatalError)
					GPRS_Run = false;

				if (GPRS_Ticks >= SIM_STARTUP_WAIT_TIME)
				{
					GPRS_Mode++;
					GPRS_To_Do = GPRS_FLAG_GET_LOCATION;
				}
				break;

			case 1: //pobierz lokalizacje AGPS
				switch (GPRS_Task(GPRS_Run, &uart3_buffer_gsm, GPRS_RxMessage,
								  0, &GPRS_To_Do))
				{
				case TJ_FatalError:
					GPRS_Run = false;
					break;
				case TJ_Ready:
					DBG_PRINT("USTALAM POZYCJE AGPS");
					strncpy(GPRS_Location, GPRS_RxMessage,
							sizeof(GPRS_Location) - 1);
					GPRS_To_Do = GPRS_FLAG_SEND_SMS;
					GPRS_Mode++;
					break;
				default:
					break;
				}
				break;

			case 2:
				switch (GPRS_Task(GPRS_Run, &uart3_buffer_gsm, GPRS_RxMessage,
								  GPRS_Location, &GPRS_To_Do))
				{
				case TJ_FatalError:
					GPRS_Run = false;
					GPRS_Mode = 0;
					break;
				case TJ_Ready:
					DBG_PRINT("WYSYLAM SMS Z LOKALIZACJA AGPS");
					GPRS_Mode++;
					GPRS_Ticks = 0; //reset czekamy na sms
					break;
				default:
					break;
				}
				break;

			case 3: // czekamy na przycodzące SMS
				if(GPRS_Ticks < 2)
					DBG_PRINT("CZEKAM NA SMS.....");
				if (GPRS_Ticks >= SIM_SMS_WAIT_TIME)
				{
					GPRS_Mode++;
					GPRS_To_Do = GPRS_FLAG_READ_SMS;
				}
				break;

			case 4: // odczytaj odebrane SMS
				switch (GPRS_Task(GPRS_Run, &uart3_buffer_gsm, GPRS_RxMessage,
								  0, &GPRS_To_Do))
				{
				case TJ_ReadSMSReady:

					switch (ParseSMS(GPRS_RxMessage))
					{
					//empty slot check next
					case SMS_SlotEmpty:
						SmsNo++;
						break;

					//non key message delete
					case SMS_Junk:
						SmsNo++;
						SmsFound++;
						break;

					//command exec
					case SMS_Command:
						timeSet_flag = true;
						SmsNo++;
						SmsFound++;
						break;

					case SMS_Error:
						SmsNo++;
						SmsFound++;
						break;

					default:
						SmsNo++;
						break;
					}
					break;

				case TJ_FatalError:
					GPRS_Run = false;
					break;
				case TJ_Ready:
					if (timeSet_flag == true)
					{
						timeSet_flag = false;
						GPRS_To_Do = GPRS_FLAG_SEND_SMS;
						GPRS_Mode++;
					}
					else
					{
						GPRS_Mode = 0;
						GPRS_Run = false;
					}
					break;
				default:
					break;
				}
				break;



			case 5: // wyslij SMS z potwierdzeniem ustawienia czasu wybudzenia
			{
				char Text[] =  {"Time is set to :     /r/n"};
				char* ToCpy;
				itoa((uint32_t)Backup.Split.RTC_Interval,&ToCpy,NULL);
				strcpy(Text + 15, ToCpy);

				switch (GPRS_Task(GPRS_Run, &uart3_buffer_gsm, GPRS_RxMessage,
								  Text, &GPRS_To_Do))
				{
				case TJ_FatalError:
					GPRS_Run = false;
					break;
				case TJ_Ready:
					DBG_PRINT_TN("NOWY CZAS WYBUDZANIA USTAWIONY : ",Backup.Split.RTC_Interval);
					GPRS_Run = false;
					GPRS_Mode =0;
					break;
				default:
					break;
				}
				break;
			}
			default:
				GPRS_Mode = 0;
			}
		}
	}

	if (!GPRS_Run) // nie else, gdyz ta flaga moze zmienic wartosc w trakcie wykonywania
	{
		GPRS_Task(false, 0, 0, 0, 0); // wy��cz radio
		HAL_GPIO_WritePin(QB_GPS_PWR_ON_GPIO_Port, QB_GPS_PWR_ON_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(QB_DEBUG_LED_GREEN_GPIO_Port, QB_DEBUG_LED_GREEN_Pin, GPIO_PIN_RESET);
	}

	return GPRS_Run;
}

#ifdef SLEEP_MODE_ENABLE
static void SystemPower_Config(void)
{
	/* Enable Power Control clock */
	__HAL_RCC_PWR_CLK_ENABLE()
	;
	/* Enable Ultra low power mode */
}
#endif /* SLEEP_MODE_ENABLE */

