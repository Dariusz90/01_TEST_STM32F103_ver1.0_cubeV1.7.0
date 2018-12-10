/*
 * DB_RFM98W.h
 *
 *  Created on: 2 sie 2018
 *      Author: DBabraj
 */

#ifndef DB_RFM98W_LORA_H_
#define DB_RFM98W_LORA_H_

/* Includes (zalaczone biblioteki) -----------------------------------------------------------------------------------*/
#ifdef USE_HAL_DRIVER
#include "DB_stm_HAL_drivers.h"	// do obslugi funkcji z bibliotek STM32 HAL
#endif /* USE_HAL_DRIVER */

/********************** NOTES **********************************************
...
*******************************************************************************/
/* Preprocessor definition (definicje preprocesora) ------------------------------------------------------------------*/


#ifndef	byte
	typedef uint8_t byte;
#endif

#ifndef word
	typedef uint16_t  word;
#endif

#ifndef lword
	typedef uint32_t lword;
#endif


/* Types definition (definicje typow) --------------------------------------------------------------------------------*/

typedef union
{
	struct
	{
		byte FreqL: 8;
		byte FreqM: 8;
		byte FreqH: 8;
		byte FreqX: 8;
	}freq;
	unsigned long Freq;
}RFM_FreqStruct_t;

typedef enum RFM_modulationType_ {OOK, FSK, GFSK, LORA}RFM_modulationType;

typedef enum RFM_moduleType_ {RFM92, RFM93, RFM95, RFM96, RFM97, RFM98}RFM_moduleType;

typedef enum RFM_sfType_ {SF6, SF7, SF8, SF9, SF10, SF11, SF12}RFM_sfType;

typedef enum RFM_bwType_ {BW62K, BW125K, BW250K, BW500K}RFM_bwType;

typedef enum RFM_crType_ {CR4_5, CR4_6, CR4_7, CR4_8}RFM_crType;

typedef union{
	struct{
		uint8_t DIO0:1;
		uint8_t DIO1:1;
		uint8_t DIO2:1;
		uint8_t DIO3:1;
		uint8_t DIO4:1;
		uint8_t DIO5:1;
		uint8_t reserved:2;
	}bit;
	uint8_t Reg;
}RFM_interruptFlag_t;

typedef struct{
//	GPIO_TypeDef* SPI_CS_GPIOx;
//	uint16_t SPI_CS_GPIO_Pin;
//	SPI_HandleTypeDef *hspi_RFM98W;

//    volatile uint8_t interruptFlag_DIO0;
//    volatile uint8_t interruptFlag_DIO5;
    volatile RFM_interruptFlag_t interruptFlag;

    RFM_modulationType Modulation;				//OOK/FSK/GFSK/LORA
    RFM_moduleType COB;							//Chip on board

 	//common parameter
 	lword Frequency;							//unit: KHz
	byte  OutputPower;							//unit: dBm   range: 2-20 [2dBm~+20dBm]
	word  PreambleLength;						//unit: byte

	uint8_t  FixedPktLength;					//OOK/FSK/GFSK:
												//	 	true-------fixed packet length
												//   	false------variable packet length
												//LoRa:
												//	 	true-------implicit header mode
												//      false------explicit header mode

	uint8_t  CrcDisable;						//OOK/FSK/GFSK:
												//		true-------CRC disable
												//		fasle------CRC enable with CCITT
												//LoRa:
												//		true-------Header indicates CRC off
												//		false------Header indicates CRC on
	byte  PayloadLength;						//PayloadLength is need to be set a value, when FixedPktLength is true.


	//for OOK/FSK/GFSK parameter
	lword SymbolTime;							//unit: ns
	lword Devation;								//unit: KHz
	word  BandWidth;							//unit: KHz
	byte  SyncLength;							//unit: none, range: 1-8[Byte], value '0' is not allowed!
	byte  SyncWord[8];							//In setting packet format, synchronous word contents need to be consistent with SyncLength settings (length).

	//for LoRa parameter
	RFM_sfType SFSel;							//unit: none, range: SF6~SF12
	RFM_bwType BWSel;							//Set the transmitting bandwidth in the LoRa mode. Select one of BW62K, BW125K, BW250K and BW500K
	RFM_crType CRSel;							//Set coding rate in the LoRa mode, select one of CR4_5, CR4_6, CR4_7 and CR4_8.

	RFM_FreqStruct_t FrequencyValue;
 	word BitRateValue;
	word DevationValue;
	byte BandWidthValue;
	byte SFValue;
	byte BWValue;
	byte CRValue;
	uint8_t RsOptimize;

}RFM98_LoRa_data_t;

/* Const declarations (deklaracje stalych) ---------------------------------------------------------------------------*/
/* Variable declarations (deklaracje zmiennych) ----------------------------------------------------------------------*/
/* Function declarations (deklaracje funkcji) ------------------------------------------------------------------------*/

uint8_t RFM98W_LoRa_init(RFM98_LoRa_data_t *_data,
					SPI_HandleTypeDef *_hspi_RFM98W,
					GPIO_TypeDef* _SPI_CS_GPIOx,
					uint16_t _SPI_CS_GPIO_Pin,
					GPIO_TypeDef* _RESET_GPIOx,
					uint16_t _RESET_GPIO_Pin);

void RFM98W_LoRa_vConfig(RFM98_LoRa_data_t *_data);
void RFM98W_LoRa_vGoRx(RFM98_LoRa_data_t *_data);
void RFM98W_LoRa_vGoStandby(void);
void RFM98W_LoRa_vGoSleep(void);
uint8_t RFM98W_LoRa_bSendMessage(RFM98_LoRa_data_t *_data, byte msg[], byte length);
byte RFM98W_LoRa_bGetMessage(RFM98_LoRa_data_t *_data, byte msg[]);
int16_t RFM98W_LoRa_iGetRSSI(void);
int16_t RFM98W_LoRa_iGetRSSIPacket(void);

int32_t RSSI_calculateDistance(int16_t rssi, int16_t txPower);

uint8_t RFM98W_LoRa_checkInterrupt(RFM98_LoRa_data_t *_data);
uint8_t RFM98W_LoRa_ReadRegister(uint8_t addr);

/* Function definition (definicje funkcji) ---------------------------------------------------------------------------*/

#endif /* DB_RFM98W_LORA_H_ */
