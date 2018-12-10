/*
 * DB_RFM98W.h
 *
 *  Created on: 2 sie 2018
 *      Author: DBabraj
 */

#ifndef DB_RFM98W_H_
#define DB_RFM98W_H_

/* Includes (zalaczone biblioteki) -----------------------------------------------------------------------------------*/
#ifdef USE_HAL_DRIVER
#include "DB_stm_HAL_drivers.h"	// do obslugi funkcji z bibliotek STM32 HAL
#endif /* USE_HAL_DRIVER */

/********************** NOTES **********************************************
...
*******************************************************************************/
/* Preprocessor definition (definicje preprocesora) ------------------------------------------------------------------*/

#define RFM98_PAYLOAD_LENGTH  55

#define RFM98_REG_FIFO                    0x00
#define RFM98_REG_FIFO_ADDR_PTR           0x0D
#define RFM98_REG_FIFO_TX_BASE_AD         0x0E
#define RFM98_REG_FIFO_RX_BASE_AD         0x0F
#define RFM98_REG_RX_NB_BYTES             0x13
#define RFM98_REG_OPMODE                  0x01
#define RFM98_REG_FIFO_RX_CURRENT_ADDR    0x10
#define RFM98_REG_IRQ_FLAGS               0x12
#define RFM98_REG_RSSI_PACKET             0x1A
#define RFM98_REG_RSSI_CURRENT            0x1B
#define RFM98_REG_DIO_MAPPING_1           0x40
#define RFM98_REG_DIO_MAPPING_2           0x41
#define RFM98_REG_MODEM_CONFIG            0x1D
#define RFM98_REG_MODEM_CONFIG2           0x1E
#define RFM98_REG_MODEM_CONFIG3           0x26
#define RFM98_REG_PAYLOAD_LENGTH          0x22
#define RFM98_REG_IRQ_FLAGS_MASK          0x11
#define RFM98_REG_HOP_PERIOD              0x24
#define RFM98_REG_MODEM_STATUS            0x18
#define RFM98_REG_PACKET_SNR              0x19
#define RFM98_REG_DETECT_OPT              0x31
#define RFM98_REG_DETECTION_THRESHOLD     0x37
#define RFM98_REG_FREQ_ERROR              0x28

// MODES
// MODES
#define RF96_MODE_RX_CONTINUOUS     0x85
#define RF96_MODE_SLEEP             0x80
#define RF96_MODE_STANDBY           0x81
#define RF96_MODE_TX                0x83

// Modem Config 1
#define RFM98_EXPLICIT_MODE               0x00
#define RFM98_IMPLICIT_MODE               0x01

#define RFM98_ERROR_CODING_4_5            0x02
#define RFM98_ERROR_CODING_4_6            0x04
#define RFM98_ERROR_CODING_4_7            0x06
#define RFM98_ERROR_CODING_4_8            0x08

#define RFM98_BANDWIDTH_7K8               0x00
#define RFM98_BANDWIDTH_10K4              0x10
#define RFM98_BANDWIDTH_15K6              0x20
#define RFM98_BANDWIDTH_20K8              0x30
#define RFM98_BANDWIDTH_31K25             0x40
#define RFM98_BANDWIDTH_41K7              0x50
#define RFM98_BANDWIDTH_62K5              0x60
#define RFM98_BANDWIDTH_125K              0x70
#define RFM98_BANDWIDTH_250K              0x80
#define RFM98_BANDWIDTH_500K              0x90

// Modem Config 2

#define RFM98_SPREADING_6                 0x60
#define RFM98_SPREADING_7                 0x70
#define RFM98_SPREADING_8                 0x80
#define RFM98_SPREADING_9                 0x90
#define RFM98_SPREADING_10                0xA0
#define RFM98_SPREADING_11                0xB0
#define RFM98_SPREADING_12                0xC0

#define RFM98_CRC_OFF                     0x00
#define RFM98_CRC_ON                      0x04


// POWER AMPLIFIER CONFIG
#define RFM98_REG_PA_CONFIG               0x09
#define RFM98_PA_MAX_BOOST                0x8F
#define RFM98_PA_LOW_BOOST                0x81
#define RFM98_PA_MED_BOOST                0x8A
#define RFM98_PA_MAX_UK                   0x88
#define RFM98_PA_OFF_BOOST                0x00
#define RFM98_RFO_MIN                     0x00

// LOW NOISE AMPLIFIER
#define RFM98_REG_LNA                     0x0C
#define RFM98_LNA_MAX_GAIN                0x23  // 0010 0011
#define RFM98_LNA_OFF_GAIN                0x00

// Modem Status Bitmasks
#define RFM98_MODEM_STATUS_SIGNAL_DETECTED    0x01
#define RFM98_MODEM_STATUS_SIGNAL_SYNC        0x02
#define RFM98_MODEM_STATUS_RX_IN_PROGRESS  	0x04
#define RFM98_MODEM_STATUS_GOT_HEADER     	0x08
#define RFM98_MODEM_STATUS_MODEM_CLEAR 		0x10

/* Types definition (definicje typow) --------------------------------------------------------------------------------*/

typedef struct{
//	GPIO_TypeDef* SPI_CS_GPIOx;
//	uint16_t SPI_CS_GPIO_Pin;
//	SPI_HandleTypeDef *hspi_RFM98W;
	uint8_t currentMode;
    uint8_t lastMessageFlags;

//    volatile uint8_t interruptFlag_DIO0;
//    volatile uint8_t interruptFlag_DIO5;
    volatile union{
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
    }interruptFlag;
}RFM98_data_t;

/* Const declarations (deklaracje stalych) ---------------------------------------------------------------------------*/
/* Variable declarations (deklaracje zmiennych) ----------------------------------------------------------------------*/
/* Function declarations (deklaracje funkcji) ------------------------------------------------------------------------*/

uint8_t RFM98W_init(RFM98_data_t *_data,
                    SPI_HandleTypeDef *_hspi_RFM98W,
                    GPIO_TypeDef* _SPI_CS_GPIOx,
                    uint16_t _SPI_CS_GPIO_Pin,
                    GPIO_TypeDef* _RST_GPIOx,
                    uint16_t _RST_GPIO_Pin);

int16_t RFM98W_getRSSI(void);
uint8_t RFM98W_checkInterrupt(RFM98_data_t *_data);
int16_t RFM98W_getRSSIPacket(void);
int32_t RFM98W_getFrequencyError(void);
void RFM98W_setMode(RFM98_data_t *_data, uint8_t _newMode);
void RFM98W_setFrequency(float Frequency);
uint8_t RFM98W_setLoRaMode(RFM98_data_t *_data, float Frequency);
void RFM98W_startReceiving(RFM98_data_t *_data);
void RFM98W_setupTX(RFM98_data_t *_data);
int16_t RFM98W_receiveMessage(RFM98_data_t *_data, char *message);
uint8_t RFM98W_getLastMessageFlags(RFM98_data_t *_data);
void RFM98W_sendData(RFM98_data_t *_data, char *buffer, int16_t len);
uint8_t RFM98W_ReadRegister(uint8_t addr);

/* Function definition (definicje funkcji) ---------------------------------------------------------------------------*/

#endif /* DB_RFM98W_H_ */
