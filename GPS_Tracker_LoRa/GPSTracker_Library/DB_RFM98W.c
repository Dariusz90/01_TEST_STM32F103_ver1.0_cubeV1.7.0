/*
 * DB_RFM98W.c
 *
 *  Created on: 2 sie 2018
 *      Author: DBabraj
 */

/* Includes (zalaczone biblioteki) -----------------------------------------------------------------------------------*/
#include "DB_RFM98W.h"

#include <string.h>
#include <stdbool.h>

/********************** NOTES **********************************************
 https://github.com/darksidelemm/RFM98Arduino
**Receiver Status**
	STATUS,rssi,modem_status\r\n
    	rssi = Current RSSI, in dBm.
    	modem_status = Contents of RFM98W ReqModemStat register. ** bits 7-5 = Coding rate of last header receiver ** bit 4 = Modem clear ** bit 3 = Header info valid ** bit 2 = RX on-going ** bit 1 = Signal synchronised ** bit 0 = Signal detected

**Data Packet**
	PKT,packet_length\r\n
    	Immediately followed by packet_length bytes of binary data

**Data Packet Info**
	PKTINFO,packet_rssi,packet_snr,pkt_freqerror,irq_flags\r\n
    	packet_rssi = RSSI of last received packet (dBm)
    	packet_snr = SNR of last received packet (dBm*4 - see page 105 of RFM98W datasheet)
    	pkt_freqerror = Contents of RFM98W's RegFeiMSB/Mid/LSB registers.
    	irq_flags = Contents of RFM98W's RegIrqFlags Register.

 Usage:
 float Frequency = 431.650e6;
 float FrequencyOffset = 0.0; // TODO: Automatic Frequency Correction
 char message[256];
 SETUP:
	rfm.setLoRaMode(Frequency + FrequencyOffset);
	rfm.startReceiving();
 LOOP:
	rx_status();

	if(rfm.checkInterrupt())
	{
  	  get_packet();
	}
	delay(1000);

void get_packet(){
    int packet_size = rfm.receiveMessage(message);
    packet_info(); // Send packet info message *before* we send the packet itself.
    Serial.print("PKT,");
    Serial.println(packet_size);
    Serial.write((uint8_t*)message, packet_size);
}

void rx_status(){
    uint8_t modem_status = rfm.readRegister(REG_MODEM_STATUS);
    int16_t rssi = rfm.getRSSI();
    uint8_t freq_error = rfm.readRegister(REG_FREQ_ERROR);

    uint8_t signal_detected = (modem_status&MODEM_STATUS_SIGNAL_DETECTED)>0;
    uint8_t signal_sync = (modem_status&MODEM_STATUS_SIGNAL_SYNC)>0;
    uint8_t rx_in_progress = (modem_status&MODEM_STATUS_RX_IN_PROGRESS)>0;
    uint8_t got_header = (modem_status&MODEM_STATUS_GOT_HEADER)>0;

    Serial.print("STATUS,");
    Serial.print(rssi);
    Serial.print(",");
    Serial.print(modem_status);
    Serial.println("");
}

void packet_info(){
    int16_t packet_rssi = rfm.getRSSIPacket();
    int8_t packet_snr = rfm.readRegister(REG_PACKET_SNR);
    int32_t freq_error = rfm.getFrequencyError();
    uint8_t lastMessageFlags = rfm.getLastMessageFlags();

    Serial.print("PKTINFO,");
    Serial.print(packet_rssi);
    Serial.print(",");
    Serial.print(packet_snr);
    Serial.print(",");
    Serial.print(freq_error);
    Serial.print(",");
    Serial.print(lastMessageFlags);
    Serial.println("");
}
*******************************************************************************/
/* Preprocessor definition (definicje preprocesora) ------------------------------------------------------------------*/

/* KONFIGURACJA -> TIMEOUT */
#define RFM98W_SPI_TIMEOUT		50	//	100

/* Types definition (definicje typow) --------------------------------------------------------------------------------*/
/* Const declarations (deklaracje stalych) ---------------------------------------------------------------------------*/
/* Variable declarations (deklaracje zmiennych) ----------------------------------------------------------------------*/
static SPI_HandleTypeDef *hspi_RFM98W;
static GPIO_TypeDef* SPI_CS_GPIOx;
static uint16_t SPI_CS_GPIO_Pin;
static GPIO_TypeDef* RST_GPIOx;
static uint16_t RST_GPIO_Pin;

/* Function declarations (deklaracje funkcji) ------------------------------------------------------------------------*/
static uint8_t WriteRegister(uint8_t addr, uint8_t value);
static uint8_t ReadRegister(uint8_t addr);
static void SPI_CS_pin(GPIO_PinState _PinState);
static void RST_pin(GPIO_PinState _PinState);

/* Function definition (definicje funkcji) ---------------------------------------------------------------------------*/

uint8_t RFM98W_init(RFM98_data_t *_data,
					SPI_HandleTypeDef *_hspi_RFM98W,
					GPIO_TypeDef* _SPI_CS_GPIOx,
					uint16_t _SPI_CS_GPIO_Pin,
					GPIO_TypeDef* _RST_GPIOx,
					uint16_t _RST_GPIO_Pin)
{
	uint8_t status = 0;
	hspi_RFM98W = _hspi_RFM98W;
	SPI_CS_GPIOx = _SPI_CS_GPIOx;
	SPI_CS_GPIO_Pin = _SPI_CS_GPIO_Pin;
	RST_GPIOx = _RST_GPIOx;
	RST_GPIO_Pin = _RST_GPIO_Pin;

	_data->currentMode = 0x81;
	_data->lastMessageFlags = 0x00;

	RST_pin(GPIO_PIN_SET);
	return status;
}


int16_t RFM98W_getRSSI(void)
{
    uint8_t rssi = ReadRegister(RFM98_REG_RSSI_CURRENT);
    return (int16_t)rssi - 137;
}

uint8_t RFM98W_checkInterrupt(RFM98_data_t *_data)
{
//    return digitalRead(this->DIO0_PIN);
	if( _data->interruptFlag.bit.DIO0 )
	{
		_data->interruptFlag.bit.DIO0 = false;
		return true;
	}
	return false;
}

int16_t RFM98W_getRSSIPacket(void)
{
    uint8_t rssi = ReadRegister(RFM98_REG_RSSI_PACKET);
    return (int16_t)rssi - 137;
}

int32_t RFM98W_getFrequencyError(void)
{
	int32_t Temp;

	Temp = (int32_t)ReadRegister(RFM98_REG_FREQ_ERROR) & 7;
	Temp <<= 8L;
	Temp += (int32_t)ReadRegister(RFM98_REG_FREQ_ERROR+1);
	Temp <<= 8L;
	Temp += (int32_t)ReadRegister(RFM98_REG_FREQ_ERROR+2);

	if(ReadRegister(RFM98_REG_FREQ_ERROR) & 8)
	{
		Temp = Temp - 524288;
	}

	return Temp;
}

void RFM98W_setMode(RFM98_data_t *_data, uint8_t _newMode)
{
	if(_newMode == _data->currentMode)
		return;

	switch (_newMode)
	{
		case RF96_MODE_TX:
//			Serial.println("Changing to Transmit Mode");
			WriteRegister(RFM98_REG_LNA, RFM98_LNA_OFF_GAIN);  // TURN LNA OFF FOR TRANSMITT
			WriteRegister(RFM98_REG_PA_CONFIG, RFM98_PA_MAX_UK);
			WriteRegister(RFM98_REG_OPMODE, _newMode);
			_data->currentMode = _newMode;
			break;
		case RF96_MODE_RX_CONTINUOUS:
			WriteRegister(RFM98_REG_PA_CONFIG, RFM98_PA_OFF_BOOST);  // TURN PA OFF FOR RECIEVE??
			WriteRegister(RFM98_REG_LNA, RFM98_LNA_MAX_GAIN);  // LNA_MAX_GAIN);  // MAX GAIN FOR RECIEVE
			WriteRegister(RFM98_REG_OPMODE, _newMode);
			_data->currentMode = _newMode;
//			Serial.println("Changing to Receive Continuous Mode\n");
			break;
		case RF96_MODE_SLEEP:
//			Serial.println("Changing to Sleep Mode");
			WriteRegister(RFM98_REG_OPMODE, _newMode);
			_data->currentMode = _newMode;
			break;
		case RF96_MODE_STANDBY:
//			Serial.println("Changing to Standby Mode");
			WriteRegister(RFM98_REG_OPMODE, _newMode);
			_data->currentMode = _newMode;
			break;

		default:
			return;
	}

	if(_newMode != RF96_MODE_SLEEP)
	{
		while(_data->interruptFlag.bit.DIO5 == 0)	//	digitalRead(this->DIO5_PIN) == 0
		{
			_data->interruptFlag.bit.DIO5 = 1;
//			Serial.print("z");
		}
	}

//	Serial.println(" Mode Change Done");
	return;
}


/*
void RFM98W_setFrequency(void)
{
//	WriteRegister(0x06, 0x6C);
//	WriteRegister(0x07, 0x9C);
//	WriteRegister(0x08, 0xCC);
	// 431.650MHz
	WriteRegister(0x06, 0x6B);
	WriteRegister(0x07, 0xE9);
	WriteRegister(0x08, 0x99);
}
*/

void RFM98W_setFrequency(float Frequency)
{
	uint64_t FrequencyValue = 0;;

	Frequency = Frequency * 7110656 / 434000000;
	FrequencyValue = (uint64_t)(Frequency);

	WriteRegister(0x06, (FrequencyValue >> 16) & 0xFF);   // Set frequency
	WriteRegister(0x07, (FrequencyValue >> 8) & 0xFF);
	WriteRegister(0x08, FrequencyValue & 0xFF);
}

/**
 * @brief Set LoRa mode.
 * @param
 * @param
 * @retval Mode.
 */
uint8_t RFM98W_setLoRaMode(RFM98_data_t *_data, float Frequency)
{
//	Serial.println("Setting LoRa Mode");
	RFM98W_setMode(_data, RF96_MODE_SLEEP);
	WriteRegister(RFM98_REG_OPMODE, 0x80);

	// frequency
//	RFM98W_setMode(_data, RF96_MODE_SLEEP);
	/*
	WriteRegister(0x06, 0x6C);
	WriteRegister(0x07, 0x9C);
	WriteRegister(0x08, 0xCC);
	*/
	RFM98W_setFrequency(Frequency);

//	Serial.println("LoRa Mode Set");
//	Serial.print("Mode = ");
//	Serial.println(ReadRegister(REG_OPMODE));

	return ReadRegister(RFM98_REG_OPMODE);
}


void RFM98W_startReceiving(RFM98_data_t *_data)
{
	WriteRegister(RFM98_REG_MODEM_CONFIG, RFM98_EXPLICIT_MODE | RFM98_ERROR_CODING_4_8 | RFM98_BANDWIDTH_125K);
	WriteRegister(RFM98_REG_MODEM_CONFIG2, RFM98_SPREADING_10 | RFM98_CRC_ON);
	WriteRegister(0x26, 0x0C);    // 0000 1 1 00
//	Serial.println("Set slow mode");

	WriteRegister(RFM98_REG_DETECT_OPT,0x03);
	WriteRegister(RFM98_REG_DETECTION_THRESHOLD,0x0A);

	WriteRegister(RFM98_REG_PAYLOAD_LENGTH, RFM98_PAYLOAD_LENGTH);
	WriteRegister(RFM98_REG_RX_NB_BYTES, RFM98_PAYLOAD_LENGTH);


	WriteRegister(RFM98_REG_HOP_PERIOD,0xFF);
	WriteRegister(RFM98_REG_FIFO_ADDR_PTR, ReadRegister(RFM98_REG_FIFO_RX_BASE_AD));

	// Setup Receive Continous Mode
	RFM98W_setMode(_data, RF96_MODE_RX_CONTINUOUS);
}


void RFM98W_setupTX(RFM98_data_t *_data)
{
	WriteRegister(RFM98_REG_MODEM_CONFIG, RFM98_EXPLICIT_MODE | RFM98_ERROR_CODING_4_8 | RFM98_BANDWIDTH_125K);

	WriteRegister(RFM98_REG_MODEM_CONFIG2, RFM98_SPREADING_10 | RFM98_CRC_ON);

	WriteRegister(0x26, 0x0C);    // 0000 1 1 00
	WriteRegister(RFM98_REG_PAYLOAD_LENGTH, RFM98_PAYLOAD_LENGTH);
	WriteRegister(RFM98_REG_RX_NB_BYTES, RFM98_PAYLOAD_LENGTH);

	// Change the DIO mapping to 01 so we can listen for TxDone on the interrupt
	WriteRegister(RFM98_REG_DIO_MAPPING_1, 0x40);
	WriteRegister(RFM98_REG_DIO_MAPPING_2 ,0x00);

	// Go to standby mode
	RFM98W_setMode(_data, RF96_MODE_STANDBY);
}


int16_t RFM98W_receiveMessage(RFM98_data_t *_data, char *message)
{
	int16_t i = 0, Bytes = 0, currentAddr = 0;

	int16_t x = ReadRegister(RFM98_REG_IRQ_FLAGS);
	_data->lastMessageFlags = x;
	// printf("Message status = %02Xh\n", x);

	// clear the rxDone flag
	// writeRegister(REG_IRQ_FLAGS, 0x40);
	WriteRegister(RFM98_REG_IRQ_FLAGS, 0xFF);

	// check for payload crc issues (0x20 is the bit we are looking for
	if((x & 0x20) == 0x20)
	{
		// printf("CRC Failure %02Xh!!\n", x);
		// reset the crc flags
		WriteRegister(RFM98_REG_IRQ_FLAGS, 0x20);
	}
	else
	{
		currentAddr = ReadRegister(RFM98_REG_FIFO_RX_CURRENT_ADDR);
		Bytes = ReadRegister(RFM98_REG_RX_NB_BYTES);
		// printf ("%d bytes in packet\n", Bytes);

		// printf("RSSI = %d\n", ReadRegister(REG_RSSI) - 137);

		WriteRegister(RFM98_REG_FIFO_ADDR_PTR, currentAddr);
		// now loop over the fifo getting the data
		for(i = 0; i < Bytes; i++)
		{
			message[i] = (uint8_t)ReadRegister(RFM98_REG_FIFO);
		}
		message[Bytes] = '\0';

		// writeRegister(REG_FIFO_ADDR_PTR, 0);  // currentAddr);
	}

	return Bytes;
}


uint8_t RFM98W_getLastMessageFlags(RFM98_data_t *_data)
{
    return _data->lastMessageFlags;
}

void RFM98W_sendData(RFM98_data_t *_data, char *buffer, int16_t len)
{
	int16_t Length = 0;
	uint8_t data[1] = {0};

	if(len==0)
	{
		Length = strlen(buffer);
	}

	//Serial.print("Sending "); Serial.print(Length);Serial.println(" bytes");
	//Serial.println(buffer);

	RFM98W_setMode(_data, RF96_MODE_STANDBY);

	WriteRegister(RFM98_REG_FIFO_TX_BASE_AD, 0x00);  // Update the address ptr to the current tx base address
	WriteRegister(RFM98_REG_FIFO_ADDR_PTR, 0x00);

	SPI_CS_pin(GPIO_PIN_RESET);
	// tell SPI which address you want to write to
//	SPI.transfer(REG_FIFO | 0x80);
	data[0] = (RFM98_REG_FIFO | 0x80);
	HAL_SPI_Transmit(hspi_RFM98W, data, 1, RFM98W_SPI_TIMEOUT);

	// loop over the payload and put it on the buffer
	int16_t i = 0;
	for(i = 0; i < RFM98_PAYLOAD_LENGTH; i++)
	{
		if(i < Length)
		{
//			SPI.transfer(buffer[i] & 0x7F);
			data[0] = (buffer[i] & 0x7F);
			HAL_SPI_Transmit(hspi_RFM98W, data, 1, RFM98W_SPI_TIMEOUT);
//			Serial.write(buffer[i]);
		}
		else
		{
//			SPI.transfer(0);
			data[0] = 0;
			HAL_SPI_Transmit(hspi_RFM98W, data, 1, RFM98W_SPI_TIMEOUT);
		}
	}
	SPI_CS_pin(GPIO_PIN_SET);


	// go into transmit mode
	RFM98W_setMode(_data, RF96_MODE_TX);
}


/**
  * @brief  Read byte from register.
  * @param  addr: address
  * @retval register value.
  */
uint8_t RFM98W_ReadRegister(uint8_t addr)
{
	return ReadRegister(addr);
}


/**
  * @brief  Transmit byte to register.
  * @param  addr: address
  * @param  value: register value.
  * @retval HAL status
  */
static uint8_t WriteRegister(uint8_t addr, uint8_t value)
{
	uint8_t data[2] = {0, 0};
	uint8_t status = 0;
	SPI_CS_pin(GPIO_PIN_RESET);
	data[0] = (addr | 0x80);
#if 0
	status = HAL_SPI_Transmit(hspi_RFM98W, data, 1, RFM98W_SPI_TIMEOUT);
	status = HAL_SPI_Transmit(hspi_RFM98W, &value, 1, RFM98W_SPI_TIMEOUT);
#else
	data[0] = (addr | 0x80);
	data[1] = value;
	status = HAL_SPI_Transmit(hspi_RFM98W, data, 2, RFM98W_SPI_TIMEOUT);
#endif
	SPI_CS_pin(GPIO_PIN_SET);
	return status;
}

/**
  * @brief  Read byte from register.
  * @param  addr: address
  * @retval register value.
  */
static uint8_t ReadRegister(uint8_t addr)
{
	uint8_t data[1] = {0};
	SPI_CS_pin(GPIO_PIN_RESET);
	data[0] = (addr & 0x7F);
	HAL_SPI_Transmit(hspi_RFM98W, data, 1, RFM98W_SPI_TIMEOUT);
	HAL_SPI_Receive(hspi_RFM98W, data, 1, RFM98W_SPI_TIMEOUT);
	SPI_CS_pin(GPIO_PIN_SET);
	return data[0];
}

inline static void RST_pin(GPIO_PinState _PinState)
{
    HAL_GPIO_WritePin(RST_GPIOx, RST_GPIO_Pin, _PinState);
}

inline static void SPI_CS_pin(GPIO_PinState _PinState)
{
	HAL_GPIO_WritePin(SPI_CS_GPIOx, SPI_CS_GPIO_Pin, _PinState);
}
