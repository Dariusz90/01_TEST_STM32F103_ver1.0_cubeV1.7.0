/*
 * DB_ring_buffer.h
 *
 *  Created on: 1 wrz 2016
 *      Author: DBabraj
 */

#ifndef DB_RING_BUFFER_H_
#define DB_RING_BUFFER_H_

/* Includes ------------------------------------------------------------------*/
#ifdef USE_HAL_DRIVER
#include "DB_stm_HAL_drivers.h"	// do obslugi funkcji z bibliotek STM32 HAL
#endif /* USE_HAL_DRIVER */
#include <stdint.h>			//	dla "uint32_t"
#include <stdbool.h>



/*
 *  constants and macros
 */

/** Size of the circular receive buffer, must be power of 2 */
#ifndef UART_RX_BUFFER_SIZE
#define UART_RX_BUFFER_SIZE 256
#endif


/** Size of the circular transmit buffer, must be power of 2 */
#ifndef UART_TX_BUFFER_SIZE
#define UART_TX_BUFFER_SIZE 256
//#define UART_TX_BUFFER_SIZE UART_RX_BUFFER_SIZE
#endif

/*
 * WYKORZYSTANIE SPRZETOWEGO BUFORA DMA DO ODBIORU DANYCH PRZEZ UARTA
 * The UART receiver DMA must be setup as CIRCULAR
 */
#define UART_DMA_CIRCULAR_BUFFER



//	UWAGA! USTAWIC ODPOWIEDNIO PIORYTETY PRZERWAN "NVIC" W STMCUBEMX JAK BY PRZERYWALO
#if 1
typedef struct{
	volatile uint32_t LOWER_RX_GUARD;
	volatile uint8_t RxData[UART_RX_BUFFER_SIZE];
	volatile uint32_t UPPER_RX_GUARD;
	volatile uint16_t RxTail;
	volatile uint16_t RxHead;

	volatile uint32_t LOWER_TX_GUARD;
	volatile uint8_t TxData[UART_TX_BUFFER_SIZE];
	volatile uint32_t UPPER_TX_GUARD;
	volatile uint16_t TxTail;
	volatile uint16_t TxHead;
	volatile uint16_t offset;

	volatile uint8_t uart_rx_flag;
	volatile uint8_t uart_tx_flag;
	volatile uint8_t uart_txrx_flag;

#ifdef HAL_UART_MODULE_ENABLED
	UART_HandleTypeDef *huart;
#endif /* HAL_UART_MODULE_ENABLED */
}RING_BUFFER_DATA;
#else
typedef struct{
	volatile uint8_t data[UART_RX_BUFFER_SIZE];
	volatile uint16_t tail;
	volatile uint16_t head;
#ifdef HAL_UART_MODULE_ENABLED
	UART_HandleTypeDef *huart;
#endif /* HAL_UART_MODULE_ENABLED */
}RING_BUFFER_DATA;
#endif

#if defined(HAL_DMA_MODULE_ENABLED) && defined(HAL_UART_MODULE_ENABLED) && defined(UART_DMA_CIRCULAR_BUFFER)
void ringBuffer_init_DMA(RING_BUFFER_DATA *buffer, UART_HandleTypeDef *huart);
#endif	/* defined(HAL_DMA_MODULE_ENABLED) && defined(HAL_UART_MODULE_ENABLED) && defined(UART_DMA_CIRCULAR_BUFFER) */

bool ringBuffer_checkBounds(RING_BUFFER_DATA *buffer);
void ringBuffer_write(RING_BUFFER_DATA *buffer, uint8_t inputDataFromInterrupt);

uint8_t ringBuffer_read(RING_BUFFER_DATA *buffer);

uint16_t ringBuffer_readBytes(RING_BUFFER_DATA *buffer, uint8_t *_buffer_out, const uint16_t length);

uint8_t ringBuffer_check(RING_BUFFER_DATA *buffer);

uint16_t ringBuffer_checkAvailable(RING_BUFFER_DATA *buffer);

void ringBuffer_flush(RING_BUFFER_DATA *buffer);


//###############################################################################################
 /* defined(HAL_DMA_MODULE_ENABLED) && defined(HAL_UART_MODULE_ENABLED) && defined(UART_DMA_CIRCULAR_BUFFER) */
//###############################################################################################



void ringBuffer_writeTx(RING_BUFFER_DATA *buffer, uint8_t inputData);

void ringBuffer_writeTx_String(RING_BUFFER_DATA *buffer, const char *s );

uint8_t ringBuffer_readTx(RING_BUFFER_DATA *buffer);

uint16_t ringBuffer_readBytesTx(RING_BUFFER_DATA *buffer, uint8_t *_buffer_out, const uint16_t length);

uint8_t ringBuffer_checkTx(RING_BUFFER_DATA *buffer);

uint16_t ringBuffer_checkAvailableTx(RING_BUFFER_DATA *buffer);

void ringBuffer_flushTx(RING_BUFFER_DATA *buffer);


void sendRingBuffer_ToDMA(RING_BUFFER_DATA *buffer, uint8_t *_buffer_out);
void UART_sendRingBuffer_ToDMA(RING_BUFFER_DATA *buffer, uint8_t *_buffer_out);
void UART_sendRingBuffer_ToDMA_fromIT(RING_BUFFER_DATA *buffer, uint8_t *_buffer_out);


#endif /* DB_RING_BUFFER_H_ */
