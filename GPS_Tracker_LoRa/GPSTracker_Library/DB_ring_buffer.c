/*
 * DB_ring_buffer.c
 *
 *  Created on: 1 wrz 2016
 *      Author: DBabraj
 */

/* Includes ------------------------------------------------------------------*/
#include "DB_ring_buffer.h"

#include <string.h>		//	dla memset()
#include <stdbool.h>	//	 dla true/false

/*
 *  constants and macros
 */
/* size of RX buffers */
#ifndef UART_RX_BUFFER_MASK
#define UART_RX_BUFFER_MASK ( UART_RX_BUFFER_SIZE - 1)
#endif

#if ( UART_RX_BUFFER_SIZE & UART_RX_BUFFER_MASK )
#error RX buffer size is not a power of 2
#endif

/* size of TX buffers */
#ifndef UART_TX_BUFFER_MASK
#define UART_TX_BUFFER_MASK ( UART_TX_BUFFER_SIZE - 1)
#endif

#if ( UART_TX_BUFFER_SIZE & UART_TX_BUFFER_MASK )
#error TX buffer size is not a power of 2
#endif




/*
** high byte error return code of uart_getc()
*/
#define UART_BUFFER_OVERFLOW  0x02              /* receive ringbuffer overflow */
#define UART_NO_DATA          0x00              /* no receive data available   */

#if defined(HAL_DMA_MODULE_ENABLED) && defined(HAL_UART_MODULE_ENABLED) && defined(UART_DMA_CIRCULAR_BUFFER)
/*************************************************************************
Function: initRingBuffer
Purpose:  initial ring buffer function DMA in circular mode
Input:    *buffer: 	buffer data structure
Input:    *huart: 	UART_HandleTypeDef data structure
TODO:	Insert "uart_tx_flag" and "uart_rx_flag" into interrupt !!!.
Returns:  void
**************************************************************************/
void ringBuffer_init_DMA(RING_BUFFER_DATA *buffer, UART_HandleTypeDef *huart)
{
	/* These uart interrupts halt any ongoing transfer if an error occurs, disable them */
	/* Disable the UART Parity Error Interrupt */
//	 __HAL_UART_DISABLE_IT(huart, UART_IT_PE);
	/* Disable the UART Error Interrupt: (Frame error, noise error, overrun error) */
//	__HAL_UART_DISABLE_IT(huart, UART_IT_ERR);
	/* Disable Half Transfer Interrupt */
//	__HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);

	buffer->huart = huart;

	// Init UART TX,RX interrupt flag
	buffer->uart_tx_flag = true;
	buffer->uart_rx_flag = false;

	buffer->RxTail = 0;
	buffer->LOWER_RX_GUARD=0xBADF00D;
	buffer->UPPER_RX_GUARD=0xDEADCAFE;
	buffer->LOWER_TX_GUARD=0xBADF00D;
	buffer->UPPER_TX_GUARD=0xDEADCAFE;

	HAL_UART_Receive_DMA(huart, (uint8_t *)buffer->RxData, UART_RX_BUFFER_SIZE);
}
#endif	/* defined(HAL_DMA_MODULE_ENABLED) && defined(HAL_UART_MODULE_ENABLED) && defined(UART_DMA_CIRCULAR_BUFFER) */

/*************************************************************************
Function: ringBuffer_write
Purpose:  called when the UART has received a character in interrupt
Input:    *buffer: 	buffer data structure
Input:    inputDataFromInterrupt: 	input data from interrupt
Returns:  void
**************************************************************************/

bool ringBuffer_checkBounds(RING_BUFFER_DATA *buffer)
{
	return
			buffer->LOWER_RX_GUARD==0xBADF00D &&
			buffer->UPPER_RX_GUARD==0xDEADCAFE &&
			buffer->LOWER_TX_GUARD==0xBADF00D &&
			buffer->UPPER_TX_GUARD==0xDEADCAFE;

}

void ringBuffer_write(RING_BUFFER_DATA *buffer, uint8_t inputDataFromInterrupt)
{
#if defined(HAL_DMA_MODULE_ENABLED) && defined(HAL_UART_MODULE_ENABLED) && defined(UART_DMA_CIRCULAR_BUFFER)
	__NOP();
#else

 #if 1	/* WERSJA POPRAWIONA/SKROCONA */
	uint16_t tmphead = 0;
	/* calculate buffer index */
	tmphead = ( buffer->RxHead + 1) & UART_RX_BUFFER_MASK;
	if (tmphead != buffer->RxTail) {
		/* store new index */
		buffer->RxHead = tmphead;
		/* store received data in buffer */
		buffer->RxData[tmphead] = inputDataFromInterrupt;
	}

 #else	/* WERSJA STARSZA */
	volatile uint16_t tmphead = 0;
	volatile uint8_t data = 0;
	data = inputDataFromInterrupt;
	/* calculate buffer index */
	tmphead = ( buffer->head + 1) & UART_RX_BUFFER_MASK;
	if (tmphead == buffer->tail) {
		/* error: receive buffer overflow */
		buffer->error = UART_BUFFER_OVERFLOW;
	}else{
		/* store new index */
		buffer->head = tmphead;
		/* store received data in buffer */
		buffer->data[tmphead] = data;
	}

 #endif

#endif	/* defined(HAL_DMA_MODULE_ENABLED) && defined(HAL_UART_MODULE_ENABLED) && defined(UART_DMA_CIRCULAR_BUFFER) */
}

/*************************************************************************
Function: ringBuffer_read()
Purpose:  return byte from ringbuffer
Input:    *buffer: 	buffer data structure
Returns:  received byte from ringbuffer
**************************************************************************/
uint8_t ringBuffer_read(RING_BUFFER_DATA *buffer)
{
#if defined(HAL_DMA_MODULE_ENABLED) && defined(HAL_UART_MODULE_ENABLED) && defined(UART_DMA_CIRCULAR_BUFFER)
	uint8_t c = 0;
	if(!ringBuffer_check(buffer)){
		return UART_NO_DATA;	/* no data available */
	}

	/*	UWAGA!!! W DMA najpierw zczytujemy "data" a puzniej ustawiamy "ogon" */
	c = buffer->RxData[buffer->RxTail];
	/* calculate /store buffer index */
	buffer->RxTail = (buffer->RxTail + 1) & UART_RX_BUFFER_MASK;
	/* get data from receive buffer */
	return c;
#else
 #if 1	/* WERSJA POPRAWIONA/SKROCONA */
	uint16_t tmptail = 0;

	if(!ringBuffer_check(buffer)){
		return UART_NO_DATA;	/* no data available */
	}

	/* calculate /store buffer index */
	tmptail = (buffer->RxTail + 1) & UART_RX_BUFFER_MASK;
	buffer->RxTail = tmptail;
	/* get data from receive buffer */
	return buffer->RxData[tmptail];

 #else	/* WERSJA STARSZA */

	volatile uint8_t tmptail = 0;
	volatile uint8_t data = 0;

	if(!ringBuffer_check(buffer)){
		buffer->error = UART_NO_DATA;
		return 0x00;	/* no data available */
	}

	/* calculate /store buffer index */
	tmptail = (buffer->tail + 1) & UART_RX_BUFFER_MASK;
	buffer->tail = tmptail;
	/* get data from receive buffer */
	data = buffer->data[tmptail];

	return data;

 #endif
#endif	/* defined(HAL_DMA_MODULE_ENABLED) && defined(HAL_UART_MODULE_ENABLED) && defined(UART_DMA_CIRCULAR_BUFFER) */
}

/*************************************************************************
Function: ringBuffer_readBytes()
Purpose:  read characters from stream into buffer
Input:    *buffer: 	buffer data structure
Input:    *_buffer_out: 	buffer data out
Input:    length: 	length of buffer data out
Returns:  returns the number of characters placed in the buffer
**************************************************************************/
uint16_t ringBuffer_readBytes(RING_BUFFER_DATA *buffer, uint8_t *_buffer_out, const uint16_t length)
{
	uint16_t count = 0;
	while (count < length) {
#if 1
	    *_buffer_out++ = (uint8_t)ringBuffer_read(buffer);
	    count++;
#else
		if(!ringBuffer_check(buffer))
			break;
		int16_t c = ringBuffer_read(buffer);
//	    if (c < 0) break;
	    *_buffer_out++ = (uint8_t)c;
	    count++;
#endif
	  }
	  return count;
}

/*************************************************************************
Function: ringBuffer_check()
Purpose:  check available byte in ringbuffer
Input:    *buffer: 	buffer data structure
Returns:  check available byte in ringbuffer
**************************************************************************/
uint8_t ringBuffer_check(RING_BUFFER_DATA *buffer)
{
#if defined(HAL_DMA_MODULE_ENABLED) && defined(HAL_UART_MODULE_ENABLED) && defined(UART_DMA_CIRCULAR_BUFFER)
	buffer->RxHead = ( (UART_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(buffer->huart->hdmarx) ) & (UART_RX_BUFFER_SIZE - 1) );
#endif	/* defined(HAL_DMA_MODULE_ENABLED) && defined(HAL_UART_MODULE_ENABLED) && defined(UART_DMA_CIRCULAR_BUFFER) */

#if 1	/* WERSJA POPRAWIONA/SKROCONA */
	return buffer->RxHead == buffer->RxTail ? 0 : 1;
#else	/* WERSJA STARSZA */
	if(buffer->head != buffer->tail){
		return 1;
	}else{
		buffer->error = UART_NO_DATA;
		return 0;
	}

#endif

}

/*************************************************************************
Function: ringBuffer_checkAvailable()
Purpose:  check numbers of bytes available  in ringbuffer
Input:    *buffer: 	buffer data structure
Returns:  numbers of bytes available  in ringbuffer
**************************************************************************/
uint16_t ringBuffer_checkAvailable(RING_BUFFER_DATA *buffer)
{
#if defined(HAL_DMA_MODULE_ENABLED) && defined(HAL_UART_MODULE_ENABLED) && defined(UART_DMA_CIRCULAR_BUFFER)
	buffer->RxHead = ( (UART_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(buffer->huart->hdmarx) ) & (UART_RX_BUFFER_SIZE - 1) );
#endif	/* defined(HAL_DMA_MODULE_ENABLED) && defined(HAL_UART_MODULE_ENABLED) && defined(UART_DMA_CIRCULAR_BUFFER) */
	return ((uint16_t)(UART_RX_BUFFER_SIZE + buffer->RxHead - buffer->RxTail)) % UART_RX_BUFFER_SIZE;
}


/*************************************************************************
Function: ringBuffer_flush()
Purpose:  removed any buffered incoming serial data
Input:    *buffer: 	buffer data structure
Returns:  none
**************************************************************************/
void ringBuffer_flush(RING_BUFFER_DATA *buffer)//	wyczyszczenie bufora
{
#if 1
//	buffer->RxHead = 0;
//	buffer->RxTail = 0;
	buffer->RxTail = buffer->RxHead;
	memset((uint8_t*)buffer->RxData, 0, sizeof(buffer->RxData));
#else
	while(ringBuffer_check(buffer)){
		ringBuffer_read(buffer);
	}
#endif
}


//###############################################################################################
 /* defined(HAL_DMA_MODULE_ENABLED) && defined(HAL_UART_MODULE_ENABLED) && defined(UART_DMA_CIRCULAR_BUFFER) */
//###############################################################################################




/*************************************************************************
Function: ringBuffer_writeTx()
Purpose:  write byte to ringbuffer for transmitting via UART
Input:    *buffer: 	buffer data structure
Input:    inputData:	byte to be transmitted
Returns:  none
UWAGA! Przy pierwszym wywolaniu "data" ladowana jest na druga pozycje tablicy buffer->TxData[1],
	   wynika to z 4-lini kodu w funkcji.
**************************************************************************/
void ringBuffer_writeTx(RING_BUFFER_DATA *buffer, uint8_t inputData)
{
	uint16_t tmphead = 0;
	/* calculate buffer index */
	tmphead = ( buffer->TxHead + 1) & UART_TX_BUFFER_MASK;
	if (tmphead != buffer->TxTail) {	// check for free space in buffer
		/* store received data in buffer */
		buffer->TxData[tmphead] = inputData;
		/* store new index */
		buffer->TxHead = tmphead;
	}

}

/*************************************************************************
Function: ringBuffer_writeTx_String()
Purpose:  transmit string to UART
Input:    *buffer: 	buffer data structure
Input:    string to be transmitted
Returns:  none
**************************************************************************/
void ringBuffer_writeTx_String(RING_BUFFER_DATA *buffer, const char *s )
{
    while (*s)
    {
    	ringBuffer_writeTx(buffer, *s++);
    }
}

/*************************************************************************
Function: ringBuffer_readTx()
Purpose:  return byte from ringbuffer
Input:    *buffer: 	buffer data structure
Returns:  received byte from ringbuffer
**************************************************************************/
uint8_t ringBuffer_readTx(RING_BUFFER_DATA *buffer)	//	FUNKCJA UZYWANA GLOWNIE PODCZAS WYSYLANIA DANYCH PRZEZ UARTA PRZEZ FUNKCJE SPRZETOWA
{
	uint16_t tmptail = 0;

	if(!ringBuffer_checkTx(buffer)){
		return UART_NO_DATA;	/* no data available */
	}

	/* calculate /store buffer index */
	tmptail = (buffer->TxTail + 1) & UART_TX_BUFFER_MASK;
	buffer->TxTail = tmptail;
	/* get data from receive buffer */
	return buffer->TxData[tmptail];

}


/*************************************************************************
Function: ringBuffer_readBytesTx()
Purpose:  read characters from stream into buffer
Input:    *buffer: 	buffer data structure
Input:    *_buffer_out: 	buffer data out
Input:    length: 	length of buffer data out
Returns:  returns the number of characters placed in the buffer
**************************************************************************/
uint16_t ringBuffer_readBytesTx(RING_BUFFER_DATA *buffer, uint8_t *_buffer_out, const uint16_t length)
{
	uint16_t count = 0;
	while (count < length) {
#if 1
	    *_buffer_out++ = (uint8_t)ringBuffer_readTx(buffer);
	    count++;
#else
		if(!ringBuffer_checkTx(buffer))
			break;
		int16_t c = ringBuffer_readTx(buffer);
//	    if (c < 0) break;
	    *_buffer_out++ = (uint8_t)c;
	    count++;
#endif
	  }
	  return count;
}


/*************************************************************************
Function: ringBuffer_checkTx()
Purpose:  check available byte in ringbuffer
Input:    *buffer: 	buffer data structure
Returns:  check available byte in ringbuffer
**************************************************************************/
uint8_t ringBuffer_checkTx(RING_BUFFER_DATA *buffer)
{
	return buffer->TxHead == buffer->TxTail ? 0 : 1;
}


/*************************************************************************
Function: ringBuffer_checkAvailableTx()
Purpose:  check numbers of bytes available  in ringbuffer
Input:    *buffer: 	buffer data structure
Returns:  numbers of bytes available  in ringbuffer
**************************************************************************/
uint16_t ringBuffer_checkAvailableTx(RING_BUFFER_DATA *buffer)
{
	return ((uint16_t)(UART_TX_BUFFER_MASK + buffer->TxHead - buffer->TxTail)) % UART_TX_BUFFER_MASK;
}


/*************************************************************************
Function: ringBuffer_flushTx()
Purpose:  removed any buffered incoming serial data
Input:    *buffer: 	buffer data structure
Returns:  none
**************************************************************************/
void ringBuffer_flushTx(RING_BUFFER_DATA *buffer)	//	wyczyszczenie bufora nadawczego
{
#if 1
	buffer->TxHead = 0;
	buffer->TxTail = 0;
	memset((uint8_t*)buffer->TxData, 0, sizeof(buffer->TxData));
#else
	while(ringBuffer_checkTx(buffer)){
		ringBuffer_readTx(buffer);
	}
#endif
}



/*************************************************************************
Function: sendRingBuffer_ToDMA()
Purpose:  write bytes from ring buffer to DMA data buffer out
Input:    *buffer: 	pointer to ring buffer data structure
Input:    *_buffer_out: 	pointer to buffer data out
Returns:  none
INFO: WIELKOSC BUFORA "UART_TX_BUFFER_SIZE" NAJLEPIEJ JAK NAJWIEKSZA.
**************************************************************************/
void sendRingBuffer_ToDMA(RING_BUFFER_DATA *buffer, uint8_t *_buffer_out)
{
	/* Send data to UART */
#ifdef HAL_UART_MODULE_ENABLED
	uint16_t size_uart = 0;

#define _TX_SEND_TYPE	2	// 2


#if _TX_SEND_TYPE == 0
	if( (HAL_UART_GetState(buffer->huart) != HAL_UART_STATE_BUSY_TX) && (HAL_UART_GetState(buffer->huart) != HAL_UART_STATE_BUSY) ){ //	Check flag for the end of the transfer
		// Sprawdzenie w buforze czy sa jakies dane do wyslania
		if( (size_uart = ringBuffer_readBytesTx(buffer, _buffer_out, ringBuffer_checkAvailableTx(buffer))) ){
			HAL_UART_Transmit_DMA(buffer->huart, _buffer_out, size_uart); // Rozpoczecie nadawania danych z wykorzystaniem DMA
//			HAL_UART_Transmit_IT(buffer->huart, _buffer_out, size_uart);	// Rozpoczecie nadawania danych z wykorzystaniem IT
		}
	}
#elif _TX_SEND_TYPE == 1
#ifdef STM32F1_FAMILY
	if( (buffer->huart->State != HAL_UART_STATE_BUSY_TX) && (buffer->huart->State != HAL_UART_STATE_BUSY) )	//	Check flag for the end of the transfer
#elif defined( STM32F4_FAMILY )
	if( (buffer->huart->gState != HAL_UART_STATE_BUSY_TX) && (buffer->huart->gState != HAL_UART_STATE_BUSY) )	//	Check flag for the end of the transfer
#endif
	{
		// Sprawdzenie w buforze czy sa jakies dane do wyslania
		if( (size_uart = ringBuffer_readBytesTx(buffer, _buffer_out, ringBuffer_checkAvailableTx(buffer))) ){
			HAL_UART_Transmit_DMA(buffer->huart, _buffer_out, size_uart); // Rozpoczecie nadawania danych z wykorzystaniem DMA
		}
	}
#elif _TX_SEND_TYPE == 2
	/*
	 * Przy tej procedurze sprawdzania musi byc zalanczone przerwanie od UARTA!
	 */
	if( __HAL_UART_GET_FLAG(buffer->huart, UART_FLAG_TXE) &&  __HAL_UART_GET_FLAG(buffer->huart, UART_FLAG_TC) ){ //	Check flag for the end of the transfer
		// Sprawdzenie w buforze czy sa jakies dane do wyslania
		if( (size_uart = ringBuffer_readBytesTx(buffer, _buffer_out, ringBuffer_checkAvailableTx(buffer))) ){
//			HAL_UART_Transmit_IT(buffer->huart, _buffer_out, size_uart);	// Rozpoczecie nadawania danych z wykorzystaniem IT
			HAL_UART_Transmit_DMA(buffer->huart, _buffer_out, size_uart);	// Rozpoczecie nadawania danych z wykorzystaniem DMA
		}
	}
#elif _TX_SEND_TYPE == 3
	/*
	 * Przy tej procedurze sprawdzania musi byc zalanczone przerwanie od UARTA!
	 */
	if( __HAL_UART_GET_FLAG(buffer->huart, UART_FLAG_TXE) &&  __HAL_UART_GET_FLAG(buffer->huart, UART_FLAG_TC)){ //	Check flag for the end of the transfer
		// Sprawdzenie w buforze czy sa jakies dane do wyslania
		if( (size_uart = ringBuffer_readBytesTx(buffer, _buffer_out, ringBuffer_checkAvailableTx(buffer))) ){
//			HAL_UART_Transmit_IT(buffer->huart, _buffer_out, size_uart);	// Rozpoczecie nadawania danych z wykorzystaniem IT
			HAL_UART_Transmit_DMA(buffer->huart, _buffer_out, size_uart);	// Rozpoczecie nadawania danych z wykorzystaniem DMA
		}
	}
#elif _TX_SEND_TYPE == 4
	// Sprawdzenie w buforze czy sa jakies dane do wyslania
	if( (size_uart = ringBuffer_readBytesTx(buffer, _buffer_out, ringBuffer_checkAvailableTx(buffer))) ){
		HAL_UART_Transmit(buffer->huart, _buffer_out, size_uart, 100);
	}
#elif _TX_SEND_TYPE == 5
	if( __HAL_UART_GET_FLAG(buffer->huart, UART_FLAG_TXE) &&  __HAL_UART_GET_FLAG(buffer->huart, UART_FLAG_TC)){
		/* DMA is free */
		if ( __HAL_DMA_GET_COUNTER(buffer->huart->hdmatx) == 0 ){
			// Sprawdzenie w buforze czy sa jakies dane do wyslania
			if( (size_uart = ringBuffer_readBytesTx(buffer, _buffer_out, ringBuffer_checkAvailableTx(buffer))) ){
//				HAL_UART_Transmit_IT(buffer->huart, _buffer_out, size_uart);	// Rozpoczecie nadawania danych z wykorzystaniem IT
				HAL_UART_Transmit_DMA(buffer->huart, _buffer_out, size_uart);	// Rozpoczecie nadawania danych z wykorzystaniem DMA
			}
		}
	}
#elif _TX_SEND_TYPE == 6
	if( __HAL_UART_GET_FLAG(buffer->huart, UART_FLAG_TXE) &&  __HAL_UART_GET_FLAG(buffer->huart, UART_FLAG_TC) ){
		/* DMA is free */
		if ( (__HAL_DMA_GET_COUNTER(buffer->huart->hdmatx) == 0) && buffer->huart->hdmatx->State == HAL_DMA_STATE_READY ){
			// Sprawdzenie w buforze czy sa jakies dane do wyslania
			if( (size_uart = ringBuffer_readBytesTx(buffer, _buffer_out, ringBuffer_checkAvailableTx(buffer))) ){
//				HAL_UART_Transmit_IT(buffer->huart, _buffer_out, size_uart);	// Rozpoczecie nadawania danych z wykorzystaniem IT
				HAL_UART_Transmit_DMA(buffer->huart, _buffer_out, size_uart);	// Rozpoczecie nadawania danych z wykorzystaniem DMA
			}
		}
	}
#elif _TX_SEND_TYPE == 7
#if defined(STM32F103xE) && defined(STM32F1_FAMILY)
	if( buffer->huart->gState == HAL_UART_STATE_READY )	//	Check flag for the end of the transfer
#elif defined(STM32F1_FAMILY)
	if( buffer->huart->State == HAL_UART_STATE_READY )	//	Check flag for the end of the transfer
#elif defined( STM32F4_FAMILY )
	if( buffer->huart->gState == HAL_UART_STATE_READY )	//	Check flag for the end of the transfer
#endif
	{
		// Sprawdzenie w buforze czy sa jakies dane do wyslania
		if( (size_uart = ringBuffer_readBytesTx(buffer, _buffer_out, ringBuffer_checkAvailableTx(buffer))) ){
			HAL_UART_Transmit_DMA(buffer->huart, _buffer_out, size_uart); // Rozpoczecie nadawania danych z wykorzystaniem DMA
		}
	}
#elif _TX_SEND_TYPE == 8	//	SOFTWARE READ TX INTERRUPT FLAG DECLARED FROM USER
	// Sprawdzenie flagi od przerwania UART TX
	if( buffer->uart_tx_flag ){	// (*uart_tx_flag)
		// Sprawdzenie w buforze czy sa jakies dane do wyslania
		if( (size_uart = ringBuffer_readBytesTx(buffer, _buffer_out, ringBuffer_checkAvailableTx(buffer))) ){
			buffer->uart_tx_flag = false;	// zerowanie flagi od przerwania UARTa
			HAL_UART_Transmit_DMA(buffer->huart, _buffer_out, size_uart); 	// Rozpoczecie nadawania danych z wykorzystaniem DMA

//			if( HAL_UART_Transmit_IT(buffer->huart, _buffer_out, size_uart) == HAL_OK ){
//				buffer->uart_tx_flag = false;	// zerowanie flagi od przerwania UARTa
//			}
		}
	}
#endif

#endif /* HAL_UART_MODULE_ENABLED */

}

void UART_sendRingBuffer_ToDMA(RING_BUFFER_DATA *buffer, uint8_t *_buffer_out)
{
	buffer->offset = ringBuffer_checkAvailableTx(buffer);
	if( buffer->uart_tx_flag && buffer->offset)
	{
		buffer->uart_tx_flag = false;	// zerowanie flagi od przerwania UARTa
		ringBuffer_readBytesTx(buffer, _buffer_out, buffer->offset);
	#ifdef HAL_UART_MODULE_ENABLED
		HAL_UART_Transmit_DMA(buffer->huart, _buffer_out, buffer->offset);
	#endif /* HAL_UART_MODULE_ENABLED */
		buffer->offset = 0;
	}
}

void UART_sendRingBuffer_ToDMA_fromIT(RING_BUFFER_DATA *buffer, uint8_t *_buffer_out)
{
	if(buffer->offset != 0)
	{
		ringBuffer_readBytesTx(buffer, _buffer_out, buffer->offset);
	#ifdef HAL_UART_MODULE_ENABLED
		HAL_UART_Transmit_DMA(buffer->huart, _buffer_out, buffer->offset);
	#endif /* HAL_UART_MODULE_ENABLED */
		buffer->offset = 0;
	}
	else
	{
		buffer->uart_tx_flag = true;
	}
}

