/*
 * stm_interrupt.c
 *
 *  Created on: 1 gru 2017
 *      Author: DBabraj
 */


/* Includes (zalaczone biblioteki) -----------------------------------------------------------------------------------*/
#include "common_fun.h"




/* Function definition (definicje funkcji) ---------------------------------------------------------------------------*/

/* USER CODE BEGIN 4 */

void HAL_SYSTICK_Callback(void)
{

	soft_timer_handler(soft_timer, ST_SIZE);

}

#ifdef HAL_UART_MODULE_ENABLED
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART2){	//	current UART
		uart2_buffer_gps.uart_tx_flag = true;
	}
	else if(huart->Instance == USART3){	//	current UART
		uart3_buffer_gsm.uart_tx_flag = true;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART2){	//	current UART
		uart2_buffer_gps.uart_rx_flag = true;
		// DMA Mode: Circular jest wlaczony dlatego te linie sa zakomentowane, nie trzeba zalanczac od nowa przerwania
	}
	else if(huart->Instance == USART3){	//	current UART
		uart3_buffer_gsm.uart_rx_flag = true;
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART2){	//	current UART
		__HAL_UART_CLEAR_OREFLAG(huart);	/*!< Overrun error       */
		__HAL_UART_CLEAR_NEFLAG(huart);		/*!< Noise error         */
		__HAL_UART_CLEAR_FEFLAG(huart);		/*!< Frame error         */

		uart2_buffer_gps.uart_tx_flag = true;

		// restart receiving
		HAL_UART_Receive_DMA(huart, (uint8_t *)uart2_buffer_gps.RxData, UART_RX_BUFFER_SIZE);
	}
	else if(huart->Instance == USART3){	//	current UART
		__HAL_UART_CLEAR_OREFLAG(huart);	/*!< Overrun error       */
		__HAL_UART_CLEAR_NEFLAG(huart);		/*!< Noise error         */
		__HAL_UART_CLEAR_FEFLAG(huart);		/*!< Frame error         */

		// restart receiving
		HAL_UART_Receive_DMA(huart, (uint8_t *)uart3_buffer_gsm.RxData, UART_RX_BUFFER_SIZE);
	}

}
#endif	/* HAL_UART_MODULE_ENABLED */


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

#ifdef DB_RFM98W_H_
  switch (GPIO_Pin) {
	  case IB_EXTI2_RFM95_DIO0_Pin:
		  RFM98_data.interruptFlag.bit.DIO0 = true;
		  break;

	  default:
		  break;
  }

#else
  (void)GPIO_Pin;
#endif	/* RC_FLAYSKY_ON */
}

/* USER CODE END 4 */
