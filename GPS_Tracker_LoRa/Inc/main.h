/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define QB_GSM_PWR_ON_Pin GPIO_PIN_1
#define QB_GSM_PWR_ON_GPIO_Port GPIOD
#define USART2_TX_GPS_Pin GPIO_PIN_2
#define USART2_TX_GPS_GPIO_Port GPIOA
#define USART2_RX_GPS_Pin GPIO_PIN_3
#define USART2_RX_GPS_GPIO_Port GPIOA
#define QB_GPS_PWR_ON_Pin GPIO_PIN_4
#define QB_GPS_PWR_ON_GPIO_Port GPIOA
#define IN_2_Pin GPIO_PIN_5
#define IN_2_GPIO_Port GPIOA
#define IN_1_Pin GPIO_PIN_6
#define IN_1_GPIO_Port GPIOA
#define QB_DEBUG_LED_GREEN_Pin GPIO_PIN_7
#define QB_DEBUG_LED_GREEN_GPIO_Port GPIOA
#define IB_EXTI2_RFM95_DIO0_Pin GPIO_PIN_0
#define IB_EXTI2_RFM95_DIO0_GPIO_Port GPIOB
#define IB_EXTI2_RFM95_DIO5_Pin GPIO_PIN_1
#define IB_EXTI2_RFM95_DIO5_GPIO_Port GPIOB
#define USART3_TX_GSM_Pin GPIO_PIN_10
#define USART3_TX_GSM_GPIO_Port GPIOB
#define USART3_RX_GSM_Pin GPIO_PIN_11
#define USART3_RX_GSM_GPIO_Port GPIOB
#define QB_SPI1_CS_Pin GPIO_PIN_12
#define QB_SPI1_CS_GPIO_Port GPIOB
#define RST_RFM_Pin GPIO_PIN_8
#define RST_RFM_GPIO_Port GPIOA
#define QB_GSM_PWR_KEY_Pin GPIO_PIN_15
#define QB_GSM_PWR_KEY_GPIO_Port GPIOA

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
