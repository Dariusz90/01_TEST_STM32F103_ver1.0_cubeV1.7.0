/*
 * DB_HAL_drivers.h
 *
 *  Created on: 23.06.2017
 *      Author: Darek
 */

#ifndef INC_DB_STM_HAL_DRIVERS_H_
#define INC_DB_STM_HAL_DRIVERS_H_

/* Includes ------------------------------------------------------------------*/

/*
 * Do obslugi funkcji z bibliotek STM32 HAL.
 * Definicja tworzona jest automatycznie i znajduje sie w:
 * "Properties->C/C++ Build->Settings->Tool Settings->Symblols"
 */
#if defined(STM32F401xC) || defined(STM32F407xx) || defined(STM32F429xx) || defined(STM32F413xx) || defined(STM32F405xx)
#include "stm32f4xx_hal.h"

#ifndef STM32F4_FAMILY
#define STM32F4_FAMILY
#endif	/* STM32F4_FAMILY */

#endif	/* defined(STM32F401xC) || defined(STM32F407xx) || defined(STM32F429xx) || defined(STM32F413xx) */


#if defined(STM32F103xB) || defined(STM32F103xE)
#include "stm32f1xx_hal.h"

#ifndef STM32F1_FAMILY
#define STM32F1_FAMILY
#endif	/* STM32F1_FAMILY */

#endif	/* defined(STM32F103xB) || defined(STM32F103xE) */


#if defined(STM32L476xx)
#include "stm32l4xx_hal.h"

#ifndef STM32L4_FAMILY
#define STM32L4_FAMILY
#endif	/* STM32L4_FAMILY */

#endif	/* defined(STM32L476xx) */


























#endif /* INC_DB_HAL_DRIVERS_H_ */
