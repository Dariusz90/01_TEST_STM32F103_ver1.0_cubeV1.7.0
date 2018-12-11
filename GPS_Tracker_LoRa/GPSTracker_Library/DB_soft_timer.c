/*
 * DB_soft_timer.c
 *
 *  Created on: 18 gru 2017
 *      Author: DBabraj
 */

/* Includes (zalaczone biblioteki) -----------------------------------------------------------------------------------*/
#include "DB_soft_timer.h"
#include <stdbool.h>	//	dla true/false


/* Function definition (definicje funkcji) ---------------------------------------------------------------------------*/

// *** SOFT TIMER functions ***
/**
  * @brief  Soft timer handler. Handler must be running on hardware timer interrupt.
  * @param  *_soft_timer: Pointer to timer data table.
  * @param  _st_size: Size of timer data table.
  * @retval void
  */
inline void soft_timer_handler(volatile uint16_t *_soft_timer, const uint8_t _st_size)
{
	static uint16_t x = 0;	//	auxiliary variable
	static uint8_t i = 0;	//	max 255 soft timers

	for (i = 0; i < _st_size; i++) {
		x = _soft_timer[i];
		if(x) _soft_timer[i] = --x;
	}
}


/**
  * @brief  Function for calculate soft timer "interrupt".
  * @param  _soft_timer: 		Pointer to timer data table.
  * @param  _st_number:			Number of the soft timer to be executed.
  * @param  _st_update_rate:	Setpoint time for timer. [ms]
  * @retval TRUE/FALSE.
  */
uint8_t soft_timer_execute(volatile uint16_t *_soft_timer, uint8_t _st_number,uint16_t _st_update_rate)
{
	  if(!_soft_timer[_st_number])
	  {
		  _soft_timer[_st_number] = _st_update_rate;
		  return true;
	  }
	  return false;
}

// *** SOFT TIMER functions ***
