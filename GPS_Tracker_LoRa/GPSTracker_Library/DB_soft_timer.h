/*
 * DB_soft_timer.h
 *
 *  Created on: 18 gru 2017
 *      Author: DBabraj
 */

#ifndef DB_SOFT_TIMER_H_
#define DB_SOFT_TIMER_H_

/* Includes (zalaczone biblioteki) -----------------------------------------------------------------------------------*/
#include <stdint.h>	// dla uint16_t itp.



/* Preprocessor definition (definicje preprocesora) ------------------------------------------------------------------*/

// *** SOFT TIMER functions ***

#ifndef SOFT_TIMER_EXECUTE
/**
  * @brief  Macro for calculate soft timer "interrupt".
  * @param  _soft_timer: 		Pointer to timer data table.
  * @param  _st_number:			Number of the soft timer to be executed.
  * @param  _st_update_rate:	Setpoint time for timer. [ms]
  * @retval TRUE/FALSE.
  */
#define SOFT_TIMER_EXECUTE(_soft_timer, _st_number, _st_update_rate)		( !_soft_timer[(_st_number)] ? (_soft_timer[_st_number] = (_st_update_rate)) : 0 )

/* How to use see below: */
/*
	typedef enum{
		ST_INPUT_TASK,
		ST_CONTROL_TASK,

		ST_SIZE	// MAX SIZE OF SOFT TIMER DATA
	}soft_timer_t;
	volatile uint16_t soft_timer[ST_SIZE];

	//  ST_MAIN_LOOP
	if( SOFT_TIMER_EXECUTE(soft_timer, ST_INPUT_TASK, SOFT_TIMER_INPUT_LOOP_TASK_UPDATE_RATE) )
	{
		HAL_GPIO_TogglePin(QB_DEBUG_LED_GREEN_GPIO_Port, QB_DEBUG_LED_GREEN_Pin);

	}
 */
#endif /* SOFT_TIMER_EXECUTE */

/* Function declarations (deklaracje funkcji) ------------------------------------------------------------------------*/
uint8_t soft_timer_execute(volatile uint16_t *_soft_timer, uint8_t _st_number, uint16_t _st_update_rate);

extern void soft_timer_handler(volatile uint16_t *_soft_timer, const uint8_t _st_size);


#ifndef SOFT_TIMER_SLAVE
/**
  * @brief  Macro for calculate soft timer "interrupt".
  * @param  _time_cnt: 			Memory variable for timer counter.
  * @param  _loop_update_rate:	Time main loop. [ms]
  * @param  _sp_update_rate:	Setpoint time for internal timer on main loop. [ms]
  * @retval TRUE/FALSE.
  */
//#define SOFT_TIMER_SLAVE(_time_cnt, _loop_update_rate, _sp_update_rate)		( ((++_time_cnt)*(_loop_update_rate)) >= (_sp_update_rate) ? 1 : 0 )
#define SOFT_TIMER_SLAVE(_time_cnt, _loop_update_rate, _sp_update_rate)		( ((++_time_cnt)*(_loop_update_rate)) >= (_sp_update_rate) ? !(_time_cnt = 0) : 0 )

/* How to use see below: */
/*
	//	ST_MAIN_LOOP
	if(!soft_timer[ST_MAIN_LOOP])
	{
		soft_timer[ST_MAIN_LOOP] = SOFT_TIMER0_MAIN_LOOP_UPDATE_RATE;
		//  DEBUG LED
		static int32_t led_debug_cnt = 0;
		if( SOFT_TIMER_SLAVE(led_debug_cnt, SOFT_TIMER0_MAIN_LOOP_UPDATE_RATE, DEBUG_LED_UPDATE_RATE) )
		{
			HAL_GPIO_TogglePin(QB_DEBUG_LED_GREEN_GPIO_Port, QB_DEBUG_LED_GREEN_Pin);
		}
		...
	}
 */
#endif /* SOFT_TIMER_SLAVE */


// *** SOFT TIMER functions ***










#endif /* DB_SOFT_TIMER_H_ */
