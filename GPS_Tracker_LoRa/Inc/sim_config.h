/*
 * sim_config.h
 *
 *  Created on: 29.08.2018
 *      Author: Prezes
 */

#ifndef SIM_CONFIG_H_
#define SIM_CONFIG_H_
#include "build_defs.h"

#define SIM_COMMAND_RETRIES 3
#define SIM_GLOBAL_RETRIES 3

#define SLEEP_MODE_ENABLE
#define not_ONLY_LORA
#define SLEEP_IN_MINUTES
#ifdef SLEEP_IN_MINUTES	// TAK LEPIEJ WYGLADA
	#define DEFAULT_SLEEP_TIME 5	//	[min]
#else
	#define DEFAULT_SLEEP_TIME 1	//	[h]
#endif

#define WORK_BEFORE_SLEEP

#define SIM_TIMEOUT_LONG  	(15 * 10)	//	(15 * 10) -> 15[s]
#define SIM_TIMEOUT_SHORT  	(1 * 5)
#define SIM_SMS_WAIT_TIME 	( 3 * 60 * 10)	//	(2 * 60 * 10) -> 2[min]
#define SIM_STARTUP_WAIT_TIME ( 3 * 60 * 10)	//	(2 * 60 * 10) -> 2[min]
//#define SIM_SMS_WAIT_TIME (10 * 10) //30 sekund
//#define AGPS_ONLY

#define C

#define TEST_CARD
//#define SMS_GATE_ONLINE
//#define SIM_ORANGE

#ifdef TEST_CARD
// LYCA MOBILE
#define SIM_DESTINATION_PHONE_NUMBER "+48602748679"
//#define SIM_DESTINATION_PHONE_NUMBER "+48888227829"
#define SIM_GPRS_APN "data.lycamobile.pl"
#define SIM_GPRS_USER "lmpl"
#define SIM_GPRS_PASSWORD "plus"
// #define SIM_PIN "0000"

#elif !defined(SIM_ORANGE) // TEST_CARD
// KYIVSTAR
#ifdef SMS_GATE_ONLINE
	#define SIM_DESTINATION_PHONE_NUMBER "+380639348839"
#else
	#define SIM_DESTINATION_PHONE_NUMBER "+380970719813"
#endif

 #define SIM_GPRS_APN "3g.kyivstar.net"
// #define SIM_GPRS_USER
// #define SIM_GPRS_PASSWORD
// #define SIM_PIN "0000"
#endif

#ifdef	SIM_ORANGE
#define SIM_DESTINATION_PHONE_NUMBER "+48602748679"
//#define SIM_DESTINATION_PHONE_NUMBER "+48503992715"
#define SIM_GPRS_APN "internet"
#define SIM_GPRS_USER "internet"
#define SIM_GPRS_PASSWORD "internet"
#endif


#endif /* SIM_CONFIG_H_ */
