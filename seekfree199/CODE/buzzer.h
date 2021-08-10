/*
 * Dream-Seekers-buzzer.h
 * 
 *  Created on:
 *      Author:
 *     Version: V1.0
 *        Core: TC264D
 *
 *   	  Name:
 *		 Brief: 
 *	      Note:
 */

#ifndef CODE_BUZZER_H_
#define CODE_BUZZER_H_


#define buzzerPort	P20_0

#define Buzzer_Init()	gpio_init(buzzerPort, GPO, 0, PUSHPULL)
#define Buzzing()		gpio_set(buzzerPort, 1)
#define NoBuzzing()		gpio_set(buzzerPort, 0)

#define Chriping(dly, t)	do{int times=0;\
								for(times=t;times>0;times--)\
								{\
									Buzzing();\
									delay(dly);\
									NoBuzzing();\
									delay(dly);\
								}\
							}while(0);


#endif /* CODE_BUZZER_H_ */
