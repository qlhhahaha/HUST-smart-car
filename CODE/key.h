/*
 * Dream-Seekers-key.h
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

#ifndef KEY_H_
#define KEY_H_

#define PIN_KEY0	P00_5
#define PIN_KEY1	P00_9
#define PIN_KEY2	P00_6
#define PIN_KEY3	P00_12

#define KEY0	1
#define KEY1	2
#define KEY2	3
#define KEY3	4

#define TimeDly0(ms)	delay_ms(ms)

void KeyInit(void);
unsigned char KEY_Read(unsigned char keyno);
unsigned char KEY_Scan(void);

#endif /* CODE_KEY_H_ */
