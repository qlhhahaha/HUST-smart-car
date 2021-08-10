/*
 * Dream-Seekers-ultrasonic.h
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

#ifndef CODE_ULTRASONIC_H_
#define CODE_ULTRASONIC_H_

#include "common.h"

#define ULTRASONIC_CHANNEL		UART_0
#define ULTRASONIC_TXD			UART0_TX_P14_0
#define ULTRASONIC_RXD			UART0_RX_P14_1

#define ULTRASONIC_Com_Init(baud)	uart_init(ULTRASONIC_CHANNEL, baud, ULTRASONIC_TXD, ULTRASONIC_RXD)
#define ULTRASONIC_SendStr(str)	uart_putstr(ULTRASONIC_CHANNEL, (const char*)str)
#define ULTRASONIC_GetChar(chr)	uart_query(ULTRASONIC_CHANNEL, chr)
#define ULTRASONIC_PutChar(chr)	uart_putchar(ULTRASONIC_CHANNEL, chr)

#define ULTRASONIC_ShowStr(x,y,str)	ips200_showstr(x,y,str)

void Test_Ultrasonic(void);

#endif /* CODE_ULTRASONIC_H_ */
