/*
 * Dream-Seekers-esp8266.h
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

#ifndef CODE_ESP8266_H_
#define CODE_ESP8266_H_

#include "common.h"

#define ESP8266_CHANNEL		UART_3
#define ESP8266_TXD			UART3_TX_P21_7
#define ESP8266_RXD			UART3_RX_P21_6

#define WIFI_Com_Init(baud)	uart_init(ESP8266_CHANNEL, baud, ESP8266_TXD, ESP8266_RXD)
#define WIFI_SendStr(str)	uart_putstr(ESP8266_CHANNEL, (const char*)str)
#define WIFI_GetChar(chr)	uart_query(ESP8266_CHANNEL, chr)
#define WIFI_PutChar(chr)	uart_putchar(ESP8266_CHANNEL, chr)

#define WIFI_ShowStr(x,y,str)	ips200_showstr(x,y,str)

typedef struct PARM{
	uint8 cmdCode[60];
	uint8 replyCode[10];
}WIFI_PARM;

typedef struct TABLETABLE{
	WIFI_PARM Test;				//测试指令
	WIFI_PARM Restart;			//重启模块
	WIFI_PARM CloseEcho;		//关闭回显

	WIFI_PARM SetCWMODE;		//设置工作模式（1-STA，2-AP，3-STA+AP）
	WIFI_PARM SetAPParm;		//设置 AP 模式下的参数（ssid，password，channel，ecn）

	WIFI_PARM GetLocalIP;		//获得本机IP地址
	WIFI_PARM GetLinkStatus;	//获得连接状态
	WIFI_PARM SetMUX;			//设置是否多连接
	WIFI_PARM SetTransMode;		//设置是否透传
	WIFI_PARM SetServer;		//设置是否配置为服务器
	WIFI_PARM BuildLink;		//建立连接
	WIFI_PARM StartSend;		//设置开始发送数据

	WIFI_PARM SetLinkAP;		//选择加入 AP
	WIFI_PARM AutoLink;			//建立自动连接

}WIFI_TABLE;

uint8 WIFI_SendCmd(WIFI_PARM par,uint16 waittime);
void WIFI_Init(void);
void TestWIFI(void);
void WIFI_Uart_Callback(void);


#endif /* CODE_ESP8266_H_ */
