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
	WIFI_PARM Test;				//����ָ��
	WIFI_PARM Restart;			//����ģ��
	WIFI_PARM CloseEcho;		//�رջ���

	WIFI_PARM SetCWMODE;		//���ù���ģʽ��1-STA��2-AP��3-STA+AP��
	WIFI_PARM SetAPParm;		//���� AP ģʽ�µĲ�����ssid��password��channel��ecn��

	WIFI_PARM GetLocalIP;		//��ñ���IP��ַ
	WIFI_PARM GetLinkStatus;	//�������״̬
	WIFI_PARM SetMUX;			//�����Ƿ������
	WIFI_PARM SetTransMode;		//�����Ƿ�͸��
	WIFI_PARM SetServer;		//�����Ƿ�����Ϊ������
	WIFI_PARM BuildLink;		//��������
	WIFI_PARM StartSend;		//���ÿ�ʼ��������

	WIFI_PARM SetLinkAP;		//ѡ����� AP
	WIFI_PARM AutoLink;			//�����Զ�����

}WIFI_TABLE;

uint8 WIFI_SendCmd(WIFI_PARM par,uint16 waittime);
void WIFI_Init(void);
void TestWIFI(void);
void WIFI_Uart_Callback(void);


#endif /* CODE_ESP8266_H_ */
