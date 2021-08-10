/*
 * Dream-Seekers-esp8266.c
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

#include "headfile.h"
#include "esp8266.h"
#define SINGLEAP 1
/**************************** wifi�����������������޸Ķ����ܶ����ż�ת�����ţ� *****************************/
#if SINGLEAP
#define WIFI_WORKMODE	"2"					//wifi����ģʽ	<mode>	1-Station ģʽ	2-AP ģʽ	3-AP+Station ģʽ

#define WIFI_SSID		"\"LALACHEN\""		//Wife����
#define WIFI_PASSWORD	"\"13456789\""		//Wife����
#define WIFI_CHANNEL	"1"					//Wifeͨ��
#define WIFI_ENCRYPTION	"4"					//Wife���ܷ�ʽ	< ecn >	0-OPEN	1-WEP	2-WPA_PSK	3-WPA2_PSK	4-WPA_WPA2_PSK

#define WIFI_MUXMODE	"0"					//wifi������ģʽ	<mode> 	0-��·����ģʽ	1-��·����ģʽ

#define WIFI_SAVE		"0"					//wifi͸������
#define WIFI_LINKMODE	"\"TCP\""			//wifi����ģʽ	(TCP����UDP)
#define WIFI_IPADDRESS	"\"192.168.4.2\""	//wifi����ip��ַ  (Զ��IP��ַ)
#define WIFI_IPCHANNEL	"8086"				//wifi����ip�˿�

#else
#define WIFI_WORKMODE	"3"					//wifi����ģʽ	<mode>	1-Station ģʽ	2-AP ģʽ	3-AP+Station ģʽ

#define WIFI_SSID		"\"LALACHEN\""		//Wife����
#define WIFI_PASSWORD	"\"13456789\""		//Wife����

#define WIFI_LINKMODE	"\"UDP\""			//wifi����ģʽ	(TCP����UDP)
#define WIFI_IPADDRESS	"\"192.168.4.1\""	//wifi����ip��ַ  (Զ��IP��ַ)
#define WIFI_IPCHANNEL1	"8080"				//wifi����ip�˿ڣ�Զ�˶˿ڣ�
#define WIFI_IPCHANNEL2	"8080"				//wifi����ip�˿ڣ����˶˿ڣ�
#endif

/********************* wifi����ATָ�������=��������Ҫ������Ӳ����ģ������޸ģ� **************************/
WIFI_TABLE TB={
	.Test 			= {"AT",				"OK"},			//ͨѶ����
	.Restart		= {"AT+RST",			"OK"},			//ģ�鸴λ
	.CloseEcho		= {"ATE0",				"OK"},			//�رջ���
	.SetCWMODE 		= {"AT+CWMODE=",		"OK"},			//ѡ�� WIFI Ӧ��ģʽ
	.SetAPParm 		= {"AT+CWSAP=",			"OK"},			//���� AP ģʽ�µĲ���
	.GetLocalIP		= {"AT+CIFSR",			"OK"},			//��ȡ���� IP ��ַ
	.GetLinkStatus 	= {"AT+CIPSTATUS",		"STATUS:5"} ,	//�������״̬
	.SetMUX			= {"AT+CIPMUX=",		"OK"},			//����������
	.SetTransMode 	= {"AT+CIPMODE=",		"OK"},			//����ģ�鴫��ģʽ
	.SetServer 		= {"AT+CIPSERVER=",		"OK"},			//����Ϊ������
	.BuildLink		= {"AT+CIPSTART=",		"OK"},			//��������
	.StartSend		= {"AT+CIPSEND",		"OK"},			//��������

	.SetLinkAP		= {"AT+CWJAP=",			"OK"},			//ѡ����� AP
	.AutoLink		= {"AT+SAVETRANSLINK="	"OK"}			//�����Զ�����
};

uint8 wifi_buff[40];
//-------------------------------------------------------------------------------------------------------------------
//  @name		WIFI_ReadData
//  @brief		��ȡģ�鷵�ص���Ϣ
//  @param		void
//  @return		�Ƿ��ж�ȡ������Ϣ
//  @since      v1.0
//  @note		�ڲ�����
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
uint8 WIFI_ReadData(void)
{
	uint8 res=0,*ptr;
	if(WIFI_GetChar(&res))
	{
		ptr = wifi_buff;
		*(ptr++) = res;						//��ʼ����
		delay_ms(1);
		while(WIFI_GetChar(&res))			//ֱ���������
			*(ptr++) = res;
		while(*(ptr) !=0)
			*(ptr++) = 0;						//��ӽ�����
		return 1;
	}
	else
		return 0;
}

//-------------------------------------------------------------------------------------------------------------------
//  @name		WIFI_CheckCmd
//  @brief		��鷵��ָ����Ԥ���Ƿ������
//  @param		*ack��У���ַ���ָ��
//  @return		ԭ���ջ������״γ���У���ַ�����ָ��
//  @since      v1.0
//  @note		�ڲ�����
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
uint8* WIFI_CheckCmd(uint8 *ack)
{
	char *strx=0;
	strx=strstr((const char*)wifi_buff,(const char*)ack);
	return (uint8*)strx;
}

//-------------------------------------------------------------------------------------------------------------------
//  @name    	WIFI_SendCmd
//  @brief   	���͵���ָ��
//  @param  	par��ָ��ṹ�壻waittime�����ͳ���ʱ��
//  @return     �����Ƿ�ɹ�(0-�ɹ���1-���ɹ�)
//  @since      v1.0
//  @note      
//  Sample usage:               
//-------------------------------------------------------------------------------------------------------------------
uint8 WIFI_SendCmd(WIFI_PARM par,uint16 waittime)
{
	uint8 res=0;
	char datt[20];
	uint8 *cmd = par.cmdCode;
	uint8 *ack = par.replyCode;
	sprintf(datt,"%s\r\n",cmd);
	printf(datt);
	WIFI_SendStr(datt);	//��������
	if(ack&&waittime)		//��Ҫ�ȴ�Ӧ��
	{
		while(--waittime)	//�ȴ�����ʱ
		{
			delay_ms(10);
			if(WIFI_ReadData())
			{
				if(WIFI_CheckCmd(ack))
				{
					printf("OKOKOKOK!\r\n");
					printf((char*)wifi_buff);
					break;//�õ���Ч����
				}
				else
				{
					printf("\r\n\r\n\r\nNONONONO!\r\n");
					printf((char*)wifi_buff);
				}
				memset(wifi_buff,0,strlen(wifi_buff)*sizeof(uint8));
			}
		}
		if(waittime==0)res=1;
	}
	return res;
}


//-------------------------------------------------------------------------------------------------------------------
//  @name		WIFI_SetCmdValue
//  @brief		���ָ�����,ÿ������һ����ظ����ý�������
//  @param		*par��ָ��ṹ��ָ�룻*app����Ҫ��ӵ�ָ�����
//  @return		void
//  @since      v1.0
//  @note
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void WIFI_SetCmdValue(WIFI_PARM *par,char* app)
{
	if(par->cmdCode[strlen((const char*)par->cmdCode)-1] != '=')
		strcat((char*)par->cmdCode,(const char*)",");
	strcat((char*)par->cmdCode,(const char*)app);
}

//-------------------------------------------------------------------------------------------------------------------
//  @name		WIFI_QuitTrans
//  @brief		�˳�͸��
//  @param		void
//  @return		�ɹ��˳�Ϊ0������Ϊ1
//  @since      v1.0
//  @note
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
uint8 WIFI_QuitTrans(void)
{
	delay_ms(15);					//���ڴ�����֡ʱ��(10ms)
	WIFI_PutChar('+');
	delay_ms(15);					//���ڴ�����֡ʱ��(10ms)
	WIFI_PutChar('+');
	delay_ms(15);					//���ڴ�����֡ʱ��(10ms)
	WIFI_PutChar('+');
	delay_ms(25);					//���ڴ�����֡ʱ��(10ms)
	return WIFI_SendCmd(TB.Test,20);//�˳�͸���ж�.
}

#if SINGLEAP
//-------------------------------------------------------------------------------------------------------------------
//  @name		WIFI_Seetings
//  @brief		��WIFI����AP����͸������
//  @param		x-��Ч����
//  @return		��Ч
//  @since      v1.0
//  @note
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void WIFI_Init(void)
{
	WIFI_Com_Init(115200);							//wifi��ʼ��

	while(WIFI_SendCmd(TB.Test,20))					//��������
	{
		WIFI_QuitTrans();							//û��Ӧ˵��û���˳�͸���������˳�͸��
		delay_ms(800);
	}
	WIFI_ShowStr(10,2,"AT\t\tOK!");

	WIFI_SetCmdValue(&TB.SetCWMODE,WIFI_WORKMODE);	//���ù���ģʽ
	WIFI_SendCmd(TB.SetCWMODE,20);					//����
	WIFI_ShowStr(10,3,"AT+CWMODE\t\tOK!");

	WIFI_SendCmd(TB.Restart,500);					//��λ
	delay_ms(1000);
	delay_ms(1000);
	delay_ms(1000);
	WIFI_ShowStr(10,4,"AT+RST\t\tOK!");

	while(WIFI_SendCmd(TB.Test,20))					//��λ���ٲ�������
	{
		WIFI_QuitTrans();							//û��Ӧ˵��û���˳�͸���������˳�͸��
		delay_ms(800);
		WIFI_SetCmdValue(&TB.AutoLink,WIFI_LINKMODE);	//�ر��Զ�͸��
		while(WIFI_SendCmd(TB.AutoLink,200));			//������������
	}

	while(WIFI_SendCmd(TB.CloseEcho,1000));			//�رջ���
	WIFI_ShowStr(10,5,"ATE0\t\tOK!");

	WIFI_SetCmdValue(&TB.SetAPParm,WIFI_SSID);		//����WIFI�˺�
	WIFI_SetCmdValue(&TB.SetAPParm,WIFI_PASSWORD);	//����WIFI����
	WIFI_SetCmdValue(&TB.SetAPParm,WIFI_CHANNEL);	//����ͨ����
	WIFI_SetCmdValue(&TB.SetAPParm,WIFI_ENCRYPTION);//���ü��ܷ�ʽ
	WIFI_SendCmd(TB.SetAPParm,1000);				//����
	WIFI_ShowStr(10,6,"AT+CWSAP\t\tOK!");

	WIFI_SetCmdValue(&TB.SetMUX,WIFI_MUXMODE);		//���õ�����
	WIFI_SendCmd(TB.SetMUX,20);						//������������
	WIFI_ShowStr(10,7,"AT+CIPMUX\t\tOK!");

	WIFI_ShowStr(10,9,"Linking...");
	WIFI_ShowStr(10,10,WIFI_IPADDRESS);
	WIFI_ShowStr(150,10,WIFI_IPCHANNEL);
	WIFI_ShowStr(10,11,"Waiting for connect!!");

	WIFI_SetCmdValue(&TB.BuildLink,WIFI_LINKMODE);	//����TCPЭ�鴫��
	WIFI_SetCmdValue(&TB.BuildLink,WIFI_IPADDRESS);	//�������ӵ�IP��ַ
	WIFI_SetCmdValue(&TB.BuildLink,WIFI_IPCHANNEL);	//�������ӵ�IP�˿ں�
	while(WIFI_SendCmd(TB.BuildLink,200));			//������������
	WIFI_ShowStr(10,12,"AT+CIPSTART\t\tOK!");

	WIFI_SetCmdValue(&TB.SetTransMode,"1");				//���ô���Ϊ͸��ģʽ
	WIFI_SendCmd(TB.SetTransMode,20);					//����

	WIFI_SendCmd(TB.StartSend,20);						//��ʼ����

	WIFI_SendStr("Get it!");
}

#else
void WIFI_Init(void)
{
	WIFI_Com_Init(115200);								//wifi��ʼ��

	while(WIFI_SendCmd(TB.Test,20))						//��������
	{
		WIFI_QuitTrans();								//û��Ӧ˵��û���˳�͸���������˳�͸��
		delay_ms(800);
	}
	WIFI_ShowStr(10,2,"AT\t\tOK!");

	WIFI_SetCmdValue(&TB.SetCWMODE,WIFI_WORKMODE);		//���ù���ģʽ
	WIFI_SendCmd(TB.SetCWMODE,20);						//����
	WIFI_ShowStr(10,3,"AT+CWMODE\t\tOK!");

	WIFI_SendCmd(TB.Restart,500);						//��λ
	delay_ms(1000);
	delay_ms(1000);
	delay_ms(1000);
	WIFI_ShowStr(10,4,"AT+RST\t\tOK!");

	while(WIFI_SendCmd(TB.Test,20))						//��λ���ٲ�������
	{
		WIFI_QuitTrans();								//û��Ӧ˵��û���˳�͸���������˳�͸��
		delay_ms(800);
		WIFI_SetCmdValue(&TB.AutoLink,WIFI_LINKMODE);	//�ر��Զ�͸��
		while(WIFI_SendCmd(TB.AutoLink,200));			//������������
	}

	WIFI_SetCmdValue(&TB.BuildLink,WIFI_LINKMODE);		//����TCPЭ�鴫��
	WIFI_SetCmdValue(&TB.BuildLink,WIFI_IPADDRESS);		//�������ӵ�IP��ַ
	WIFI_SetCmdValue(&TB.BuildLink,WIFI_IPCHANNEL1);	//�������ӵ�IP�˿ں�
	WIFI_SetCmdValue(&TB.BuildLink,WIFI_IPCHANNEL2);	//�������ӵ�IP�˿ں�
	while(WIFI_SendCmd(TB.BuildLink,200));				//������������
	WIFI_ShowStr(10,12,"AT+CIPSTART\t\tOK!");

	WIFI_SetCmdValue(&TB.SetTransMode,"1");				//���ô���Ϊ͸��ģʽ
	WIFI_SendCmd(TB.SetTransMode,20);					//����

	WIFI_SendCmd(TB.StartSend,20);						//��ʼ����

	WIFI_SendStr("Get it!");
}

#endif
//-------------------------------------------------------------------------------------------------------------------
//  @name
//  @brief
//  @param
//  @return
//  @since      v1.0
//  @note
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void TestWIFI(void)
{
	WIFI_Init();
	while(1)
	{
		if(WIFI_ReadData())
		{
			printf((char*)wifi_buff);
			WIFI_SendStr(wifi_buff);
			char dat[50];sprintf(dat,"REC:%20s",wifi_buff);
			WIFI_ShowStr(10,14,dat);
			memset(wifi_buff,0,strlen(wifi_buff)*sizeof(uint8));
		}
		if(GET_KEYCODE()==MY_KEY_CANCLE)
			break;
	}
}

//-------------------------------------------------------------------------------------------------------------------
//  @name
//  @brief
//  @param
//  @return
//  @since      v1.0
//  @note
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void WIFI_Uart_Callback(void)
{
	while(uart_query(ESP8266_CHANNEL, wifi_buff));
	if(strstr((const char*)wifi_buff,"g"))
	{
		StartFlag = StartRun;
//		Chriping(100,3);
		uart_disable(ESP8266_CHANNEL);

	}
}
//-------------------------------------------------------------------------------------------------------------------
//  @name
//  @brief
//  @param
//  @return
//  @since      v1.0
//  @note
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------

