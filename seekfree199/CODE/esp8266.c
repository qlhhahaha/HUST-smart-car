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
/**************************** wifi参数配置区（所有修改都不能动引号及转义引号） *****************************/
#if SINGLEAP
#define WIFI_WORKMODE	"2"					//wifi工作模式	<mode>	1-Station 模式	2-AP 模式	3-AP+Station 模式

#define WIFI_SSID		"\"LALACHEN\""		//Wife名称
#define WIFI_PASSWORD	"\"13456789\""		//Wife密码
#define WIFI_CHANNEL	"1"					//Wife通道
#define WIFI_ENCRYPTION	"4"					//Wife加密方式	< ecn >	0-OPEN	1-WEP	2-WPA_PSK	3-WPA2_PSK	4-WPA_WPA2_PSK

#define WIFI_MUXMODE	"0"					//wifi多链接模式	<mode> 	0-单路连接模式	1-多路连接模式

#define WIFI_SAVE		"0"					//wifi透传保存
#define WIFI_LINKMODE	"\"TCP\""			//wifi链接模式	(TCP或者UDP)
#define WIFI_IPADDRESS	"\"192.168.4.2\""	//wifi链接ip地址  (远端IP地址)
#define WIFI_IPCHANNEL	"8086"				//wifi链接ip端口

#else
#define WIFI_WORKMODE	"3"					//wifi工作模式	<mode>	1-Station 模式	2-AP 模式	3-AP+Station 模式

#define WIFI_SSID		"\"LALACHEN\""		//Wife名称
#define WIFI_PASSWORD	"\"13456789\""		//Wife密码

#define WIFI_LINKMODE	"\"UDP\""			//wifi链接模式	(TCP或者UDP)
#define WIFI_IPADDRESS	"\"192.168.4.1\""	//wifi链接ip地址  (远端IP地址)
#define WIFI_IPCHANNEL1	"8080"				//wifi链接ip端口（远端端口）
#define WIFI_IPCHANNEL2	"8080"				//wifi链接ip端口（本端端口）
#endif

/********************* wifi配置AT指令集（带‘=’的是需要后续添加参数的，无需修改） **************************/
WIFI_TABLE TB={
	.Test 			= {"AT",				"OK"},			//通讯测试
	.Restart		= {"AT+RST",			"OK"},			//模块复位
	.CloseEcho		= {"ATE0",				"OK"},			//关闭回显
	.SetCWMODE 		= {"AT+CWMODE=",		"OK"},			//选择 WIFI 应用模式
	.SetAPParm 		= {"AT+CWSAP=",			"OK"},			//设置 AP 模式下的参数
	.GetLocalIP		= {"AT+CIFSR",			"OK"},			//获取本地 IP 地址
	.GetLinkStatus 	= {"AT+CIPSTATUS",		"STATUS:5"} ,	//获得连接状态
	.SetMUX			= {"AT+CIPMUX=",		"OK"},			//启动多连接
	.SetTransMode 	= {"AT+CIPMODE=",		"OK"},			//设置模块传输模式
	.SetServer 		= {"AT+CIPSERVER=",		"OK"},			//配置为服务器
	.BuildLink		= {"AT+CIPSTART=",		"OK"},			//建立连接
	.StartSend		= {"AT+CIPSEND",		"OK"},			//发送数据

	.SetLinkAP		= {"AT+CWJAP=",			"OK"},			//选择加入 AP
	.AutoLink		= {"AT+SAVETRANSLINK="	"OK"}			//建立自动链接
};

uint8 wifi_buff[40];
//-------------------------------------------------------------------------------------------------------------------
//  @name		WIFI_ReadData
//  @brief		读取模块返回的信息
//  @param		void
//  @return		是否有读取到的信息
//  @since      v1.0
//  @note		内部调用
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
uint8 WIFI_ReadData(void)
{
	uint8 res=0,*ptr;
	if(WIFI_GetChar(&res))
	{
		ptr = wifi_buff;
		*(ptr++) = res;						//开始接受
		delay_ms(1);
		while(WIFI_GetChar(&res))			//直到接受完成
			*(ptr++) = res;
		while(*(ptr) !=0)
			*(ptr++) = 0;						//添加结束符
		return 1;
	}
	else
		return 0;
}

//-------------------------------------------------------------------------------------------------------------------
//  @name		WIFI_CheckCmd
//  @brief		检查返回指令与预设是否相符合
//  @param		*ack：校验字符串指针
//  @return		原接收缓冲内首次出现校验字符串的指针
//  @since      v1.0
//  @note		内部调用
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
//  @brief   	发送单个指令
//  @param  	par：指令结构体；waittime：发送超载时间
//  @return     发送是否成功(0-成功，1-不成功)
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
	WIFI_SendStr(datt);	//发送命令
	if(ack&&waittime)		//需要等待应答
	{
		while(--waittime)	//等待倒计时
		{
			delay_ms(10);
			if(WIFI_ReadData())
			{
				if(WIFI_CheckCmd(ack))
				{
					printf("OKOKOKOK!\r\n");
					printf((char*)wifi_buff);
					break;//得到有效数据
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
//  @brief		添加指令参数,每次设置一项，可重复调用进行设置
//  @param		*par：指令结构体指针；*app：需要添加的指令参数
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
//  @brief		退出透传
//  @param		void
//  @return		成功退出为0，否则为1
//  @since      v1.0
//  @note
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
uint8 WIFI_QuitTrans(void)
{
	delay_ms(15);					//大于串口组帧时间(10ms)
	WIFI_PutChar('+');
	delay_ms(15);					//大于串口组帧时间(10ms)
	WIFI_PutChar('+');
	delay_ms(15);					//大于串口组帧时间(10ms)
	WIFI_PutChar('+');
	delay_ms(25);					//大于串口组帧时间(10ms)
	return WIFI_SendCmd(TB.Test,20);//退出透传判断.
}

#if SINGLEAP
//-------------------------------------------------------------------------------------------------------------------
//  @name		WIFI_Seetings
//  @brief		对WIFI进行AP单点透传配置
//  @param		x-无效参数
//  @return		无效
//  @since      v1.0
//  @note
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void WIFI_Init(void)
{
	WIFI_Com_Init(115200);							//wifi初始化

	while(WIFI_SendCmd(TB.Test,20))					//测试连接
	{
		WIFI_QuitTrans();							//没反应说明没有退出透传，这里退出透传
		delay_ms(800);
	}
	WIFI_ShowStr(10,2,"AT\t\tOK!");

	WIFI_SetCmdValue(&TB.SetCWMODE,WIFI_WORKMODE);	//设置工作模式
	WIFI_SendCmd(TB.SetCWMODE,20);					//发送
	WIFI_ShowStr(10,3,"AT+CWMODE\t\tOK!");

	WIFI_SendCmd(TB.Restart,500);					//复位
	delay_ms(1000);
	delay_ms(1000);
	delay_ms(1000);
	WIFI_ShowStr(10,4,"AT+RST\t\tOK!");

	while(WIFI_SendCmd(TB.Test,20))					//复位后再测试连接
	{
		WIFI_QuitTrans();							//没反应说明没有退出透传，这里退出透传
		delay_ms(800);
		WIFI_SetCmdValue(&TB.AutoLink,WIFI_LINKMODE);	//关闭自动透传
		while(WIFI_SendCmd(TB.AutoLink,200));			//发送配置数据
	}

	while(WIFI_SendCmd(TB.CloseEcho,1000));			//关闭回显
	WIFI_ShowStr(10,5,"ATE0\t\tOK!");

	WIFI_SetCmdValue(&TB.SetAPParm,WIFI_SSID);		//设置WIFI账号
	WIFI_SetCmdValue(&TB.SetAPParm,WIFI_PASSWORD);	//设置WIFI密码
	WIFI_SetCmdValue(&TB.SetAPParm,WIFI_CHANNEL);	//设置通道号
	WIFI_SetCmdValue(&TB.SetAPParm,WIFI_ENCRYPTION);//设置加密方式
	WIFI_SendCmd(TB.SetAPParm,1000);				//发送
	WIFI_ShowStr(10,6,"AT+CWSAP\t\tOK!");

	WIFI_SetCmdValue(&TB.SetMUX,WIFI_MUXMODE);		//设置单连接
	WIFI_SendCmd(TB.SetMUX,20);						//发送配置数据
	WIFI_ShowStr(10,7,"AT+CIPMUX\t\tOK!");

	WIFI_ShowStr(10,9,"Linking...");
	WIFI_ShowStr(10,10,WIFI_IPADDRESS);
	WIFI_ShowStr(150,10,WIFI_IPCHANNEL);
	WIFI_ShowStr(10,11,"Waiting for connect!!");

	WIFI_SetCmdValue(&TB.BuildLink,WIFI_LINKMODE);	//设置TCP协议传输
	WIFI_SetCmdValue(&TB.BuildLink,WIFI_IPADDRESS);	//设置链接的IP地址
	WIFI_SetCmdValue(&TB.BuildLink,WIFI_IPCHANNEL);	//设置链接的IP端口号
	while(WIFI_SendCmd(TB.BuildLink,200));			//发送配置数据
	WIFI_ShowStr(10,12,"AT+CIPSTART\t\tOK!");

	WIFI_SetCmdValue(&TB.SetTransMode,"1");				//设置传输为透传模式
	WIFI_SendCmd(TB.SetTransMode,20);					//发送

	WIFI_SendCmd(TB.StartSend,20);						//开始传输

	WIFI_SendStr("Get it!");
}

#else
void WIFI_Init(void)
{
	WIFI_Com_Init(115200);								//wifi初始化

	while(WIFI_SendCmd(TB.Test,20))						//测试连接
	{
		WIFI_QuitTrans();								//没反应说明没有退出透传，这里退出透传
		delay_ms(800);
	}
	WIFI_ShowStr(10,2,"AT\t\tOK!");

	WIFI_SetCmdValue(&TB.SetCWMODE,WIFI_WORKMODE);		//设置工作模式
	WIFI_SendCmd(TB.SetCWMODE,20);						//发送
	WIFI_ShowStr(10,3,"AT+CWMODE\t\tOK!");

	WIFI_SendCmd(TB.Restart,500);						//复位
	delay_ms(1000);
	delay_ms(1000);
	delay_ms(1000);
	WIFI_ShowStr(10,4,"AT+RST\t\tOK!");

	while(WIFI_SendCmd(TB.Test,20))						//复位后再测试连接
	{
		WIFI_QuitTrans();								//没反应说明没有退出透传，这里退出透传
		delay_ms(800);
		WIFI_SetCmdValue(&TB.AutoLink,WIFI_LINKMODE);	//关闭自动透传
		while(WIFI_SendCmd(TB.AutoLink,200));			//发送配置数据
	}

	WIFI_SetCmdValue(&TB.BuildLink,WIFI_LINKMODE);		//设置TCP协议传输
	WIFI_SetCmdValue(&TB.BuildLink,WIFI_IPADDRESS);		//设置链接的IP地址
	WIFI_SetCmdValue(&TB.BuildLink,WIFI_IPCHANNEL1);	//设置链接的IP端口号
	WIFI_SetCmdValue(&TB.BuildLink,WIFI_IPCHANNEL2);	//设置链接的IP端口号
	while(WIFI_SendCmd(TB.BuildLink,200));				//发送配置数据
	WIFI_ShowStr(10,12,"AT+CIPSTART\t\tOK!");

	WIFI_SetCmdValue(&TB.SetTransMode,"1");				//设置传输为透传模式
	WIFI_SendCmd(TB.SetTransMode,20);					//发送

	WIFI_SendCmd(TB.StartSend,20);						//开始传输

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

