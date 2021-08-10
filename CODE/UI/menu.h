/*
 * Dream-Seekers-menu.h
 * 
 *  Created on:
 *      Author:
 *     Version: V1.0
 *        Core: TC264D
 *
 *   	  Name: 多级菜单显示函数
 *		 Brief: 显示多级菜单
 *	      Note: V2.0
 */

#ifndef _MENU_H_
#define _MENU_H_

/************************************* UI基本接口的配置 *************************************/
//选择使用的 LCD
#ifndef USE_SCREEN
	#define OLED			1
	#define VCANLCD			2
	#define IPS200			3

	#define USE_SCREEN      IPS200
#endif

//配置对应功能按键(可以配置为三键或者四键模式。当配置为三键模式时，上下键可选其一，另一个注释掉宏声明即可)
#include "key.h"							//添加对应头文件
#define MY_KEY_UP			KEY0			//上
#define MY_KEY_DOWN			KEY1//KEY3			//下
#define MY_KEY_ENTER		KEY2			//进入
#define MY_KEY_CANCLE		KEY3//KEY1			//退出

//配置按键接口(注意，按键初始化如果在UI载入前配置，该宏直接空定义即可)
#define GET_KEYCODE()		KEY_Scan()		//按键扫描函数（必须返回为上面的功能按键值）
#define SET_KEYINIT()		KeyInit()		//按键初始化函数

//配置系统延时函数
#include "zf_stm_systick.h"					//添加对应头文件
#define TimeDly(ms)			delay_ms(ms)	//延时（ms)


/************************************* UI界面的配置 *************************************/
//配置字体大小(默认字库里支持3216/3015/2814/2613/2412/2211/2010/1809/1608/1407/1206字体)
//（可以自行添加字体大小，但需保证长宽比，且取模方式为阴码 逐行式 逆向）
#define FontH				26						//字体高
#define FontW				13						//字体宽
#define FontTable			asc_2613				//字体表名称
//配置菜单显示颜色（OLED不需要配置）
#define UI_ForeColor		WHITE					//菜单前景色（文字颜色）
#define UI_BKColor			BLACK					//菜单背景色


/****************************** 对应屏幕驱动——根据情况修改，用户基本无需关心 ************************************/
#if USE_SCREEN == IPS200
#include "SEEKFREE_IPS200_PARALLEL8.h"
	#define INIT_SCREEN()			ips200_init()
	#define	CLEAR_SCREEN(N)			do{\
										int p=0,q=FontH;\
										ips200_address_set(0,(N-1)*FontH,IPS200_X_MAX-1,N*FontH-1);\
										while(q--){\
											for(p=0;p<IPS200_X_MAX;p++){\
												ips200_wr_data16(UI_BKColor);}\
										}\
									   }while(0);
	#define CLEAR_SCREEN_ALL()		do{\
										int i,j;\
										ips200_address_set(0,0,IPS200_X_MAX-1,IPS200_Y_MAX-1);\
										for(i=0;i<IPS200_X_MAX;i++){\
											for (j=0;j<IPS200_Y_MAX;j++){\
												ips200_wr_data16(UI_BKColor);}}\
									   }while(0);
	#define FontWidth				(FontW/8+((FontW%8)?1:0))
	#define SHOW_CHAR(x0,y0,chr,clear)	do{\
										int i,j,temp;\
										for(i=0; i<FontWidth*FontH; i++){\
											ips200_address_set(x0+i%FontWidth*8,y0+i/FontWidth,x0+i%FontWidth*8+7,y0+i/FontWidth);\
											temp = FontTable[(uint16)chr-32][i];\
											for(j=0; j<8; j++){\
												if(clear)	{if(temp&0x01)	ips200_wr_data16(UI_ForeColor);\
															 else			ips200_wr_data16(UI_BKColor);}\
												else		{if(temp&0x01)	ips200_wr_data16(UI_BKColor);\
															 else			ips200_wr_data16(UI_ForeColor);}\
												temp>>=1;}}\
									   }while(0);
	#define SHOW_LIST(FN,N,clear)	do{\
										int k=0;\
										char *dat = sOption[FN+N-1].KeyWord;\
										while(dat[k] != '\0'){\
											SHOW_CHAR(k*FontW,(N-1)*FontH,dat[k],clear);\
											k++;}\
								   	   }while(0);
	#define	SHOW_STR(X,Y,str)		do{\
										int k=0,kx=0,ky=0;\
										char *dat = str;\
										while(dat[k] != '\0'){\
											if(X+(k-kx)*FontW>(IPS200_X_MAX-(FontWidth*8))) {kx=k;ky++;}\
        									if(Y+ky*FontH>(IPS200_Y_MAX-FontH)){break;}\
											SHOW_CHAR(X+(k-kx)*FontW,Y+ky*FontH,dat[k],1);\
											k++;}\
								   	   }while(0);
	#define	SCREEN_Refresh()
#endif

/*********************** MENU接口(根据情况自动对应)——用户无需关心 ******************************/
#define SINIT()				INIT_SCREEN()		//屏幕初始化
#define SCLEAR(N)			CLEAR_SCREEN(N)		//清楚对应菜单栏
#define SCLEARALL()			CLEAR_SCREEN_ALL()	//清除菜单显示区
#define SSHOWSTR(X,Y,str)	SHOW_STR(X,Y,str)	//显示字符
#define SSHOW(FN,N,clear)	SHOW_LIST(FN,N,clear)	//显示对应菜单栏
#define SREFRESH()			SCREEN_Refresh()	//刷屏
#define SKEYSCAN()			GET_KEYCODE()		//获得按键值
#define SKEYINIT()			SET_KEYINIT()		//初始化按键

/*菜单结构定义*/
struct Option
{
    unsigned char KeyLevel;       	//菜单选项所属菜单表号
    unsigned char EnterIndex;     	//选项进入索引号
    unsigned char CancelIndex;    	//选项退出索引号
    char *KeyWord;       			//菜单选项文字描述指针
    void (*func)(int x);	  		//跳转函数指针（创建函数须遵循指针规范）
};

/*函数声明*/
void Display_Menu (unsigned char mode);	//刷新菜单功能函数
void MenuUpOneOption (void);			//菜单上移一项函数
void MenuDownOneOption (void);			//菜单下移一项函数
void MenuCancelOption(void);			//菜单退出功能函数
void MenuReflash (void);				//菜单更新函数
void NoThisFunction(void);				//无需执行的函数提示

#endif

