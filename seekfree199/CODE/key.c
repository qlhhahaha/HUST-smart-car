/*
 * Dream-Seekers-key.c
 *
 *  Created on:
 *      Author:
 *     Version: V1.0
 *        Core: TC264D
 *
 *   	  Name: key.c
 *		 Brief: 按键代码
 *	      Note:
 */

#include "headfile.h"
#include "key.h"


//------------------------------------------------------------------------------------------------------------------
//  @name
//  @brief
//  @param
//  @return
//  @since      v1.0
//  @note
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void KeyInit(void)
{
	gpio_init(PIN_KEY0, GPI, 1, PULLUP);		//PIN_KEY0初始化为GPIO功能、输入模式、初始高电平、无推挽
	gpio_init(PIN_KEY1, GPI, 1, PULLUP);		//PIN_KEY1初始化为GPIO功能、输入模式、初始高电平、无推挽
	gpio_init(PIN_KEY2, GPI, 1, PULLUP);		//PIN_KEY2初始化为GPIO功能、输入模式、初始高电平、无推挽
	gpio_init(PIN_KEY3, GPI, 1, PULLUP);		//PIN_KEY3初始化为GPIO功能、输入模式、初始高电平、无推挽
}

unsigned char KEY_Read(unsigned char keyno)
{
	switch(keyno)
	{
		case KEY0:
			return gpio_get(PIN_KEY0);
		case KEY1:
			return gpio_get(PIN_KEY1);
		case KEY2:
			return gpio_get(PIN_KEY2);
		case KEY3:
			return gpio_get(PIN_KEY3);
		default:
			return 0XFF;
	}
}

unsigned char KEY_Scan(void)
{
	if(	gpio_get(PIN_KEY0)==0||gpio_get(PIN_KEY1)==0||
	        gpio_get(PIN_KEY2)==0||gpio_get(PIN_KEY3)==0)
	{
		TimeDly0(15);//去抖动
		if(gpio_get(PIN_KEY0)==0)
		{
			TimeDly0(10);
			return KEY0;
		}
		if(gpio_get(PIN_KEY1)==0)
		{
			TimeDly0(10);
			return KEY1;
		}
		if(gpio_get(PIN_KEY2)==0)
		{
			TimeDly0(6);
			return KEY2;
		}
		if(gpio_get(PIN_KEY3)==0)
		{
			TimeDly0(6);
			return KEY3;
		}
	}
	return 0;// 无按键按下
}

