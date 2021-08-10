/*
 * Dream-Seekers-key.c
 *
 *  Created on:
 *      Author:
 *     Version: V1.0
 *        Core: TC264D
 *
 *   	  Name: key.c
 *		 Brief: ��������
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
	gpio_init(PIN_KEY0, GPI, 1, PULLUP);		//PIN_KEY0��ʼ��ΪGPIO���ܡ�����ģʽ����ʼ�ߵ�ƽ��������
	gpio_init(PIN_KEY1, GPI, 1, PULLUP);		//PIN_KEY1��ʼ��ΪGPIO���ܡ�����ģʽ����ʼ�ߵ�ƽ��������
	gpio_init(PIN_KEY2, GPI, 1, PULLUP);		//PIN_KEY2��ʼ��ΪGPIO���ܡ�����ģʽ����ʼ�ߵ�ƽ��������
	gpio_init(PIN_KEY3, GPI, 1, PULLUP);		//PIN_KEY3��ʼ��ΪGPIO���ܡ�����ģʽ����ʼ�ߵ�ƽ��������
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
		TimeDly0(15);//ȥ����
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
	return 0;// �ް�������
}

