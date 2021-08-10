/*
 * Dream-Seekers-led.c
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
#include "key.h"
//-------------------------------------------------------------------------------------------------------------------
//  @name    LED_Init
//  @brief   初始化LED所用IO口
//  @param  		    
//  @return     				
//  @since      v1.0
//  @note      
//  Sample usage:   LED_Init()
//-------------------------------------------------------------------------------------------------------------------
void LED_Init(void)
{
	gpio_init(PIN_LED0, GPO, 1, PULLUP);		//PIN_KEY0初始化为GPIO功能、输出模式、初始高电平、无推挽
	gpio_init(PIN_LED1, GPO, 1, PULLUP);		//PIN_KEY1初始化为GPIO功能、输出模式、初始高电平、无推挽
	gpio_init(PIN_LED2, GPO, 1, PULLUP);		//PIN_KEY2初始化为GPIO功能、输出模式、初始高电平、无推挽
	gpio_init(PIN_LED3, GPO, 1, PULLUP);		//PIN_KEY3初始化为GPIO功能、输出模式、初始高电平、无推挽

	gpio_init(PIN_LED4, GPO, 1, PULLUP);		//PIN_KEY0初始化为GPIO功能、输出模式、初始高电平、无推挽
	gpio_init(PIN_LED5, GPO, 1, PULLUP);		//PIN_KEY1初始化为GPIO功能、输出模式、初始高电平、无推挽
	gpio_init(PIN_LED6, GPO, 1, PULLUP);		//PIN_KEY2初始化为GPIO功能、输出模式、初始高电平、无推挽
	gpio_init(PIN_LED7, GPO, 1, PULLUP);		//PIN_KEY3初始化为GPIO功能、输出模式、初始高电平、无推挽
}

//-------------------------------------------------------------------------------------------------------------------
//  @name    LED_Ctrl
//  @brief   控制LED灯
//  @param
//  @return
//  @since      v1.0
//  @note
//  Sample usage:   LED_Ctrl(LEDCORE, ON)
//-------------------------------------------------------------------------------------------------------------------
void LED_Ctrl(LEDn_e ledno, LEDs_e sta)
{
  switch(ledno)
  {
	  case LED0:
		if(sta==ON)        	gpio_set(PIN_LED0,ON);
		else if(sta==OFF) 	gpio_set(PIN_LED0,OFF);
		else if(sta==RVS) 	gpio_toggle(PIN_LED0);
		break;
	  case LED1:
		if(sta==ON)        	gpio_set(PIN_LED1,ON);
		else if(sta==OFF) 	gpio_set(PIN_LED1,OFF);
		else if(sta==RVS) 	gpio_toggle(PIN_LED1);
		break;
	  case LED2:
		if(sta==ON)        	gpio_set(PIN_LED2,ON);
		else if(sta==OFF) 	gpio_set(PIN_LED2,OFF);
		else if(sta==RVS) 	gpio_toggle(PIN_LED2);
		break;
	  case LED3:
		if(sta==ON)        	gpio_set(PIN_LED3,ON);
		else if(sta==OFF) 	gpio_set(PIN_LED3,OFF);
		else if(sta==RVS) 	gpio_toggle(PIN_LED3);
		break;
	  case LED4:
		if(sta==ON)        	gpio_set(PIN_LED4,ON);
		else if(sta==OFF) 	gpio_set(PIN_LED4,OFF);
		else if(sta==RVS) 	gpio_toggle(PIN_LED4);
		break;
	  case LED5:
		if(sta==ON)        	gpio_set(PIN_LED5,ON);
		else if(sta==OFF) 	gpio_set(PIN_LED5,OFF);
		else if(sta==RVS) 	gpio_toggle(PIN_LED5);
		break;
	  case LED6:
		if(sta==ON)        	gpio_set(PIN_LED6,ON);
		else if(sta==OFF) 	gpio_set(PIN_LED6,OFF);
		else if(sta==RVS) 	gpio_toggle(PIN_LED6);
		break;
	  case LED7:
		if(sta==ON)        	gpio_set(PIN_LED7,ON);
		else if(sta==OFF) 	gpio_set(PIN_LED7,OFF);
		else if(sta==RVS) 	gpio_toggle(PIN_LED7);
		break;
	  case LEDCORE:
		if(sta==ON)
		{
			gpio_set(PIN_LED0,ON);
			gpio_set(PIN_LED1,ON);
			gpio_set(PIN_LED2,ON);
			gpio_set(PIN_LED3,ON);
		}
		else if(sta==OFF)
		{
			gpio_set(PIN_LED0,OFF);
			gpio_set(PIN_LED1,OFF);
			gpio_set(PIN_LED2,OFF);
			gpio_set(PIN_LED3,OFF);
		}
		else if(sta==RVS)
		{
			gpio_toggle(PIN_LED0);
			gpio_toggle(PIN_LED1);
			gpio_toggle(PIN_LED2);
			gpio_toggle(PIN_LED3);
		}
		break;
	  case LEDOUT:
		if(sta==ON)
		{
			gpio_set(PIN_LED4,ON);
			gpio_set(PIN_LED5,ON);
			gpio_set(PIN_LED6,ON);
			gpio_set(PIN_LED7,ON);
		}
		else if(sta==OFF)
		{
			gpio_set(PIN_LED4,OFF);
			gpio_set(PIN_LED5,OFF);
			gpio_set(PIN_LED6,OFF);
			gpio_set(PIN_LED7,OFF);
		}
		else if(sta==RVS)
		{
			gpio_toggle(PIN_LED4);
			gpio_toggle(PIN_LED5);
			gpio_toggle(PIN_LED6);
			gpio_toggle(PIN_LED7);
		}
		break;
	  default:
		break;
  }
}
