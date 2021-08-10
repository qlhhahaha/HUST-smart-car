/*
 * Dream-Seekers-led.h
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

#ifndef CODE_LED_H_
#define CODE_LED_H_

#define PIN_LED0	P33_7		//核心板上的LED引脚
#define PIN_LED1	P33_6
#define PIN_LED2	P33_5
#define PIN_LED3	P33_4

#define PIN_LED4	P33_7		//母板上的LED引脚
#define PIN_LED5	P33_6
#define PIN_LED6	P33_5
#define PIN_LED7	P33_4

//定义模块号
typedef enum
{
  LED0=0,
  LED1,
  LED2,
  LED3,
  LEDCORE,		//核心板上的
  LED4,
  LED5,
  LED6,
  LED7,
  LEDOUT,		//母板上的
} LEDn_e;

typedef enum
{
  ON=0,  //亮
  OFF=1, //灭
  RVS=2, //反转
}LEDs_e;

extern void LED_Init(void);
extern void LED_Ctrl(LEDn_e ledno, LEDs_e sta);
extern void Test_LED(void);




#endif /* CODE_LED_H_ */
