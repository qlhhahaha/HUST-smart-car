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

#define PIN_LED0	P33_7		//���İ��ϵ�LED����
#define PIN_LED1	P33_6
#define PIN_LED2	P33_5
#define PIN_LED3	P33_4

#define PIN_LED4	P33_7		//ĸ���ϵ�LED����
#define PIN_LED5	P33_6
#define PIN_LED6	P33_5
#define PIN_LED7	P33_4

//����ģ���
typedef enum
{
  LED0=0,
  LED1,
  LED2,
  LED3,
  LEDCORE,		//���İ��ϵ�
  LED4,
  LED5,
  LED6,
  LED7,
  LEDOUT,		//ĸ���ϵ�
} LEDn_e;

typedef enum
{
  ON=0,  //��
  OFF=1, //��
  RVS=2, //��ת
}LEDs_e;

extern void LED_Init(void);
extern void LED_Ctrl(LEDn_e ledno, LEDs_e sta);
extern void Test_LED(void);




#endif /* CODE_LED_H_ */
