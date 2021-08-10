/*
 * Dream-Seekers-adc.h
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

#ifndef USER_ADC_H_
#define USER_ADC_H_

#define Left_ADCModule		ADC_0
#define Left_ADCChannel		ADC0_CH0_A0

#define MiddleL_ADCModule	ADC_0
#define MiddleL_ADCChannel	ADC0_CH1_A1

#define MiddleR_ADCModule	ADC_0
#define MiddleR_ADCChannel	ADC0_CH3_A3

#define Right_ADCModule		ADC_0
#define Right_ADCChannel	ADC0_CH4_A4

void ADC_Init(void);
void TestADC(void);

void Read_ADC(void);
int16 ADC_Calculate(void);
u16 ADC_DetectiveCircle(void);
u16 AbortedRoute(void);

#endif /* USER_ADC_H_ */
