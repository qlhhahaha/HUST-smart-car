/*
 * Dream-Seekers-Encoder.h
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

#ifndef CODE_ENCODER_H_
#define CODE_ENCODER_H_

//���������������
#define LeftEncoderTimer		GPT12_T3
#define LeftEncoderINT			GPT12_T3INA_P02_6
#define LeftEncoderDIR			GPT12_T3EUDA_P02_7

#define RightEncoderTimer       GPT12_T6
#define RightEncoderINT         GPT12_T6INA_P20_3
#define RightEncoderDIR         GPT12_T6EUDA_P20_0//�ұ���������ʱʹ�ã�



//�û��ڴ˴����ø��ֳ�ʼ��������
//��һ������ ��ʾѡ��ʹ�õĶ�ʱ��
//�ڶ������� ��ʾѡ��ļ�������    ���������뷽�����Ų��ɽ���
//���������� ��ʾѡ��ķ�������
#define LeftEncoderInit()			  	gpt12_init(LeftEncoderTimer, LeftEncoderINT, LeftEncoderDIR)
#define RightEncoderInit()			  	gpt12_init(RightEncoderTimer, RightEncoderINT, RightEncoderDIR)
#define EncoderTimerInit(Frequent)		pit_interrupt_ms(CCU6_0, PIT_CH0, Frequent)

#define GetLeftEncoderCount()			gpt12_get(LeftEncoderTimer)
#define GetrightEncoderCount()			gpt12_get(RightEncoderTimer)

#define ClearLeftEncoderCount()			gpt12_clear(LeftEncoderTimer)
#define ClearRightEncoderCount()		gpt12_clear(RightEncoderTimer)


void Encoder_Init();
void Encoder_Count(void);
void TestEncoder(void);

#endif /* CODE_ENCODER_H_ */
