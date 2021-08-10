/*
 * Dream-Seekers-encoder.c
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
#include "encoder.h"
int16 LeftWheelSpeed=0;
int16 RightWheelSpeed=0;
//30:68 ������	21.275cm-21.25cm
///50:105������

//-------------------------------------------------------------------------------------------------------------------
//  @name    	EncoderInit
//  @brief  	��������ʼ��
//  @param  	frq:��������������  (Ms)
//  @return     void
//  @since      v1.0
//  @note
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void Encoder_Init()
{
	LeftEncoderInit();
	RightEncoderInit();
}

//-------------------------------------------------------------------------------------------------------------------
//  @name    	EncoderCount
//  @brief   	��������������ʱ�ж��ڵ��ã�
//  @param
//  @return     				
//  @since      v1.0
//  @note      
//  Sample usage:               
//-------------------------------------------------------------------------------------------------------------------
void Encoder_Count(void)
{
	int16 LastLSpeed = LeftWheelSpeed;
	int16 LastRSpeed = RightWheelSpeed;
	LeftWheelSpeed = gpt12_get(LeftEncoderTimer);///���ֵ��ٶ�
	RightWheelSpeed = gpt12_get(RightEncoderTimer);
	ClearLeftEncoderCount();
    ClearRightEncoderCount();

    LeftWheelSpeed = RANGE16(LeftWheelSpeed,-800, 800);	//����ȡ����/// RANGE16(LeftWheelSpeed,-800, 500)
    LeftWheelSpeed = -LeftWheelSpeed;
    RightWheelSpeed = RANGE16(RightWheelSpeed,-500, 800);	//����ȡ����
   RightWheelSpeed = -RANGE16(RightWheelSpeed,-500, 200);	//����ȡ����

}

//-------------------------------------------------------------------------------------------------------------------
//  @name	TestEncoder
//  @brief	����������
//  @param
//  @return
//  @since      v1.0
//  @note
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void TestEncoder(void)
{
	char show[30];
	Encoder_Init();
	Motor_Init();
	EncoderTimerInit(10);
	while(TRUE)
	{
		sprintf(show,"Left : %4d",LeftWheelSpeed);
		ips200_showstr(10,2,show);
		sprintf(show,"Right: %4d",RightWheelSpeed);
	    ips200_showstr(10,4,show);
		Motor_Control(3000);//,2000);
		if(GET_KEYCODE()==MY_KEY_CANCLE)
			break;
	}
}

