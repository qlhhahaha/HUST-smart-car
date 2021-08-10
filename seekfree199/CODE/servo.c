/*
 * Dream-Seekers-servo.c
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
#include "pid.h"
#include "servo.h"


//���ö���Ĳ���
int16 SERVO_CFG[][2]=
{
    {SERVO_Frequency, 			50},			//���Ƶ������

	//280 370 460
	{SERVO_MidDuty,          	400},	//2100		//�����ֵռ�ձ�      385          //2370
    {SERVO_MinDuty,          	260},	//1000		//�����С����ռ�ձ�(0~100000)     //1000
    {SERVO_MaxDuty,          	580},	//3000		//����������ռ�ձ�(0~100000)     //3100
};

//-------------------------------------------------------------------------------------------------------------------
//  @name    Servo_Init
//  @brief   �����ʼ��
//  @param
//  @return  void
//  @since      v1.0
//  @note
//  Sample usage:  Servo_Init()
//-------------------------------------------------------------------------------------------------------------------
void Servo_Init()
{
	gtm_pwm_init(Servo_Module, SERVO_CFG[SERVO_Frequency][1], SERVO_CFG[SERVO_MidDuty][1], 1);
}

//-------------------------------------------------------------------------------------------------------------------
//  @name    Servo_Control
//  @brief   ���ռ�ձȿ���
//  @param   duty-ռ�ձ� (0-10000)
//  @return  void
//  @since      v1.0
//  @note      
//  Sample usage:  Servo_Control(5000)
//-------------------------------------------------------------------------------------------------------------------
void Servo_Control(uint32 duty)
{
	duty = RANGE16(duty, SERVO_CFG[SERVO_MinDuty][1], SERVO_CFG[SERVO_MaxDuty][1]);
	pwm_duty(Servo_Module, duty, 1);///ϵ��1���ƶ��PWM,����Ϊ���Ƶ��PWM
}


//-------------------------------------------------------------------------------------------------------------------
//  @name    TestServo
//  @brief   �������
//  @param   x-��Ч����
//  @return  void
//  @since      v1.0
//  @note
//  Sample usage:  Servo_Init(50, 0)
//-------------------------------------------------------------------------------------------------------------------
void TestServo(void)
{
	int num=SERVO_CFG[SERVO_MinDuty][1];
	char show[20];
	Servo_Init();
	ips200_showstr(10,2,"Servo-Duty : 0");
	while(1)
	{
		if(GET_KEYCODE()==MY_KEY_UP)
		{
			num+=10;
			num = RANGE16(num, SERVO_CFG[SERVO_MinDuty][1], SERVO_CFG[SERVO_MaxDuty][1]);
			sprintf(show,"Servo-Duty : %5d",num);
			ips200_showstr(10,2,show);
			Servo_Control(num);
		}
		else if(GET_KEYCODE()==MY_KEY_DOWN)
		{
			num-=10;
			num = RANGE16(num, SERVO_CFG[SERVO_MinDuty][1], SERVO_CFG[SERVO_MaxDuty][1]);
			sprintf(show,"Servo-Duty : %5d",num);
			ips200_showstr(10,2,show);
			Servo_Control(num);
		}
		else if(GET_KEYCODE()==MY_KEY_CANCLE)
		{
			Servo_Control(SERVO_CFG[SERVO_MidDuty][1]);
			break;
		}
	}
}

int16 ServoPWMAdd = 0;
int16 ServoPWM = 0;
float ServoDiff = 0;


/**
 * @file		ת���������˲�
 * @note
 * @author
 * @date
 */
int16 Turn_Out_Filter(int16 turn_out)    //
{
  int Turn_Out_Filtered;
  static int Pre1_Error[4];
  Pre1_Error[3]=Pre1_Error[2];
  Pre1_Error[2]=Pre1_Error[1];
  Pre1_Error[1]=Pre1_Error[0];
  Pre1_Error[0]=turn_out;
  Turn_Out_Filtered=Pre1_Error[0]*0.4+Pre1_Error[1]*0.3+Pre1_Error[2]*0.2+Pre1_Error[3]*0.1;
  															//0.5								0.2								 0.2							 0.1
  return Turn_Out_Filtered;
}

void ServoOut(void)
{
	ServoPWM = SERVO_CFG[SERVO_MidDuty][1] + ServoPWMAdd;
	ServoPWM = Turn_Out_Filter(ServoPWM);                               										//�����������˲�
  	ServoPWM = RANGE16(ServoPWM,SERVO_CFG[SERVO_MinDuty][1],SERVO_CFG[SERVO_MaxDuty][1]);						//�������޷�
	if(ServoPWM < SERVO_CFG[SERVO_MidDuty][1])///<
	{
		ServoDiff = 45.0*(float)(SERVO_CFG[SERVO_MidDuty][1] - ServoPWM)/(SERVO_CFG[SERVO_MidDuty][1] - SERVO_CFG[SERVO_MinDuty][1]);
		ServoDiff = RANGEfloat(ServoDiff,0.0,45.0);
	}
	else
	{
		ServoDiff = 45.0*(float)(SERVO_CFG[SERVO_MidDuty][1] - ServoPWM)/(SERVO_CFG[SERVO_MaxDuty][1] - SERVO_CFG[SERVO_MidDuty][1]);
		ServoDiff = RANGEfloat(ServoDiff,-45.0,0.0);
	}
	Servo_Control(6300*2-ServoPWM);///x=6300*2-y
	Servo_Control(SERVO_CFG[SERVO_MidDuty][1]*2-ServoPWM);
}


void ServoControl(float CalcCenter)
{
	ServoPWMAdd = PID_Realize(&ServoPID,ColumnMax*10/2,(int)(CalcCenter*10));		//����ȡһλС��
	ServoOut();
#if UploadMode==2
  	Upload_Signal(CalcCenter,LCDW/2,0,0);
#endif
}

extern float ADCDirectionError;
void ADC_ServoControl(void)
{
	static float ADCDirectionError_temp[5];
	float DirectionError_dot;
	ADCDirectionError_temp[4] = ADCDirectionError_temp[3];
	ADCDirectionError_temp[3] = ADCDirectionError_temp[2];
	ADCDirectionError_temp[2] = ADCDirectionError_temp[1];
	ADCDirectionError_temp[1] = ADCDirectionError_temp[0];
	ADCDirectionError_temp[0] = ADCDirectionError;
	DirectionError_dot = 150*(ADCDirectionError_temp[0]-ADCDirectionError_temp[3]);//ƫ��΢��
	DirectionError_dot = (DirectionError_dot>65?65:DirectionError_dot);//΢���޷�
	DirectionError_dot = (DirectionError_dot<-65?-65:DirectionError_dot);
	///ServoPWMAdd = ADCDirectionError*MagneticPID.Proportion + DirectionError_dot*(MagneticPID.Derivative/100);//PDת��
	ServoOut();
}

