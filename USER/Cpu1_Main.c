/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ����Ⱥ��824575535
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		main
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ3184284598)
 * @version    		�鿴doc��version�ļ� �汾˵��
 * @Software 		tasking v6.3r1
 * @Target core		TC264D
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2020-3-23
 ********************************************************************************************************************/

//cpu1�����ã�

#include "headfile.h"


extern uint8 wifi_buff[];

//extern float LSpeedSet;//60;//2.5M/S
extern float RSpeedSet;
extern int16 LeftWheelSpeed;//60;//2.5M/S
//extern int16 RightWheelSpeed;
//extern int16 LeftMotorPWM;
extern int16 RightMotorPWM;

extern int Test;
extern int16 spurroadtriangle_i;

extern int16 spurroadtriangle_j;

extern int16  left_right_lost_i;//����ȫ��������ȡֵ��LastLoseAllLine
void core1_main(void)
{
	get_clk();
   // IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());

	float num1=ServoPID.Proportion, num2=ServoPID.Integral, num3=ServoPID.Derivative;
	int8 flag=1;
	char show[20];

    while (TRUE)
    {
		if(mt9v03x_finish_flag)
		{
			ShowZoomImage(&mt9v03x_image[0], 188, 120, 94, 60);//���Ͻǻ�����ʾ����
			Draw_Road();///����ֵ����ͼ���Լ����ߡ�ǰհ��ʾ����
			DrawLine1((ColumnMax-2)/2,Test,0,0);
			sprintf((char*)show,"%4.2f",RSpeedSet);
			ips200_showstr(140,6,show);
			sprintf((char*)show,"%4d, %4d",LeftWheelSpeed, RightMotorPWM);
			ips200_showstr(140,7,show);
		}

		if(SpeedParm.MaxSpeed)
		{
			if(GET_KEYCODE() == MY_KEY_UP)
			{
				if(flag==1)
				{//�������ϵ��P
					num1+=0.01;
					ServoPID.Proportion=num1;
					sprintf(show,"KP : %3f",num1);
					ips200_showstr(10,7,show);

				}
				else if(flag==2)
				{//����΢��ϵ��D
					num3+=0.1;
					ServoPID.Derivative=num3;
					sprintf(show,"KD : %3f",num3);
					ips200_showstr(10,9,show);
				}
			}
			
			if(GET_KEYCODE()==MY_KEY_DOWN)
			{
				if(flag==1)
				{//��С����ϵ��P
					num1-=0.01;
					ServoPID.Proportion=num1;
					sprintf(show,"KP : %3f",num1);
					ips200_showstr(10,7,show);

				}
				else if(flag==2)
				{//��С΢��ϵ��D
					num3-=0.1;
					ServoPID.Derivative=num3;
					sprintf(show,"KD : %3f",num3);
					ips200_showstr(10,9,show);
				}
			}

			if(GET_KEYCODE()==MY_KEY_ENTER)
			{//�ڱ���ϵ����΢��ϵ��֮���л�
				if(flag==1)	flag=2;
				else if(flag==2)	flag=1;
			}

			if(GET_KEYCODE()==MY_KEY_CANCLE)
			{//��ʼ�������
				Motor_Control(0);
				break;
			}
		}

		//�û��ڴ˴���д�������
    }

}
