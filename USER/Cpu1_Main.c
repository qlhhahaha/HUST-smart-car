/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：三群：824575535
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		main
 * @company	   		成都逐飞科技有限公司
 * @author     		逐飞科技(QQ3184284598)
 * @version    		查看doc内version文件 版本说明
 * @Software 		tasking v6.3r1
 * @Target core		TC264D
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2020-3-23
 ********************************************************************************************************************/

//cpu1的作用：

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

extern int16  left_right_lost_i;//左右全丢结束行取值自LastLoseAllLine
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
			ShowZoomImage(&mt9v03x_image[0], 188, 120, 94, 60);//左上角画面显示函数
			Draw_Road();///将二值化的图像以及中线、前瞻显示出来
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
				{//增大比例系数P
					num1+=0.01;
					ServoPID.Proportion=num1;
					sprintf(show,"KP : %3f",num1);
					ips200_showstr(10,7,show);

				}
				else if(flag==2)
				{//增大微分系数D
					num3+=0.1;
					ServoPID.Derivative=num3;
					sprintf(show,"KD : %3f",num3);
					ips200_showstr(10,9,show);
				}
			}
			
			if(GET_KEYCODE()==MY_KEY_DOWN)
			{
				if(flag==1)
				{//减小比例系数P
					num1-=0.01;
					ServoPID.Proportion=num1;
					sprintf(show,"KP : %3f",num1);
					ips200_showstr(10,7,show);

				}
				else if(flag==2)
				{//减小微分系数D
					num3-=0.1;
					ServoPID.Derivative=num3;
					sprintf(show,"KD : %3f",num3);
					ips200_showstr(10,9,show);
				}
			}

			if(GET_KEYCODE()==MY_KEY_ENTER)
			{//在比例系数和微分系数之间切换
				if(flag==1)	flag=2;
				else if(flag==2)	flag=1;
			}

			if(GET_KEYCODE()==MY_KEY_CANCLE)
			{//开始进入控制
				Motor_Control(0);
				break;
			}
		}

		//用户在此处编写任务代码
    }

}
