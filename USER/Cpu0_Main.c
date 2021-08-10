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
 * @Software 		ADS v1.2.2
 * @Target core		TC377TP
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2020-11-23
 ********************************************************************************************************************/

#include "Cpu0_Main.h"
#include "headfile.h"

uint8 cam_flag;
uint8 pro_mode;
uint8 sobel_finish_flag;
uint8 img_finish_flag;
uint8 test1_finish_flag;

extern uint8 img[IMG_H][IMG_W];
extern uint8 sobel[IMG_H][IMG_W];
extern uint8 fixed_thres[IMG_H][IMG_W];

extern IFX_ALIGN(4) uint8 mt9v03x_image[MT9V03X_H][MT9V03X_W];
extern float32 sobel_thsf;
extern float32 img_thsf;

extern ImgData id[2];
extern ImgData *pd_last;
extern ImgData *pd_now;

#pragma section all "cpu0_dsram"
//将本语句与#pragma section all restore语句之间的全局变量都放在CPU0的RAM中

//工程导入到软件之后，应该选中工程然后点击refresh刷新一下之后再编译

//工程默认设置为关闭优化，可以自己右击工程选择properties->C/C++ Build->Setting
//然后在右侧的窗口中找到C/C++ Compiler->Optimization->Optimization level处设置优化等级
//一般默认新建立的工程都会默认开2级优化，因此大家也可以设置为2级优化

//对于TC系列默认是不支持中断嵌套的，希望支持中断嵌套需要在中断内使用enableInterrupts();来开启中断嵌套
//简单点说实际上进入中断后TC系列的硬件自动调用了disableInterrupts();来拒绝响应任何的中断，因为需要我们自己手动调用enableInterrupts();来开启中断的响应。

int core0_main(void)
{
	uint8 mode;
	get_clk(); //获取时钟频率  务必保留
	//用户在此处调用各种初始化函数等

	mt9v03x_init();
	sobel_init((uint8 *)sobel, IMG_W, IMG_H);
	pd_last = id;
	pd_now = id + 1;
	id_init(pd_last, pd_now);

	//等待所有核心初始化完毕
	IfxCpu_emitEvent(&g_cpuSyncEvent);
	IfxCpu_waitEvent(&g_cpuSyncEvent, 0xFFFF);
	enableInterrupts();
	while (TRUE)
	{
		//用户在此处编写任务代码
		if (pro_mode != 0)
		{
			mode = pro_mode;
			pro_mode = 0;
			while (!mt9v03x_finish_flag)
				; //等待图像获取完毕
			img_get((uint8 *)mt9v03x_image, (uint8 *)img, CAM_W, CAM_H);
			switch (mode)
			{
			case SOBEL_MODE:
				sobel_get((uint8 *)img, (uint8 *)sobel, IMG_W, IMG_H);
				binary((uint8 *)sobel, IMG_W, IMG_H, (uint8)(sobel_thsf + 0.5));
				/*binary3((uint8 *)mt9v03x_image, (uint8 *)fixed_thres, IMG_W, IMG_H);*/
				sobel_finish_flag = 1;
				break;
			case IMAGE_MODE:
				sobel_get((uint8 *)img, (uint8 *)sobel, IMG_W, IMG_H);
				binary2((uint8 *)img, (uint8 *)sobel, IMG_W, IMG_H,
						(uint8)(img_thsf + 0.5), (uint8)(sobel_thsf + 0.5));
				/*binary3((uint8 *)mt9v03x_image, (uint8 *)fixed_thres, IMG_W, IMG_H);*/
				img_finish_flag = 1;
				break;
			case TEST1_MODE:
				sobel_get((uint8 *)img, (uint8 *)sobel, IMG_W, IMG_H);
				binary2((uint8 *)img, (uint8 *)sobel, IMG_W, IMG_H,
						(uint8)(img_thsf + 0.5), (uint8)(sobel_thsf + 0.5));
				/*binary3((uint8 *)mt9v03x_image, (uint8 *)fixed_thres, IMG_W, IMG_H);*/
				id_get((uint8 *)sobel, &pd_last, &pd_now);
				//id2img(pd_now,(uint8*)sobel,IMG_W,IMG_H);
				test1_finish_flag = 1;
				break;
			}
			mt9v03x_finish_flag = 0;
		}
		if (cam_flag)
		{
			if (mt9v03x_finish_flag)
			{
				mt9v03x_finish_flag = 0;
				cam_flag = 0;
			}
		}
	}
}

#pragma section all restore
