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
//���������#pragma section all restore���֮���ȫ�ֱ���������CPU0��RAM��

//���̵��뵽���֮��Ӧ��ѡ�й���Ȼ����refreshˢ��һ��֮���ٱ���

//����Ĭ������Ϊ�ر��Ż��������Լ��һ�����ѡ��properties->C/C++ Build->Setting
//Ȼ�����Ҳ�Ĵ������ҵ�C/C++ Compiler->Optimization->Optimization level�������Ż��ȼ�
//һ��Ĭ���½����Ĺ��̶���Ĭ�Ͽ�2���Ż�����˴��Ҳ��������Ϊ2���Ż�

//����TCϵ��Ĭ���ǲ�֧���ж�Ƕ�׵ģ�ϣ��֧���ж�Ƕ����Ҫ���ж���ʹ��enableInterrupts();�������ж�Ƕ��
//�򵥵�˵ʵ���Ͻ����жϺ�TCϵ�е�Ӳ���Զ�������disableInterrupts();���ܾ���Ӧ�κε��жϣ���Ϊ��Ҫ�����Լ��ֶ�����enableInterrupts();�������жϵ���Ӧ��

int core0_main(void)
{
	uint8 mode;
	get_clk(); //��ȡʱ��Ƶ��  ��ر���
	//�û��ڴ˴����ø��ֳ�ʼ��������

	mt9v03x_init();
	sobel_init((uint8 *)sobel, IMG_W, IMG_H);
	pd_last = id;
	pd_now = id + 1;
	id_init(pd_last, pd_now);

	//�ȴ����к��ĳ�ʼ�����
	IfxCpu_emitEvent(&g_cpuSyncEvent);
	IfxCpu_waitEvent(&g_cpuSyncEvent, 0xFFFF);
	enableInterrupts();
	while (TRUE)
	{
		//�û��ڴ˴���д�������
		if (pro_mode != 0)
		{
			mode = pro_mode;
			pro_mode = 0;
			while (!mt9v03x_finish_flag)
				; //�ȴ�ͼ���ȡ���
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
