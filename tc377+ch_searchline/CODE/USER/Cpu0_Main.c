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

//CPU0���ڴ�������ͷ��ͼ���㷨

#include "Cpu0_Main.h"
#include "headfile.h"

uint8 cam_flag;
uint8 pro_flag;
extern int16 ServoPWMAdd;
extern int Test;
#pragma section all "cpu0_dsram"
//���������#pragma section all restore���֮���ȫ�ֱ���������CPU0��RAM��


//���̵��뵽���֮��Ӧ��ѡ�й���Ȼ����refreshˢ��һ��֮���ٱ���

//����Ĭ������Ϊ�ر��Ż��������Լ��һ�����ѡ��properties->C/C++ Build->Setting
//Ȼ�����Ҳ�Ĵ������ҵ�C/C++ Compiler->Optimization->Optimization level�������Ż��ȼ�
//һ��Ĭ���½����Ĺ��� ����Ĭ�Ͽ�2���Ż�����˴��Ҳ��������Ϊ2���Ż�

//����TCϵ��Ĭ���ǲ�֧���ж�Ƕ�׵ģ�ϣ��֧���ж�Ƕ����Ҫ���ж���ʹ��enableInterrupts();�������ж�Ƕ��
//�򵥵�˵ʵ���Ͻ����жϺ�TCϵ�е�Ӳ���Զ�������disableInterrupts();���ܾ���Ӧ�κε��жϣ���Ϊ��Ҫ�����Լ��ֶ�����enableInterrupts();�������жϵ���Ӧ��

int core0_main(void)
{
	get_clk();//��ȡʱ��Ƶ��  ��ر���
	//�û��ڴ˴����ø��ֳ�ʼ��������

	mt9v03x_init();

	//eru_init(ERU_CH0_REQ4_P10_7,RISING);

    //�ȴ����к��ĳ�ʼ�����
	IfxCpu_emitEvent(&g_cpuSyncEvent);
	IfxCpu_waitEvent(&g_cpuSyncEvent, 0xFFFF);
	enableInterrupts();
	while (TRUE)
	{
		//�û��ڴ˴���д�������
	   if(pro_flag)
	    {
	        cam_flag=1;
	        while(!mt9v03x_finish_flag);//����ͷδ��ɳ�ʼ��ʱ���ѭ��
	        mt9v03x_finish_flag=0;
	    }
		
	    if(cam_flag)
	    {
	        if(mt9v03x_finish_flag)
	        {
	            mt9v03x_finish_flag=0;
	            cam_flag=0;
	        }
	    }
		      //DMAOnTimeHandle();
	}
}

#pragma section all restore
