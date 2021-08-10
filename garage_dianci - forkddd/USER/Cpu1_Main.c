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

#include "headfile.h"

extern MenuInfo mi;
extern uint8 cam_flag;
extern uint8 pro_mode;
extern uint8 sobel_finish_flag;
extern uint8 img_finish_flag;
extern uint8 test1_finish_flag;

extern uint8 mpid_flag;
extern uint8 spid_flag;

#pragma section all "cpu1_dsram"
//���������#pragma section all restore���֮���ȫ�ֱ���������CPU1��RAM��

void core1_main(void)
{
    disableInterrupts();
    IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());
    //�û��ڴ˴����ø��ֳ�ʼ��������

    cam_flag = 0;
    pro_mode = 0;
    sobel_finish_flag = 0;
    img_finish_flag = 0;
    test1_finish_flag = 0;

    mpid_flag = 0;
    spid_flag = 0;

    ips200_init();
    Key_Init();
    LED_Init();
    Bat_Init();

    Menu_Init();

    //�ȴ����к��ĳ�ʼ�����
    IfxCpu_emitEvent(&g_cpuSyncEvent);
    IfxCpu_waitEvent(&g_cpuSyncEvent, 0xFFFF);
    enableInterrupts();
    while (TRUE)
    {
        //�û��ڴ˴���д�������
        Menu_Handle(&mi);
    }
}

#pragma section all restore
