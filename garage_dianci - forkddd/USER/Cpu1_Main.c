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
//将本语句与#pragma section all restore语句之间的全局变量都放在CPU1的RAM中

void core1_main(void)
{
    disableInterrupts();
    IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());
    //用户在此处调用各种初始化函数等

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

    //等待所有核心初始化完毕
    IfxCpu_emitEvent(&g_cpuSyncEvent);
    IfxCpu_waitEvent(&g_cpuSyncEvent, 0xFFFF);
    enableInterrupts();
    while (TRUE)
    {
        //用户在此处编写任务代码
        Menu_Handle(&mi);
    }
}

#pragma section all restore
