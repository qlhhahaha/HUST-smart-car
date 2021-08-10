

/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ����Ⱥ��824575535
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ����������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		isr
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ3184284598)
 * @version    		�鿴doc��version�ļ� �汾˵��
 * @Software 		ADS v1.2.2
 * @Target core		TC377TP
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2020-11-23
 ********************************************************************************************************************/

#include "isr_config.h"
#include "isr.h"

uint8 mpid_flag;
uint8 spid_flag;
extern int32 motor_duty;
extern M_PID mpid;
extern int16 enc_spd;

extern uint32 servo_duty;
extern S_PID spid;

//PIT�жϺ���  ʾ��
IFX_INTERRUPT(cc60_pit_ch0_isr, CCU6_0_CH0_INT_VECTAB_NUM, CCU6_0_CH0_ISR_PRIORITY)
{
    enableInterrupts(); //�����ж�Ƕ��
    PIT_CLEAR_FLAG(CCU6_0, PIT_CH0);
    Enc_Int();
    if (mpid_flag)
    {
        Motor_Pid(&motor_duty, enc_spd, &mpid);
        Motor_Duty();
    }
    if (spid_flag)
    {
        Servo_Pid(&servo_duty, &spid);
        Servo_Duty();
    }
}

IFX_INTERRUPT(cc60_pit_ch1_isr, CCU6_0_CH1_INT_VECTAB_NUM, CCU6_0_CH1_ISR_PRIORITY)
{
    enableInterrupts(); //�����ж�Ƕ��
    PIT_CLEAR_FLAG(CCU6_0, PIT_CH1);
    Induc_GetInfo();
}

IFX_INTERRUPT(cc61_pit_ch0_isr, CCU6_1_CH0_INT_VECTAB_NUM, CCU6_1_CH0_ISR_PRIORITY)
{
    enableInterrupts(); //�����ж�Ƕ��
    PIT_CLEAR_FLAG(CCU6_1, PIT_CH0);
}

IFX_INTERRUPT(cc61_pit_ch1_isr, CCU6_1_CH1_INT_VECTAB_NUM, CCU6_1_CH1_ISR_PRIORITY)
{
    enableInterrupts(); //�����ж�Ƕ��
    PIT_CLEAR_FLAG(CCU6_1, PIT_CH1);
}

IFX_INTERRUPT(eru_ch0_ch4_isr, ERU_CH0_CH4_INT_VECTAB_NUM, ERU_CH0_CH4_INT_PRIO)
{
    enableInterrupts();                    //�����ж�Ƕ��
    if (GET_GPIO_FLAG(ERU_CH0_REQ4_P10_7)) //ͨ��0�ж�
    {
        CLEAR_GPIO_FLAG(ERU_CH0_REQ4_P10_7);
    }

    if (GET_GPIO_FLAG(ERU_CH4_REQ13_P15_5)) //ͨ��4�ж�
    {
        CLEAR_GPIO_FLAG(ERU_CH4_REQ13_P15_5);
    }
}

IFX_INTERRUPT(eru_ch1_ch5_isr, ERU_CH1_CH5_INT_VECTAB_NUM, ERU_CH1_CH5_INT_PRIO)
{
    enableInterrupts();                    //�����ж�Ƕ��
    if (GET_GPIO_FLAG(ERU_CH1_REQ5_P10_8)) //ͨ��1�ж�
    {
        CLEAR_GPIO_FLAG(ERU_CH1_REQ5_P10_8);
    }

    if (GET_GPIO_FLAG(ERU_CH5_REQ1_P15_8)) //ͨ��5�ж�
    {
        CLEAR_GPIO_FLAG(ERU_CH5_REQ1_P15_8);
    }
}

//��������ͷpclk����Ĭ��ռ���� 2ͨ�������ڴ���DMA��������ﲻ�ٶ����жϺ���
//IFX_INTERRUPT(eru_ch2_ch6_isr, ERU_CH2_CH6_INT_VECTAB_NUM, ERU_CH2_CH6_INT_PRIO)
//{
//	enableInterrupts();//�����ж�Ƕ��
//	if(GET_GPIO_FLAG(ERU_CH2_REQ7_P00_4))//ͨ��2�ж�
//	{
//		CLEAR_GPIO_FLAG(ERU_CH2_REQ7_P00_4);
//
//	}
//	if(GET_GPIO_FLAG(ERU_CH6_REQ9_P20_0))//ͨ��6�ж�
//	{
//		CLEAR_GPIO_FLAG(ERU_CH6_REQ9_P20_0);
//
//	}
//}

IFX_INTERRUPT(eru_ch3_ch7_isr, ERU_CH3_CH7_INT_VECTAB_NUM, ERU_CH3_CH7_INT_PRIO)
{
    enableInterrupts();                    //�����ж�Ƕ��
    if (GET_GPIO_FLAG(ERU_CH3_REQ6_P02_0)) //ͨ��3�ж�
    {
        CLEAR_GPIO_FLAG(ERU_CH3_REQ6_P02_0);
        if (CAMERA_GRAYSCALE == camera_type)
            mt9v03x_vsync();
    }
    if (GET_GPIO_FLAG(ERU_CH7_REQ16_P15_1)) //ͨ��7�ж�
    {
        CLEAR_GPIO_FLAG(ERU_CH7_REQ16_P15_1);
    }
}

IFX_INTERRUPT(dma_ch5_isr, ERU_DMA_INT_VECTAB_NUM, ERU_DMA_INT_PRIO)
{
    enableInterrupts(); //�����ж�Ƕ��
    if (CAMERA_GRAYSCALE == camera_type)
        mt9v03x_dma();
}

//�����жϺ���  ʾ��
IFX_INTERRUPT(uart0_tx_isr, UART0_INT_VECTAB_NUM, UART0_TX_INT_PRIO)
{
    enableInterrupts(); //�����ж�Ƕ��
    IfxAsclin_Asc_isrTransmit(&uart0_handle);
}
IFX_INTERRUPT(uart0_rx_isr, UART0_INT_VECTAB_NUM, UART0_RX_INT_PRIO)
{
    enableInterrupts(); //�����ж�Ƕ��
    IfxAsclin_Asc_isrReceive(&uart0_handle);
}
IFX_INTERRUPT(uart0_er_isr, UART0_INT_VECTAB_NUM, UART0_ER_INT_PRIO)
{
    enableInterrupts(); //�����ж�Ƕ��
    IfxAsclin_Asc_isrError(&uart0_handle);
}

//����1Ĭ�����ӵ�����ͷ���ô���
IFX_INTERRUPT(uart1_tx_isr, UART1_INT_VECTAB_NUM, UART1_TX_INT_PRIO)
{
    enableInterrupts(); //�����ж�Ƕ��
    IfxAsclin_Asc_isrTransmit(&uart1_handle);
}
IFX_INTERRUPT(uart1_rx_isr, UART1_INT_VECTAB_NUM, UART1_RX_INT_PRIO)
{
    enableInterrupts(); //�����ж�Ƕ��
    IfxAsclin_Asc_isrReceive(&uart1_handle);
    if (CAMERA_GRAYSCALE == camera_type)
        mt9v03x_uart_callback();
}
IFX_INTERRUPT(uart1_er_isr, UART1_INT_VECTAB_NUM, UART1_ER_INT_PRIO)
{
    enableInterrupts(); //�����ж�Ƕ��
    IfxAsclin_Asc_isrError(&uart1_handle);
}

//����2Ĭ�����ӵ�����ת����ģ��
IFX_INTERRUPT(uart2_tx_isr, UART2_INT_VECTAB_NUM, UART2_TX_INT_PRIO)
{
    enableInterrupts(); //�����ж�Ƕ��
    IfxAsclin_Asc_isrTransmit(&uart2_handle);
}
IFX_INTERRUPT(uart2_rx_isr, UART2_INT_VECTAB_NUM, UART2_RX_INT_PRIO)
{
    enableInterrupts(); //�����ж�Ƕ��
    IfxAsclin_Asc_isrReceive(&uart2_handle);
}
IFX_INTERRUPT(uart2_er_isr, UART2_INT_VECTAB_NUM, UART2_ER_INT_PRIO)
{
    enableInterrupts(); //�����ж�Ƕ��
    IfxAsclin_Asc_isrError(&uart2_handle);
}

IFX_INTERRUPT(uart3_tx_isr, UART3_INT_VECTAB_NUM, UART3_TX_INT_PRIO)
{
    enableInterrupts(); //�����ж�Ƕ��
    IfxAsclin_Asc_isrTransmit(&uart3_handle);
}
IFX_INTERRUPT(uart3_rx_isr, UART3_INT_VECTAB_NUM, UART3_RX_INT_PRIO)
{
    enableInterrupts(); //�����ж�Ƕ��
    IfxAsclin_Asc_isrReceive(&uart3_handle);
}
IFX_INTERRUPT(uart3_er_isr, UART3_INT_VECTAB_NUM, UART3_ER_INT_PRIO)
{
    enableInterrupts(); //�����ж�Ƕ��
    IfxAsclin_Asc_isrError(&uart3_handle);
}