/*
 * Dream-Seekers-adc.c
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
#include "adc.h"
uint16 LeftResult;
uint16 MiddleLResult;
uint16 MiddleRResult;
uint16 RightResult;

uint16 AD_val_min = 0;
uint16 AD_val_max = 4095;
uint16 ValueOfAD[4];
uint16 NormalizeOfAD[4];
//-------------------------------------------------------------------------------------------------------------------
//  @name    	ADC_Init
//  @brief   	ADC��ʼ��
//  @param  	void
//  @return		void
//  @since      v1.0
//  @note
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void ADC_Init(void)
{
	adc_init(Left_ADCModule, Left_ADCChannel);		//��ʼ��ͨ��������
	adc_init(MiddleL_ADCModule, MiddleL_ADCChannel);	//��ʼ��ͨ��������
	adc_init(MiddleR_ADCModule, MiddleR_ADCChannel);	//��ʼ��ͨ��������
	adc_init(Right_ADCModule, Right_ADCChannel);	//��ʼ��ͨ��������
}

//-------------------------------------------------------------------------------------------------------------------
//  @name    	TestADC
//  @brief   	ADC�������
//  @param		x-��Ч����
//  @return
//  @since      v1.0
//  @note
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void TestADC(void)
{
	ADC_Init();										//��ʼ��ADC
	while(TRUE)
	{
		Read_ADC();
		LeftResult = adc_mean_filter(Left_ADCModule, Left_ADCChannel, ADC_12BIT, 10);			//�ɼ�10����ƽ��  �ֱ���12λ
		MiddleLResult = adc_mean_filter(MiddleL_ADCModule, MiddleL_ADCChannel, ADC_12BIT, 10);		//�ɼ�10����ƽ��  �ֱ���12λ
		MiddleRResult = adc_mean_filter(MiddleR_ADCModule, MiddleR_ADCChannel, ADC_12BIT, 10);		//�ɼ�10����ƽ��  �ֱ���12λ
		RightResult = adc_mean_filter(Right_ADCModule, Right_ADCChannel, ADC_12BIT, 10);		//�ɼ�10����ƽ��  �ֱ���12λ
		delay_ms(20);

		char dat[30];
		sprintf(dat,"Left: %4d", NormalizeOfAD[0]);//��//��һ����õ����ֵ
		ips200_showstr(10,2,dat);

		sprintf(dat,"MiddleL: %4d",NormalizeOfAD[1]);//����
		ips200_showstr(10,4,dat);

		sprintf(dat,"MiddleL: %4d",NormalizeOfAD[2]);//����
		ips200_showstr(10,6,dat);

		sprintf(dat,"Right: %4d",NormalizeOfAD[3]);//��
		ips200_showstr(10,8,dat);

		if(GET_KEYCODE()==MY_KEY_CANCLE)
			break;
	}
}


#define INDNUM	4
void Read_ADC(void)
{
     int16  i,j,k,temp;
     int16  ad_valu[INDNUM][5],ad_valu1[INDNUM],ad_sum[INDNUM];
	 static int16 ad_valu1_temp[INDNUM][5] = {0};

     for(i=0;i<5;i++)
     {
	   	ad_valu[0][i] = adc_mean_filter(Left_ADCModule, Left_ADCChannel, ADC_12BIT, 10);   //����
	   	ad_valu[1][i] = adc_mean_filter(MiddleL_ADCModule, MiddleL_ADCChannel, ADC_12BIT, 10);
	   	ad_valu[2][i] = adc_mean_filter(MiddleR_ADCModule, MiddleR_ADCChannel, ADC_12BIT, 10);
		ad_valu[3][i] = adc_mean_filter(Right_ADCModule, Right_ADCChannel, ADC_12BIT, 10);  //�ҵ��
     }
/*=========================ð����������==========================*/
     for(i=0;i<INDNUM;i++)
     {
        for(j=0;j<4;j++)
        {
           for(k=0;k<4-j;k++)
           {
              if(ad_valu[i][k] > ad_valu[i][k+1])        //ǰ��ıȺ���Ĵ�  ����н���
              {
                 temp = ad_valu[i][k+1];
                 ad_valu[i][k+1] = ad_valu[i][k];
                 ad_valu[i][k] = temp;
              }
           }
        }
     }
/*===========================��ֵ�˲�=================================*/
     for(i=0;i<INDNUM;i++)    //���м�����ĺ�
     {
        ad_sum[i] = ad_valu[i][1] + ad_valu[i][2] + ad_valu[i][3];
        ad_valu1[i] = ad_sum[i] / 3;
     }
/*============================����ƽ���˲�============================= */

	 for(i=0;i<INDNUM;i++)
	 {
	 	ad_valu1_temp[i][4] = ad_valu1_temp[i][3];
	 	ad_valu1_temp[i][3] = ad_valu1_temp[i][2];
	 	ad_valu1_temp[i][2] = ad_valu1_temp[i][1];
	 	ad_valu1_temp[i][1] = ad_valu1_temp[i][0];
	 	ad_valu1_temp[i][0] = ad_valu1[i];
	 }

	 for(i=0;i<INDNUM;i++)
	 {
	   	ValueOfAD[i] = (int16)(ad_valu1_temp[i][0]/10);  //����
		ValueOfAD[i] = (int16)(ValueOfAD[i]*10);
	 }
	//����źŹ�һ��
	 for(i=0;i<INDNUM;i++)
		 NormalizeOfAD[i] = 100*(ValueOfAD[i] - AD_val_min)/(AD_val_max-AD_val_min);
}

float ADCDirectionError;
int16 ADC_Calculate(void)
{
	ADCDirectionError = (float)(NormalizeOfAD[0] - NormalizeOfAD[1])/(NormalizeOfAD[0] + NormalizeOfAD[1]);//��Ⱥ�
	ADCDirectionError = RANGEfloat(ADCDirectionError,-1,1);													//��Ⱥ��޷�
	ADCDirectionError = ADCDirectionError * 100;															//�Ŵ�Ϊ�ٷֱ�
	ADCDirectionError = ADCDirectionError * (ADCDirectionError*ADCDirectionError/1250.0+2)/10;				//ţ�ٵ�����ʽ
}


/**
 * @file		��м�⻷��
 * @note      	���ҵ�к�ǿ��+�м�������ֵ�ж�
 * @author
 * @date		2019
 */
u16 ADC_DetectiveCircle(void)
{
    static int16 check[4][5],finish[4];
    int i=0;
/*****************************����ʽ�����˲�**********************************/
    for(i=0;i<4;i++)
    {
      check[0][i] = check[0][i+1];
      check[1][i] = check[1][i+1];
      check[2][i] = check[2][i+1];
      check[3][i] = check[3][i+1];
    }
	check[0][4] = NormalizeOfAD[0];
	check[1][4] = NormalizeOfAD[1];
  	check[2][4] = NormalizeOfAD[2];
  	check[3][4] = NormalizeOfAD[3];
    for(i=0;i<4;i++)
      finish[i]=(check[i][0]+check[i][1]+check[i][2]+check[i][3]+check[i][4])/5;

/*********************************�����ж�**********************************/
//	if(	(finish[0] + finish[3] >140) || (finish[0]>95 && finish[3] > 10) || (finish[0]>10 && finish[3] > 95) )

	if(finish[0] + finish[3] >80)
	{
		if(LeftCircleFlag == NoCircle && RightCircleFlag == NoCircle)
		{
			LeftCircleFlag = ShouldBeCircle;
			RightCircleFlag = ShouldBeCircle;
			SpeedParm = GearCtl.Stop;
			for(i=0;i<4;i++)
			{
				check[0][i] = 0;
				check[1][i] = 0;
				check[2][i] = 0;
				check[3][i] = 0;
			}
		}
	}

}
//-------------------------------------------------------------------------------------------------------------------
//  @name
//  @brief
//  @param
//  @return
//  @since      v1.0
//  @note
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------

u16 AbortedRoute(void)
{
	if(NormalizeOfAD[0]<5 && NormalizeOfAD[3]<5)
	  	return 1;
	else
	  	return 0;
}
