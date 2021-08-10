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
//  @brief   	ADC初始化
//  @param  	void
//  @return		void
//  @since      v1.0
//  @note
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void ADC_Init(void)
{
	adc_init(Left_ADCModule, Left_ADCChannel);		//初始化通道和引脚
	adc_init(MiddleL_ADCModule, MiddleL_ADCChannel);	//初始化通道和引脚
	adc_init(MiddleR_ADCModule, MiddleR_ADCChannel);	//初始化通道和引脚
	adc_init(Right_ADCModule, Right_ADCChannel);	//初始化通道和引脚
}

//-------------------------------------------------------------------------------------------------------------------
//  @name    	TestADC
//  @brief   	ADC采样检测
//  @param		x-无效参数
//  @return
//  @since      v1.0
//  @note
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void TestADC(void)
{
	ADC_Init();										//初始化ADC
	while(TRUE)
	{
		Read_ADC();
		LeftResult = adc_mean_filter(Left_ADCModule, Left_ADCChannel, ADC_12BIT, 10);			//采集10次求平均  分辨率12位
		MiddleLResult = adc_mean_filter(MiddleL_ADCModule, MiddleL_ADCChannel, ADC_12BIT, 10);		//采集10次求平均  分辨率12位
		MiddleRResult = adc_mean_filter(MiddleR_ADCModule, MiddleR_ADCChannel, ADC_12BIT, 10);		//采集10次求平均  分辨率12位
		RightResult = adc_mean_filter(Right_ADCModule, Right_ADCChannel, ADC_12BIT, 10);		//采集10次求平均  分辨率12位
		delay_ms(20);

		char dat[30];
		sprintf(dat,"Left: %4d", NormalizeOfAD[0]);//左//归一化后得到电感值
		ips200_showstr(10,2,dat);

		sprintf(dat,"MiddleL: %4d",NormalizeOfAD[1]);//中左
		ips200_showstr(10,4,dat);

		sprintf(dat,"MiddleL: %4d",NormalizeOfAD[2]);//中右
		ips200_showstr(10,6,dat);

		sprintf(dat,"Right: %4d",NormalizeOfAD[3]);//右
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
	   	ad_valu[0][i] = adc_mean_filter(Left_ADCModule, Left_ADCChannel, ADC_12BIT, 10);   //左电感
	   	ad_valu[1][i] = adc_mean_filter(MiddleL_ADCModule, MiddleL_ADCChannel, ADC_12BIT, 10);
	   	ad_valu[2][i] = adc_mean_filter(MiddleR_ADCModule, MiddleR_ADCChannel, ADC_12BIT, 10);
		ad_valu[3][i] = adc_mean_filter(Right_ADCModule, Right_ADCChannel, ADC_12BIT, 10);  //右电感
     }
/*=========================冒泡排序升序==========================*/
     for(i=0;i<INDNUM;i++)
     {
        for(j=0;j<4;j++)
        {
           for(k=0;k<4-j;k++)
           {
              if(ad_valu[i][k] > ad_valu[i][k+1])        //前面的比后面的大  则进行交换
              {
                 temp = ad_valu[i][k+1];
                 ad_valu[i][k+1] = ad_valu[i][k];
                 ad_valu[i][k] = temp;
              }
           }
        }
     }
/*===========================中值滤波=================================*/
     for(i=0;i<INDNUM;i++)    //求中间三项的和
     {
        ad_sum[i] = ad_valu[i][1] + ad_valu[i][2] + ad_valu[i][3];
        ad_valu1[i] = ad_sum[i] / 3;
     }
/*============================滑动平均滤波============================= */

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
	   	ValueOfAD[i] = (int16)(ad_valu1_temp[i][0]/10);  //左电感
		ValueOfAD[i] = (int16)(ValueOfAD[i]*10);
	 }
	//电磁信号归一化
	 for(i=0;i<INDNUM;i++)
		 NormalizeOfAD[i] = 100*(ValueOfAD[i] - AD_val_min)/(AD_val_max-AD_val_min);
}

float ADCDirectionError;
int16 ADC_Calculate(void)
{
	ADCDirectionError = (float)(NormalizeOfAD[0] - NormalizeOfAD[1])/(NormalizeOfAD[0] + NormalizeOfAD[1]);//差比和
	ADCDirectionError = RANGEfloat(ADCDirectionError,-1,1);													//差比和限幅
	ADCDirectionError = ADCDirectionError * 100;															//放大为百分比
	ADCDirectionError = ADCDirectionError * (ADCDirectionError*ADCDirectionError/1250.0+2)/10;				//牛顿迭代公式
}


/**
 * @file		电感检测环岛
 * @note      	左右电感和强度+中间电感特征值判断
 * @author
 * @date		2019
 */
u16 ADC_DetectiveCircle(void)
{
    static int16 check[4][5],finish[4];
    int i=0;
/*****************************矩阵式递推滤波**********************************/
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

/*********************************环岛判断**********************************/
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
