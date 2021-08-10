/*
 * bat.c
 *
 *  Created on: 2021��5��8��
 *      Author: �콭��
 */

#include "bat.h"

int16 bat_value;
float64 bat_voltage;

const int16 value_min=2865;
const int16 value_max=3330;


void Bat_Init(void)
{
    adc_init(BAT_ADC,BAT_PIN);
    Bat_GetInfo();//�ɼ�ʮ�ε�ѹ��ƽ��ֵ������adcֵת��Ϊ�����ѹֵ
}

float64 Bat_val2vol(int16 val)
{
    float64 vol,f1,f2;
    f1=(float64)(val-value_min);
    f2=(float64)(value_max-value_min);
    f1/=f2;
    f2=BAT_MAXVOL-BAT_MINVOL;
    f1*=f2;
    vol=BAT_MINVOL+f1;
    //�ȱ������㣬��int�͵�adcֵת��Ϊfloat�͵ĵ�ѹֵ
    return vol;
}

void Bat_GetInfo(void)
{
    bat_value=adc_mean_filter(BAT_ADC,BAT_PIN,ADC_12BIT,BAT_COUNT);
    bat_voltage=Bat_val2vol(bat_value);
}
