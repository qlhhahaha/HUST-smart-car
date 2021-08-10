/*
 * bat.c
 *
 *  Created on: 2021年5月8日
 *      Author: 朱江禹
 */

#include "bat.h"

int16 bat_value;
float64 bat_voltage;

const int16 value_min=2865;
const int16 value_max=3330;


void Bat_Init(void)
{
    adc_init(BAT_ADC,BAT_PIN);
    Bat_GetInfo();//采集十次电压求平均值，并由adc值转化为具体电压值
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
    //等比例计算，将int型的adc值转化为float型的电压值
    return vol;
}

void Bat_GetInfo(void)
{
    bat_value=adc_mean_filter(BAT_ADC,BAT_PIN,ADC_12BIT,BAT_COUNT);
    bat_voltage=Bat_val2vol(bat_value);
}
