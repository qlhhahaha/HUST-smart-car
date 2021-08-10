/*
 * bat.h
 *
 *  Created on: 2021年5月8日
 *      Author: 朱江禹
 */

#ifndef _BAT_H_
#define _BAT_H_

#include "common.h"
#include "zf_vadc.h"

#define BAT_ADC ADC_0
#define BAT_PIN ADC0_CH1_A1
#define BAT_COUNT 10
#define BAT_MAXVOL 8.4
#define BAT_MINVOL 7.2

void Bat_Init(void);
float64 Bat_val2vol(int16 val);
void Bat_GetInfo(void);

#endif
