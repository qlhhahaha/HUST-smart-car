/*
 * key.h
 *
 *  Created on: 2021年5月7日
 *      Author: 朱江禹
 */

#ifndef _KEY_H_
#define _KEY_H_

#include "common.h"
#include "zf_gpio.h"

#define KEY_NUM 4
#define KEY0_PIN P00_12
#define KEY1_PIN P00_6
#define KEY2_PIN P00_9
#define KEY3_PIN P00_5      //独立按键对应引脚

typedef struct
{
    uint8 key_last[KEY_NUM];
    uint8 key_now[KEY_NUM];
}KeyInfo;

void Key_Init(void);
void Key_Scan(KeyInfo* pki);

#endif
