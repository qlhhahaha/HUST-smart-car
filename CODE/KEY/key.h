/*
 * key.h
 *
 *  Created on: 2021年5月7日
 *       
 */

#ifndef _KEY_H_
#define _KEY_H_

#include "common.h"
#include "zf_gpio.h"

#define KEY_NUM 4
#define KEY0_PIN P33_4
#define KEY1_PIN P33_5
#define KEY2_PIN P33_6
#define KEY3_PIN P33_7 //独立按键对应引脚

typedef struct
{
    uint8 key_last[KEY_NUM];
    uint8 key_now[KEY_NUM];
} KeyInfo;

void Key_Init(void);
void Key_Scan(KeyInfo *pki);

#endif
