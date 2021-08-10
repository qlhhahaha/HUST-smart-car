/*
 * key.c
 *
 *  Created on: 2021年5月7日
 *      Author: 朱江禹
 */

#include "key.h"

KeyInfo ki=
{
    {0,0,0,0},
    {0,0,0,0}
};

void Key_Init(void)
{
    gpio_init(KEY0_PIN,GPI,0,NO_PULL);
    gpio_init(KEY1_PIN,GPI,0,NO_PULL);
    gpio_init(KEY2_PIN,GPI,0,NO_PULL);
    gpio_init(KEY3_PIN,GPI,0,NO_PULL);
}

void Key_Scan(KeyInfo* pki)
{
    int i;
    for(i=0;i<KEY_NUM;i++)
        pki->key_last[i]=pki->key_now[i];
    pki->key_now[0]=gpio_get(KEY0_PIN);
    pki->key_now[1]=gpio_get(KEY1_PIN);
    pki->key_now[2]=gpio_get(KEY2_PIN);
    pki->key_now[3]=gpio_get(KEY3_PIN);
}
