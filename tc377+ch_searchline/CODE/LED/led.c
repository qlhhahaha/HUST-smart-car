/*
 * led.c
 *
 *  Created on: 2021年5月8日
 *      Author: 朱江禹
 */

#include "led.h"

void LED_Init(void)
{
    gpio_init(LED0_PIN,GPO,0,PUSHPULL);
    gpio_init(LED1_PIN,GPO,0,PUSHPULL);
    gpio_init(LED2_PIN,GPO,0,PUSHPULL);
    gpio_init(LED3_PIN,GPO,0,PUSHPULL);
}
