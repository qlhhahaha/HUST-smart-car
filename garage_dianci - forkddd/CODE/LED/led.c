/*
 * led.c
 *
 *  Created on: 2021��5��8��
 *      Author: �콭��
 */

#include "led.h"

void LED_Init(void)
{
    gpio_init(LED0_PIN,GPO,0,PUSHPULL);
    gpio_init(LED1_PIN,GPO,0,PUSHPULL);
    gpio_init(LED2_PIN,GPO,0,PUSHPULL);
    gpio_init(LED3_PIN,GPO,0,PUSHPULL);
}
