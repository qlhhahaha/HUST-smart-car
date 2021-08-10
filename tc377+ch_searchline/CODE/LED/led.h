/*
 * led.h
 *
 *  Created on: 2021年5月8日
 *      Author: 朱江禹
 */

#ifndef _LED_H_
#define _LED_H_

#include "common.h"
#include "zf_gpio.h"

#define LED0_PIN P33_7
#define LED1_PIN P33_6
#define LED2_PIN P33_5
#define LED3_PIN P33_4

#define LED0(x) gpio_set(LED0_PIN,x)
#define LED1(x) gpio_set(LED1_PIN,x)
#define LED2(x) gpio_set(LED2_PIN,x)
#define LED3(x) gpio_set(LED3_PIN,x)

#define LED_0 gpio_get(LED0_PIN)
#define LED_1 gpio_get(LED1_PIN)
#define LED_2 gpio_get(LED2_PIN)
#define LED_3 gpio_get(LED3_PIN)

void LED_Init(void);

#endif
