/*
 * batui.h
 *
 *  Created on: 2021年5月9日
 *      Author: 朱江禹
 */

#ifndef _BATUI_H_
#define _BATUI_H_

#include "common.h"
#include "menu.h"
#include "key.h"
#include "SEEKFREE_IPS200_PARALLEL8.h"
#include "zf_stm_systick.h"
#include "led.h"
#include "bat.h"

#define VOL_0_0 7.2
#define VOL_0_1 7.245
#define VOL_1_0 7.585
#define VOL_1_1 7.63
#define VOL_2_0 7.97
#define VOL_2_1 8.015
#define VOL_3_0 8.355
#define VOL_3_1 8.4

void Batui_Draw(void);
void Batui_Refresh(void);
void Batui_Handle(void);
void Batui_LED(void);

#endif
