/*
 * pidui.h
 *
 *  Created on: 2021年5月9日
 *      Author: 朱江禹
 */

#ifndef _PIDUI_H_
#define _PIDUI_H_

#include "common.h"
#include "menu.h"
#include "key.h"
#include "SEEKFREE_IPS200_PARALLEL8.h"
#include "zf_stm_systick.h"
#include "zf_eeprom.h"

void Pidui_Init(void);
void Pidui_Draw(void);
void Pidui_Refresh0(void);
void Pidui_Refresh1(void);
void Pidui_Refresh2(void);
void Pidui_Refresh3(void);
void Pidui_Refresh4(void);
void Pidui_Refresh5(void);
void Pidui_Refresh6(void);
void Pidui_Handle(void);

#endif
