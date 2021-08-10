/*
 * motui.h
 *
 *  Created on: 2021年5月9日
 *      Author: 朱江禹
 */

#ifndef _MOTUI_H_
#define _MOTUI_H_

#include "common.h"
#include "menu.h"
#include "key.h"
#include "SEEKFREE_IPS200_PARALLEL8.h"
#include "zf_stm_systick.h"
#include "enc.h"
#include "motor.h"

void Motui_Draw(void);
void Motui_Refresh1(void);
void Motui_Refresh2(void);
void Motui_Refresh3(void);
void Motui_Handle(void);

#endif
