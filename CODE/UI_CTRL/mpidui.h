/*
 * mpidui.h
 *
 *  Created on: 2021年5月29日
 *      Author: 朱江禹
 */

#ifndef _MPIDUI_H_
#define _MPIDUI_H_

#include "common.h"
#include "menu.h"
#include "key.h"
#include "SEEKFREE_IPS200_PARALLEL8.h"
#include "zf_stm_systick.h"
#include "motor.h"

void Mpidui_Draw(void);
void Mpidui_Refresh_s(void);
void Mpidui_Refresh_t(void);
void Mpidui_Refresh_p(void);
void Mpidui_Refresh_i(void);
void Mpidui_Refresh_d(void);
void Mpidui_Handle(void);

#endif
