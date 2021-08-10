/*
 * sobui.h
 *
 *  Created on: 2021年5月20日
 *      Author: 朱江禹
 */

#ifndef _SOBUI_H_
#define _SOBUI_H_

#include "common.h"
#include "menu.h"
#include "key.h"
#include "SEEKFREE_IPS200_PARALLEL8.h"
#include "zf_stm_systick.h"

void Sobui_Draw(void);
void Sobui_Refresh1(void);
void Sobui_Refresh2(void);
void Sobui_Handle(void);

#endif
