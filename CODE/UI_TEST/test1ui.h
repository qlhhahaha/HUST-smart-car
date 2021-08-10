/*
 * test1ui.h
 *
 *  Created on: 2021年5月21日
 *      Author: 朱江禹
 */

#ifndef _TEST1UI_H_
#define _TEST1UI_H_

#include "common.h"
#include "menu.h"
#include "key.h"
#include "SEEKFREE_IPS200_PARALLEL8.h"
#include "zf_stm_systick.h"

void Test1ui_Draw(void);
void Test1ui_Refresh1(void);
void Test1ui_Zoom1(uint8* p, uint16 width, uint16 height,
        uint16 start_x, uint16 start_y, uint16 dis_width, uint16 dis_height);
void Test1ui_Handle(void);

#endif
