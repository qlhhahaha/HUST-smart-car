/*
 * imgui.h
 *
 *  Created on: 2021��5��21��
 *      Author: �콭��
 */

#ifndef _IMGUI_H_
#define _IMGUI_H_

#include "common.h"
#include "menu.h"
#include "key.h"
#include "SEEKFREE_IPS200_PARALLEL8.h"
#include "zf_stm_systick.h"

void Imgui_Draw(void);
void Imgui_Refresh1(void);
void Imgui_Refresh2(void);
void Imgui_Refresh3(void);
void Imgui_Refresh4(void);
void Imgui_Handle(void);

#endif
