/*
 * serui.h
 *
 *  Created on: 2021��5��9��
 *      Author: �콭��
 */

#ifndef _SERUI_H_
#define _SERUI_H_

#include "common.h"
#include "menu.h"
#include "key.h"
#include "SEEKFREE_IPS200_PARALLEL8.h"
#include "servo.h"

void Serui_Draw(void);
void Serui_Refresh1(void);
void Serui_Refresh2(void);
void Serui_Handle(void);

#endif
