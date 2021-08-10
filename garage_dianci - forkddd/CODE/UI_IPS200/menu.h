/*
 * menu.h
 *
 *  Created on: 2021年5月7日
 *      Author: 朱江禹
 */

#ifndef _MENU_H_
#define _MENU_H_

#include "common.h"
#include "key.h"
#include "SEEKFREE_IPS200_PARALLEL8.h"
#include "zf_stm_systick.h"
#include "led.h"

#include "pidui.h"
#include "batui.h"
#include "motui.h"
#include "serui.h"
#include "camui.h"
#include "sobui.h"
#include "imgui.h"
#include "inducui.h"

#include "test1ui.h"

#include "mpidui.h"

#define MENU_OPT_PAGE 10    //每页的选项个数
#define MENU_PAGE_NUM 4     //页数
#define MENU_OPT_SLEN 19    //菜单选项长度

#define MENU_CHAR_W 8
#define MENU_CHAR_H 16

#define KEY_UP 0
#define KEY_DOWN 1
#define KEY_ENTER 2
#define KEY_CANCEL 3        //按键功能对应结构体的数组下标

typedef struct
{
        uint8 opts[MENU_PAGE_NUM][MENU_OPT_PAGE][MENU_OPT_SLEN];
        void(* pfun[MENU_PAGE_NUM][MENU_OPT_PAGE])(void);
        int16 page_last;
        int16 page_now;
        int16 opt_last;
        int16 opt_now;
}MenuInfo;

void Menu_Init(void);
void Empty_Fun(void);
void Menu_ClrLine(uint16 y,uint16 color);
void Menu_ShowChar(uint16 x,uint16 y,const int8 dat,uint16 color,uint16 bgcolor);
void Menu_DrawLine(MenuInfo* pmi,uint16 page,uint16 opt,
        uint16 color,uint16 bgcolor);
void Menu_Draw(MenuInfo* pmi);
void Menu_Handle(MenuInfo* pmi);

#endif






























