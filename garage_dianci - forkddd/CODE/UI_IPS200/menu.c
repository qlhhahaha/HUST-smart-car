/*
 * menu.c
 *
 *  Created on: 2021年5月7日
 *      Author: 朱江禹
 */

#include "menu.h"

MenuInfo mi =
    {
        {{"Run", "BatVol", "MotorTest", "ServoTest",
          "CamTest", "slave_pid", "Img-Img", "Img-Test1",
          "MotorPid", "Inductest"},
         {"--------", "--------", "--------", "--------",
          "--------", "--------", "--------", "--------",
          "--------", "--------"},
         {"--------", "--------", "--------", "--------",
          "--------", "--------", "--------", "--------",
          "--------", "--------"},
         {"--------", "--------", "--------", "--------",
          "--------", "--------", "--------", "--------",
          "--------", "--------"}},
        {{&Empty_Fun, &Batui_Handle, &Motui_Handle, &Serui_Handle,
          &Camui_Handle, &Pidui_Handle, &Imgui_Handle, &Test1ui_Handle,
          &Mpidui_Handle, &Inducui_Handle},
         {&Empty_Fun, &Empty_Fun, &Empty_Fun, &Empty_Fun,
          &Empty_Fun, &Empty_Fun, &Empty_Fun, &Empty_Fun,
          &Empty_Fun, &Empty_Fun},
         {&Empty_Fun, &Empty_Fun, &Empty_Fun, &Empty_Fun,
          &Empty_Fun, &Empty_Fun, &Empty_Fun, &Empty_Fun,
          &Empty_Fun, &Empty_Fun},
         {&Empty_Fun, &Empty_Fun, &Empty_Fun, &Empty_Fun,
          &Empty_Fun, &Empty_Fun, &Empty_Fun, &Empty_Fun,
          &Empty_Fun, &Empty_Fun}},
        0,
        0,
        7,
        7};

extern KeyInfo ki;
extern const uint8 tft_ascii[95][16];

void Menu_Init(void)
{
    Menu_Draw(&mi);
}

void Empty_Fun(void)
{
    ips200_clear(IPS200_BGCOLOR);
    ips200_showstr(4, 1, "empty option!");
    systick_delay_ms(STM1, 1000);
}

void Menu_ClrLine(uint16 y, uint16 color)
{
    uint16 i, j, ymin, ymax;
    ymin = y * MENU_CHAR_H;
    ymax = (y + 1) * MENU_CHAR_H;
    ips200_address_set(0, ymin, IPS200_X_MAX - 1, ymax - 1);
    for (i = 0; i < IPS200_X_MAX; i++)
    {
        for (j = ymin; j < ymax; j++)
        {
            ips200_wr_data16(color);
        }
    }
}

void Menu_ShowChar(uint16 x, uint16 y, const int8 dat, uint16 color, uint16 bgcolor)
{
    uint8 i, j;
    uint8 temp;

    for (i = 0; i < 16; i++)
    {
        ips200_address_set(x, y + i, x + 7, y + i);
        temp = tft_ascii[(uint16)dat - 32][i]; //减32因为是取模是从空格开始取得 空格在ascii中序号是32
        for (j = 0; j < 8; j++)
        {
            if (temp & 0x01)
                ips200_wr_data16(color);
            else
                ips200_wr_data16(bgcolor);
            temp >>= 1;
        }
    }
}

void Menu_DrawLine(MenuInfo *pmi, uint16 page, uint16 opt,
                   uint16 color, uint16 bgcolor)
{
    uint8 *pch;
    uint16 nowx, nowy;
    nowy = opt * MENU_CHAR_H;
    nowx = 2 * MENU_CHAR_W;
    pch = pmi->opts[page][opt]; //指向指定的字符串
    while (*pch)
    {
        Menu_ShowChar(nowx, nowy, *pch, color, bgcolor);
        nowx += MENU_CHAR_W;
        pch++;
    }
}

void Menu_Draw(MenuInfo *pmi)
{
    uint16 i;
    ips200_clear(IPS200_BGCOLOR);
    for (i = 0; i < MENU_OPT_PAGE; i++)
    {
        if (i != pmi->opt_now)
            Menu_DrawLine(pmi, pmi->page_now, i,
                          IPS200_PENCOLOR, IPS200_BGCOLOR);
        else
        {
            Menu_ClrLine(pmi->opt_now, IPS200_PENCOLOR);
            Menu_DrawLine(pmi, pmi->page_now, i,
                          IPS200_BGCOLOR, IPS200_PENCOLOR);
        }
    }
}

void Menu_Handle(MenuInfo *pmi)
{
    Key_Scan(&ki);
    systick_delay_ms(STM1, 10);
    if (ki.key_now[KEY_UP] && (!ki.key_last[KEY_UP])) //上键
    {
        pmi->opt_last = pmi->opt_now;
        pmi->page_last = pmi->page_now;
        if (pmi->opt_now == 0)
        {
            if (pmi->page_now == 0)
                pmi->page_now += (MENU_PAGE_NUM - 1);
            else
                pmi->page_now--;
            pmi->opt_now += (MENU_OPT_PAGE - 1);
            Menu_Draw(pmi);
        }
        else
        {
            pmi->opt_now--;
            Menu_ClrLine(pmi->opt_last, IPS200_BGCOLOR);
            Menu_DrawLine(pmi, pmi->page_last, pmi->opt_last,
                          IPS200_PENCOLOR, IPS200_BGCOLOR);
            Menu_ClrLine(pmi->opt_now, IPS200_PENCOLOR);
            Menu_DrawLine(pmi, pmi->page_now, pmi->opt_now,
                          IPS200_BGCOLOR, IPS200_PENCOLOR);
        }
    }
    else if (ki.key_now[KEY_DOWN] && (!ki.key_last[KEY_DOWN])) //下键
    {
        pmi->opt_last = pmi->opt_now;
        pmi->page_last = pmi->page_now;
        if (pmi->opt_now == (MENU_OPT_PAGE - 1))
        {
            if (pmi->page_now == (MENU_PAGE_NUM - 1))
                pmi->page_now -= (MENU_PAGE_NUM - 1);
            else
                pmi->page_now++;
            pmi->opt_now -= (MENU_OPT_PAGE - 1);
            Menu_Draw(pmi);
        }
        else
        {
            pmi->opt_now++;
            Menu_ClrLine(pmi->opt_last, IPS200_BGCOLOR);
            Menu_DrawLine(pmi, pmi->page_last, pmi->opt_last,
                          IPS200_PENCOLOR, IPS200_BGCOLOR);
            Menu_ClrLine(pmi->opt_now, IPS200_PENCOLOR);
            Menu_DrawLine(pmi, pmi->page_now, pmi->opt_now,
                          IPS200_BGCOLOR, IPS200_PENCOLOR);
        }
    }
    else if (ki.key_now[KEY_ENTER] && (!ki.key_last[KEY_ENTER])) //确认键
    {
        pmi->pfun[pmi->page_now][pmi->opt_now]();
        Menu_Draw(pmi);
    }
}
