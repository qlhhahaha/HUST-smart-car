/*
 * sobui.c
 *
 *  Created on: 2021年5月20日
 *      Author: 朱江禹
 */

#include "sobui.h"

extern KeyInfo ki;
extern uint8 sobel_finish_flag;
extern uint8 pro_mode;
extern uint8 sobel[IMG_H][IMG_W];
extern float32 sobel_k;
extern float32 sobel_thsf;

void Sobui_Draw(void)
{
    ips200_clear(IPS200_BGCOLOR);
    ips200_showstr(7*MENU_CHAR_W,10,"sobel_k:");
    ips200_showfloat(15*MENU_CHAR_W,10,sobel_k,3,2);
    ips200_showstr(4*MENU_CHAR_W,11,"sobel_thsf:");
    ips200_showfloat(15*MENU_CHAR_W,11,sobel_thsf,3,2);
}

void Sobui_Refresh1(void)
{
    ips200_showstr(15*MENU_CHAR_W,10,"       ");
    ips200_showfloat(15*MENU_CHAR_W,10,sobel_k,3,2);
}

void Sobui_Refresh2(void)
{
    ips200_showstr(15*MENU_CHAR_W,11,"       ");
    ips200_showfloat(15*MENU_CHAR_W,11,sobel_thsf,3,2);
}

void Sobui_Handle(void)
{
    uint16 count;
    Sobui_Draw();
    pro_mode=SOBEL_MODE;
    count=0;
    while(1)
    {
        Key_Scan(&ki);
        systick_delay_ms(STM1,10);
        count++;
        if(count>10)
        {
            count=0;
            Sobui_Refresh2();
        }
        if(sobel_finish_flag)
        {
            sobel_finish_flag=0;
            ips200_displayimage032_zoom1((uint8*)sobel,
                    IMG_W,IMG_H,0,0,DIS_W,DIS_H);
            pro_mode=SOBEL_MODE;
        }
        if(ki.key_now[KEY_UP])
        {
            sobel_k+=0.04;
            Sobui_Refresh1();
        }
        else if(ki.key_now[KEY_DOWN])
        {
            sobel_k-=0.04;
            Sobui_Refresh1();
        }
        else if(ki.key_now[KEY_CANCEL]&&(!ki.key_last[KEY_CANCEL]))
            break;
    }
    pro_mode=0;
}
