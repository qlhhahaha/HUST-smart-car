/*
 * imgui.c
 *
 *  Created on: 2021年5月21日
 *      Author: 朱江禹
 */

#include "imgui.h"

extern KeyInfo ki;
extern uint8 img_finish_flag;
extern uint8 pro_mode;
extern uint8 sobel[IMG_H][IMG_W];
extern uint8 fixed_thres[IMG_H][IMG_W];

extern float32 sobel_k;
extern float32 sobel_thsf;
extern float32 img_k;
extern float32 img_thsf;

extern int16 fixed_thres_value;

void Imgui_Draw(void)
{
    ips200_clear(IPS200_BGCOLOR);
    ips200_showstr(7*MENU_CHAR_W,10,"sobel_k:");
    ips200_showfloat(15*MENU_CHAR_W,10,sobel_k,3,2);
    ips200_showstr(4*MENU_CHAR_W,11,"sobel_thsf:");
    ips200_showfloat(15*MENU_CHAR_W,11,sobel_thsf,3,2);
    ips200_showstr(9*MENU_CHAR_W,12,"img_k:");
    ips200_showfloat(15*MENU_CHAR_W,12,img_k,3,2);
    ips200_showstr(4*MENU_CHAR_W,13,"img_thsf:");
    ips200_showfloat(15*MENU_CHAR_W,13,img_thsf,3,2);
}

void Imgui_Refresh1(void)
{
    ips200_showstr(15*MENU_CHAR_W,10,"       ");
    ips200_showfloat(15*MENU_CHAR_W,10,sobel_k,3,2);
}

void Imgui_Refresh2(void)
{
    ips200_showstr(15*MENU_CHAR_W,11,"       ");
    ips200_showfloat(15*MENU_CHAR_W,11,sobel_thsf,3,2);
}

void Imgui_Refresh3(void)
{
    ips200_showstr(15*MENU_CHAR_W,12,"       ");
    ips200_showfloat(15*MENU_CHAR_W,12,img_k,3,2);
}

void Imgui_Refresh4(void)
{
    ips200_showstr(15*MENU_CHAR_W,13,"       ");
    ips200_showfloat(15*MENU_CHAR_W,13,img_thsf,3,2);
}

void Imgui_Handle(void)
{
    uint16 count,choice;
    Imgui_Draw();
    pro_mode=IMAGE_MODE;
    count=0;
    choice=0;   //0为sobel，1为img
    while(1)
    {
        Key_Scan(&ki);
        systick_delay_ms(STM1,10);
        count++;
        if(count>10)
        {
            count=0;
            Imgui_Refresh2();
            Imgui_Refresh4();
        }
        if(img_finish_flag)
        {
            img_finish_flag=0;
            ips200_displayimage032_zoom1((uint8*)sobel,
                    IMG_W,IMG_H,0,0,DIS_W,DIS_H);
            pro_mode=IMAGE_MODE;
        }
        if(ki.key_now[KEY_UP])
        {
            if(choice==0)
            {
                sobel_k+=0.04;
                Imgui_Refresh1();
            }
            else
            {
                img_k+=0.04;
                Imgui_Refresh3();
            }
        }
        else if(ki.key_now[KEY_DOWN])
        {
            if(choice==0)
            {
                sobel_k-=0.04;
                Imgui_Refresh1();
            }
            else
            {
                img_k-=0.04;
                Imgui_Refresh3();
            }
        }
        else if(ki.key_now[KEY_ENTER]&&(!ki.key_last[KEY_ENTER]))
        {
            choice=(choice==0)?1:0;
        }
        else if(ki.key_now[KEY_CANCEL]&&(!ki.key_last[KEY_CANCEL]))
            break;
    }
    pro_mode=0;
}

/*void Imgui_Draw(void)
{
    ips200_clear(IPS200_BGCOLOR);
    ips200_showstr(7 * MENU_CHAR_W, 10, "sobel_k:");
    ips200_showfloat(15 * MENU_CHAR_W, 10, sobel_k, 3, 2);
    ips200_showstr(4 * MENU_CHAR_W, 11, "fixed:");
    ips200_showint16(15 * MENU_CHAR_W, 11, fixed_thres_value);
    ips200_showstr(9 * MENU_CHAR_W, 12, "img_k:");
    ips200_showfloat(15 * MENU_CHAR_W, 12, img_k, 3, 2);
    ips200_showstr(4 * MENU_CHAR_W, 13, "img_thsf:");
    ips200_showfloat(15 * MENU_CHAR_W, 13, img_thsf, 3, 2);
}

void Imgui_Refresh1(void)
{
    ips200_showstr(15 * MENU_CHAR_W, 10, "       ");
    ips200_showfloat(15 * MENU_CHAR_W, 10, sobel_k, 3, 2);
}

void Imgui_Refresh2(void)
{
    ips200_showstr(15 * MENU_CHAR_W, 11, "       ");
    ips200_showint16(15 * MENU_CHAR_W, 11, fixed_thres_value);
}

void Imgui_Refresh3(void)
{
    ips200_showstr(15 * MENU_CHAR_W, 12, "       ");
    ips200_showfloat(15 * MENU_CHAR_W, 12, img_k, 3, 2);
}

void Imgui_Refresh4(void)
{
    ips200_showstr(15 * MENU_CHAR_W, 13, "       ");
    ips200_showfloat(15 * MENU_CHAR_W, 13, img_thsf, 3, 2);
}

void Imgui_Handle(void)
{
    uint16 count, choice;
    Imgui_Draw();
    pro_mode = IMAGE_MODE;
    count = 0;
    choice = 0; //0为sobel，1为img
    while (1)
    {
        Key_Scan(&ki);
        systick_delay_ms(STM1, 10);
        count++;
        if (count > 10)
        {
            count = 0;
            Imgui_Refresh2();
            Imgui_Refresh4();
        }
        if (img_finish_flag)
        {
            img_finish_flag = 0;
            ips200_displayimage032_zoom1((uint8 *)fixed_thres,
                                         IMG_W, IMG_H, 0, 0, DIS_W, DIS_H);
            pro_mode = IMAGE_MODE;
        }
        if (ki.key_now[KEY_UP])
        {
            if (choice == 0)
            {
                fixed_thres_value += 0.04;
                Imgui_Refresh2();
            }
            else
            {
                img_k += 0.04;
                Imgui_Refresh3();
            }
        }
        else if (ki.key_now[KEY_DOWN])
        {
            if (choice == 0)
            {
                fixed_thres_value -= 0.04;
                Imgui_Refresh2();
            }
            else
            {
                img_k -= 0.04;
                Imgui_Refresh3();
            }
        }
        else if (ki.key_now[KEY_ENTER] && (!ki.key_last[KEY_ENTER]))
        {
            choice = (choice == 0) ? 1 : 0;
        }
        else if (ki.key_now[KEY_CANCEL] && (!ki.key_last[KEY_CANCEL]))
            break;
    }
    pro_mode = 0;
}*/