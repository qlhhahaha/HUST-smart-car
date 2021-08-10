/*
 * motui.c
 *
 *  Created on: 2021年5月9日
 *      Author: 朱江禹
 */

#include "motui.h"

extern KeyInfo ki;
extern int16 enc_spd;
extern int32 motor_duty;
int16 motor_set;

void Motui_Draw(void)
{
    ips200_clear(IPS200_BGCOLOR);
    ips200_showstr(10*MENU_CHAR_W,1,"Motor_Test");
    ips200_showstr(7*MENU_CHAR_W,3,"enc_spd:");
    ips200_showint16(15*MENU_CHAR_W,3,enc_spd);
    ips200_showstr(5*MENU_CHAR_W,4,"motor_set:");
    ips200_showint16(15*MENU_CHAR_W,4,motor_set);
    ips200_showstr(4*MENU_CHAR_W,5,"motor_duty:");
    ips200_showint16(15*MENU_CHAR_W,5,(int16)motor_duty);
}

void Motui_Refresh1(void)
{
    ips200_showstr(15*MENU_CHAR_W,3,"      ");
    ips200_showint16(15*MENU_CHAR_W,3,enc_spd);
}

void Motui_Refresh2(void)
{
    ips200_showstr(15*MENU_CHAR_W,4,"      ");
    ips200_showint16(15*MENU_CHAR_W,4,motor_set);
}

void Motui_Refresh3(void)
{
    ips200_showstr(15*MENU_CHAR_W,5,"      ");
    ips200_showint16(15*MENU_CHAR_W,5,(int16)motor_duty);
}

void Motui_Handle(void)
{
    int16 count;
    count=0;
    motor_set=0;
    motor_duty=0;
    Motui_Draw();
    Motor_Duty();
    while(1)
    {
        Key_Scan(&ki);
        systick_delay_ms(STM1,10);
        count++;
        if(count>=50)
        {
            count=0;
            Motui_Refresh1();
        }
        if(ki.key_now[KEY_UP])
        {
            motor_set+=10;
            Motui_Refresh2();
        }
        else if(ki.key_now[KEY_DOWN])
        {
            motor_set-=10;
            Motui_Refresh2();
        }
        else if(ki.key_now[KEY_ENTER]&&(!ki.key_last[KEY_ENTER]))
        {
            motor_duty=(int32)motor_set;
            Motui_Refresh3();
            Motor_Duty();
        }
        else if(ki.key_now[KEY_CANCEL]&&(!ki.key_last[KEY_CANCEL]))
            break;
    }
    motor_set=0;
    motor_duty=0;
    Motor_Duty();
}
