/*
 * serui.c
 *
 *  Created on: 2021年5月9日
 *      Author: 朱江禹
 */

#include "serui.h"

extern KeyInfo ki;
extern uint32 servo_duty;
extern uint32 servo_ref[3];
uint16 servo_set;

void Serui_Draw(void)
{
    ips200_clear(IPS200_BGCOLOR);
    ips200_showstr(10*MENU_CHAR_W,1,"Servo_Test");
    ips200_showstr(5*MENU_CHAR_W,3,"servo_set:");
    ips200_showint16(15*MENU_CHAR_W,3,(int16)servo_set);
    ips200_showstr(4*MENU_CHAR_W,4,"servo_duty:");
    ips200_showint16(15*MENU_CHAR_W,4,(int16)servo_duty);
}

void Serui_Refresh1(void)
{
    ips200_showstr(15*MENU_CHAR_W,3,"      ");
    ips200_showint16(15*MENU_CHAR_W,3,(int16)servo_set);
}

void Serui_Refresh2(void)
{
    ips200_showstr(15*MENU_CHAR_W,4,"      ");
    ips200_showint16(15*MENU_CHAR_W,4,(int16)servo_duty);
}

void Serui_Handle(void)
{
    servo_set=(uint16)servo_duty;
    Serui_Draw();
    Servo_Duty();
    while(1)
    {
        Key_Scan(&ki);
        systick_delay_ms(STM1,10);
        if(ki.key_now[KEY_UP])
        {
            servo_set+=10;
            if(servo_set>servo_ref[2])
                servo_set=(uint16)servo_ref[2];
            Serui_Refresh1();
        }
        else if(ki.key_now[KEY_DOWN])
        {
            servo_set-=10;
            if(servo_set<servo_ref[0])
                servo_set=(uint16)servo_ref[0];
            Serui_Refresh1();
        }
        else if(ki.key_now[KEY_ENTER]&&(!ki.key_last[KEY_ENTER]))
        {
            servo_duty=(uint32)servo_set;
            Serui_Refresh2();
            Servo_Duty();
        }
        else if(ki.key_now[KEY_CANCEL]&&(!ki.key_last[KEY_CANCEL]))
            break;
    }
    Servo_Duty();
}
