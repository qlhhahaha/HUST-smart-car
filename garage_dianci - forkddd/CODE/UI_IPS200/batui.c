/*
 * batui.c
 *
 *  Created on: 2021年5月9日
 *      Author: 朱江禹
 */

#include "batui.h"

extern int16 bat_value;
extern float64 bat_voltage;
extern KeyInfo ki;

void Batui_Draw(void)
{
    ips200_clear(IPS200_BGCOLOR);
    ips200_showstr(11*MENU_CHAR_W,1,"Bat_Test");
    ips200_showstr(9*MENU_CHAR_W,3,"value:");
    ips200_showint16(15*MENU_CHAR_W,3,bat_value);
    ips200_showstr(7*MENU_CHAR_W,4,"voltage:");
    ips200_showfloat(15*MENU_CHAR_W,4,bat_voltage,2,3);
}

void Batui_Refresh(void)
{
    ips200_showstr(15*MENU_CHAR_W,3,"      ");
    ips200_showint16(15*MENU_CHAR_W,3,bat_value);
    ips200_showstr(15*MENU_CHAR_W,4,"       ");
    ips200_showfloat(15*MENU_CHAR_W,4,bat_voltage,2,3);
}

void Batui_Handle(void)
{
    int16 count;
    count=0;
    Batui_Draw();
    LED0(1);
    LED1(1);
    LED2(1);
    LED3(1);
    Batui_LED();
    while(1)
    {
        Key_Scan(&ki);
        systick_delay_ms(STM1,10);
        count++;
        if(count>=50)   //五十次按键一次刷新
        {
            count=0;
            Bat_GetInfo();
            Batui_Refresh();
            Batui_LED();
        }
        if(ki.key_now[KEY_CANCEL]&&(!ki.key_last[KEY_CANCEL]))   //取消键
            break;
    }
    LED0(0);
    LED1(0);
    LED2(0);
    LED3(0);
}

void Batui_LED(void)
{
    if(LED_0)
    {
        if(bat_voltage<VOL_0_0)
            LED0(0);
    }
    else
    {
        if(bat_voltage>VOL_0_1)
            LED0(1);
    }
    if(LED_1)
    {
        if(bat_voltage<VOL_1_0)
            LED1(0);
    }
    else
    {
        if(bat_voltage>VOL_1_1)
            LED1(1);
    }
    if(LED_2)
    {
        if(bat_voltage<VOL_2_0)
            LED2(0);
    }
    else
    {
        if(bat_voltage>VOL_2_1)
            LED2(1);
    }
    if(LED_3)
    {
        if(bat_voltage<VOL_3_0)
            LED3(0);
    }
    else
    {
        if(bat_voltage>VOL_3_1)
            LED3(1);
    }
}
