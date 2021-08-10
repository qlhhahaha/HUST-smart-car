/*
 * mpidui.c
 *
 *  Created on: 2021年5月29日
 *      Author: 朱江禹
 */

#include "mpidui.h"

extern KeyInfo ki;
extern int16 enc_spd;
extern int32 motor_duty;
extern M_PID mpid;
extern uint8 mpid_flag;

void Mpidui_Draw(void)
{
    ips200_clear(IPS200_BGCOLOR);
    ips200_showstr(11 * MENU_CHAR_W, 1, "Motor_Pid");
    ips200_showstr(7 * MENU_CHAR_W, 3, "enc_tar:");
    ips200_showint16(15 * MENU_CHAR_W, 3, mpid.spd_tar);
    ips200_showstr(7 * MENU_CHAR_W, 4, "enc_spd:");
    ips200_showint16(15 * MENU_CHAR_W, 4, enc_spd);
    ips200_showstr(4 * MENU_CHAR_W, 5, "motor_duty:");
    ips200_showint16(15 * MENU_CHAR_W, 5, (int16)motor_duty);
    ips200_showstr(7 * MENU_CHAR_W, 6, "motor_p:");
    ips200_showfloat(15 * MENU_CHAR_W, 6, mpid.m_p, 3, 2);
    ips200_showstr(7 * MENU_CHAR_W, 7, "motor_i:");
    ips200_showfloat(15 * MENU_CHAR_W, 7, mpid.m_i, 3, 2);
    ips200_showstr(7 * MENU_CHAR_W, 8, "motor_d:");
    ips200_showfloat(15 * MENU_CHAR_W, 8, mpid.m_d, 3, 2);
}

void Mpidui_Refresh_s(void)
{
    ips200_showstr(15 * MENU_CHAR_W, 4, "      ");
    ips200_showint16(15 * MENU_CHAR_W, 4, enc_spd);
    ips200_showstr(15 * MENU_CHAR_W, 5, "      ");
    ips200_showint16(15 * MENU_CHAR_W, 5, (int16)motor_duty);
}

void Mpidui_Refresh_t(void)
{
    ips200_showstr(15 * MENU_CHAR_W, 3, "      ");
    ips200_showint16(15 * MENU_CHAR_W, 3, mpid.spd_tar);
}

void Mpidui_Refresh_p(void)
{
    ips200_showstr(15 * MENU_CHAR_W, 6, "       ");
    ips200_showfloat(15 * MENU_CHAR_W, 6, mpid.m_p, 3, 2);
}

void Mpidui_Refresh_i(void)
{
    ips200_showstr(15 * MENU_CHAR_W, 7, "       ");
    ips200_showfloat(15 * MENU_CHAR_W, 7, mpid.m_i, 3, 2);
}

void Mpidui_Refresh_d(void)
{
    ips200_showstr(15 * MENU_CHAR_W, 8, "       ");
    ips200_showfloat(15 * MENU_CHAR_W, 8, mpid.m_d, 3, 2);
}

void Mpidui_Handle(void)
{
    uint16 count, choice;
    Mpidui_Draw();
    mpid_flag = 1; //用标志位在中断里开启pid算法
    count = 0;
    choice = 0; //0为t，1为p，2为i，3为d
    while (1)
    {
        Key_Scan(&ki);
        systick_delay_ms(STM1, 10);
        count++;
        if (count > 20)
        {
            count = 0;
            Mpidui_Refresh_s();
        }
        if (ki.key_now[KEY_UP])
        {
            switch (choice)
            {
            case 0:
                mpid.spd_tar++;
                Mpidui_Refresh_t();
                break;
            case 1:
                mpid.m_p += 0.01;
                Mpidui_Refresh_p();
                break;
            case 2:
                mpid.m_i += 0.01;
                Mpidui_Refresh_i();
                break;
            case 3:
                mpid.m_d += 0.01;
                Mpidui_Refresh_d();
                break;
            }
        }
        else if (ki.key_now[KEY_DOWN])
        {
            switch (choice)
            {
            case 0:
                mpid.spd_tar--;
                Mpidui_Refresh_t();
                break;
            case 1:
                mpid.m_p -= 0.01;
                if (mpid.m_p < 0.0)
                    mpid.m_p = 0.0;
                Mpidui_Refresh_p();
                break;
            case 2:
                mpid.m_i -= 0.01;
                if (mpid.m_i < 0.0)
                    mpid.m_i = 0.0;
                Mpidui_Refresh_i();
                break;
            case 3:
                mpid.m_d -= 0.01;
                if (mpid.m_d < 0.0)
                    mpid.m_d = 0.0;
                Mpidui_Refresh_d();
                break;
            }
        }
        else if (ki.key_now[KEY_ENTER] && (!ki.key_last[KEY_ENTER]))
        {
            choice = (choice >= 3) ? 0 : (choice + 1);
        }
        else if (ki.key_now[KEY_CANCEL] && (!ki.key_last[KEY_CANCEL]))
            break;
    }
    mpid_flag = 0;
    motor_duty = 0;
    Motor_Duty();
    Mpid_Init();
}
