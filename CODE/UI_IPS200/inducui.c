
#include "inducui.h"


extern int16 induc_value[5];
extern KeyInfo ki;

void Inducui_Draw(void)
{
    ips200_clear(IPS200_BGCOLOR);
    ips200_showstr(11 * MENU_CHAR_W, 1, "Induc_Test");

    ips200_showstr(9 * MENU_CHAR_W, 3, "value1:");
    ips200_showint16(15 * MENU_CHAR_W, 3, induc_value[0]);

    ips200_showstr(9 * MENU_CHAR_W, 4, "value2:");
    ips200_showint16(15 * MENU_CHAR_W, 4, induc_value[1]);

    ips200_showstr(9 * MENU_CHAR_W, 5, "value3:");
    ips200_showint16(15 * MENU_CHAR_W, 5, induc_value[2]);

    ips200_showstr(9 * MENU_CHAR_W, 6, "value4:");
    ips200_showint16(15 * MENU_CHAR_W, 6, induc_value[3]);

    ips200_showstr(9 * MENU_CHAR_W, 7, "value5:");
    ips200_showint16(15 * MENU_CHAR_W, 7, induc_value[4]);
}

void Inducui_Refresh(void)
{
    ips200_showstr(9 * MENU_CHAR_W, 3, "      ");
    ips200_showint16(15 * MENU_CHAR_W, 3, induc_value[0]);

    ips200_showstr(9 * MENU_CHAR_W, 4, "      ");
    ips200_showint16(15 * MENU_CHAR_W, 4, induc_value[1]);

    ips200_showstr(9 * MENU_CHAR_W, 5, "      ");
    ips200_showint16(15 * MENU_CHAR_W, 5, induc_value[2]);

    ips200_showstr(9 * MENU_CHAR_W, 6, "      ");
    ips200_showint16(15 * MENU_CHAR_W, 6, induc_value[3]);

    ips200_showstr(7 * MENU_CHAR_W, 7, "      ");
    ips200_showint16(15 * MENU_CHAR_W, 7, induc_value[4]);
}

void Inducui_Handle(void)
{
    int16 count;
    count = 0;
    Inducui_Draw();
    while (1)
    {
        Key_Scan(&ki);
        systick_delay_ms(STM1, 10);
        count++;
        if (count >= 50) //五十次按键一次刷新
        {
            count = 0;
            Induc_GetInfo();
            Inducui_Refresh();
        }
        if (ki.key_now[KEY_CANCEL] && (!ki.key_last[KEY_CANCEL])) //取消键
            break;
    }
}
