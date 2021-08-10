/*
 * test1ui.c
 *
 *  Created on: 2021年5月21日
 *      Author: 朱江禹
 */

#include "test1ui.h"

extern KeyInfo ki;
extern uint8 test1_finish_flag;
extern uint8 pro_mode;
extern uint8 sobel[IMG_H][IMG_W];
extern ImgData *pd_now;
extern float32 error[5];
extern uint8 spid_flag;
extern uint8 mpid_flag;
extern int32 motor_duty;
extern int16 fork_check_flag;
extern int16 fork_check_flag_1;

extern int16 enter_jump_count;

extern int16 max_col;

extern int16 col_l;
extern int16 col_r;

extern int16 l_angle_flag;
extern int16 r_angle_flag;

float32 tsum;

void Test1ui_Draw(void)
{
    ips200_clear(IPS200_BGCOLOR);
    ips200_showstr(7 * MENU_CHAR_W, 5, "state:");
    ips200_showuint16(15 * MENU_CHAR_W, 5, pd_now->state);

    ips200_showstr(7 * MENU_CHAR_W, 6, "max_col:");
    ips200_showint16(15 * MENU_CHAR_W, 6, max_col);

    ips200_showstr(7 * MENU_CHAR_W, 7, "col_l:");
    ips200_showint16(15 * MENU_CHAR_W, 7, col_l);

    ips200_showstr(7 * MENU_CHAR_W, 8, "col_r:");
    ips200_showint16(15 * MENU_CHAR_W, 8, col_r);

    ips200_showstr(7 * MENU_CHAR_W, 9, "fork1:");
    ips200_showint16(15 * MENU_CHAR_W, 9, fork_check_flag_1);

    ips200_showstr(7 * MENU_CHAR_W, 10, "tri_sum:");
    ips200_showfloat(15 * MENU_CHAR_W, 10, tsum, 3, 2);

    ips200_showstr(7 * MENU_CHAR_W, 11, "l_angle:");
    ips200_showint16(15 * MENU_CHAR_W, 11, l_angle_flag);

    ips200_showstr(7 * MENU_CHAR_W, 12, "r_angle:");
    ips200_showint16(15 * MENU_CHAR_W, 12, r_angle_flag);

    ips200_showstr(7 * MENU_CHAR_W, 13, "jump:");
    ips200_showint16(15 * MENU_CHAR_W, 13, enter_jump_count);

    ips200_showstr(7 * MENU_CHAR_W, 14, "l_lost:");
    ips200_showint16(15 * MENU_CHAR_W, 14, pd_now->l1.lost);

    ips200_showstr(7 * MENU_CHAR_W, 15, "r_lost:");
    ips200_showint16(15 * MENU_CHAR_W, 15, pd_now->r1.lost);
}

void Test1ui_Refresh1(void)
{
    ips200_showstr(7 * MENU_CHAR_W, 5, "state:");
    ips200_showuint16(15 * MENU_CHAR_W, 5, pd_now->state);

    ips200_showstr(7 * MENU_CHAR_W, 6, "max_col:");
    ips200_showint16(15 * MENU_CHAR_W, 6, max_col);

    ips200_showstr(7 * MENU_CHAR_W, 7, "col_l:");
    ips200_showint16(15 * MENU_CHAR_W, 7, col_l);

    ips200_showstr(7 * MENU_CHAR_W, 8, "col_r:");
    ips200_showint16(15 * MENU_CHAR_W, 8, col_r);

    ips200_showstr(7 * MENU_CHAR_W, 9, "fork1:");
    ips200_showint16(15 * MENU_CHAR_W, 9, fork_check_flag_1);

    ips200_showstr(7 * MENU_CHAR_W, 10, "tri_sum:");
    ips200_showfloat(15 * MENU_CHAR_W, 10, tsum, 3, 2);

    ips200_showstr(7 * MENU_CHAR_W, 11, "l_angle:");
    ips200_showint16(15 * MENU_CHAR_W, 11, l_angle_flag);

    ips200_showstr(7 * MENU_CHAR_W, 12, "r_angle:");
    ips200_showint16(15 * MENU_CHAR_W, 12, r_angle_flag);

    ips200_showstr(7 * MENU_CHAR_W, 13, "jump:");
    ips200_showint16(15 * MENU_CHAR_W, 13, enter_jump_count);

    ips200_showstr(7 * MENU_CHAR_W, 14, "l_lost:");
    ips200_showint16(15 * MENU_CHAR_W, 14, pd_now->l1.lost);

    ips200_showstr(7 * MENU_CHAR_W, 15, "r_lost:");
    ips200_showint16(15 * MENU_CHAR_W, 15, pd_now->r1.lost);
}

void Test1ui_Zoom1(uint8 *p, uint16 width, uint16 height,
                   uint16 start_x, uint16 start_y, uint16 dis_width, uint16 dis_height)
{
    uint32 i, j;

    uint16 color = 0;
    uint16 temp = 0;

    ips200_address_set(start_x, start_y, start_x + dis_width - 1, start_y + dis_height - 1); //设置显示区域

    for (j = 0; j < dis_height; j++)
    {
        for (i = 0; i < dis_width; i++)
        {
            temp = *(p + (j * height / dis_height) * width + i * width / dis_width); //读取像素点
            switch (temp & 0x00ff)
            {
            case 0:
                color = 0;
                break;
            case 255:
                color = 0xffff;
                break;
            case 1:
                color = RED;
                break;
            case 2:
                color = BLUE;
                break;
            }
            ips200_wr_data16(color);
        }
    }
}

void Test1ui_Handle(void)
{
    int16 count;
    Test1ui_Draw();
    pro_mode = TEST1_MODE;
    count = 0;
    spid_flag = 1;
    mpid_flag = 1;
    //motor_duty=4500;
    //Motor_Duty();
    while (1)
    {
        Key_Scan(&ki);
        systick_delay_ms(STM1, 10);
        count++;
        if (count > 4)
        {
            count = 0;
            Test1ui_Refresh1();
        }
        if (test1_finish_flag)
        {
            test1_finish_flag = 0;
            Test1ui_Zoom1((uint8 *)sobel,
                          IMG_W, IMG_H, 0, 0, DIS_W, DIS_H);
            /*tsum=pd_now->tri_sum;*/
            pro_mode = TEST1_MODE;
        }
        if (ki.key_now[KEY_CANCEL])
            break;
    }
    spid_flag = 0;
    mpid_flag = 0;
    motor_duty = 0;
    Motor_Duty();
    pro_mode = 0;
}
