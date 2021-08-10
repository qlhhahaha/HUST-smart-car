/*
 * camui.c
 *
 *  Created on: 2021年5月9日
 *      Author: 朱江禹
 */

#include "camui.h"

extern uint8 cam_flag;
extern KeyInfo ki;
extern uint8 mt9v03x_finish_flag;
extern uint8 mt9v03x_image[CAM_W][CAM_H];

void Camui_Draw(void)
{
    ips200_clear(IPS200_BGCOLOR);
}

void Camui_Handle(void)
{
    Camui_Draw();
    while(1)
    {
        Key_Scan(&ki);
        if(mt9v03x_finish_flag)
        {
            ips200_displayimage032_zoom1((uint8*)mt9v03x_image,
                    CAM_W,CAM_H,0,0,DIS_W,DIS_H);
            cam_flag=1;
        }
        if(ki.key_now[KEY_CANCEL]&&(!ki.key_last[KEY_CANCEL]))
            break;
    }
    cam_flag=0;
}
