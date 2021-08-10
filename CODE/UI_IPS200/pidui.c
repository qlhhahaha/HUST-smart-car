/*
 * pidui.c
 *
 *  Created on: 2021年5月9日
 *      Author: 朱江禹
 */

#include <pidui.h>

extern KeyInfo ki;
extern M_PID mpid;
extern S_PID spid;
float32 mmpid[3];
float32 sspid[3];
int16 mtar;

void Pidui_Init(void)
{
    if(flash_check(SCT_PID,PGE_MP))
        mmpid[0]=flash_read(SCT_PID,PGE_MP,float32);
    else
        mmpid[0]=mpid.m_p;
    if(flash_check(SCT_PID,PGE_MI))
        mmpid[1]=flash_read(SCT_PID,PGE_MI,float32);
    else
        mmpid[1]=mpid.m_i;
    if(flash_check(SCT_PID,PGE_MD))
        mmpid[2]=flash_read(SCT_PID,PGE_MD,float32);
    else
        mmpid[2]=mpid.m_d;
    if(flash_check(SCT_PID,PGE_MT))
        mtar=flash_read(SCT_PID,PGE_MT,int16);
    else
        mtar=mpid.spd_tar;
    if(flash_check(SCT_PID,PGE_SP))
        sspid[0]=flash_read(SCT_PID,PGE_SP,float32);
    else
        sspid[0]=spid.s_p;
    if(flash_check(SCT_PID,PGE_SI))
        sspid[1]=flash_read(SCT_PID,PGE_SI,float32);
    else
        sspid[1]=spid.s_i;
    if(flash_check(SCT_PID,PGE_SD))
        sspid[2]=flash_read(SCT_PID,PGE_SD,float32);
    else
        sspid[2]=spid.s_d;
}

void Pidui_Draw(void)
{
    ips200_clear(IPS200_BGCOLOR);
    ips200_showstr(10*MENU_CHAR_W,1,"Pid_Param");
    ips200_showstr(11*MENU_CHAR_W,3,"m_p:");
    ips200_showstr(11*MENU_CHAR_W,4,"m_i:");
    ips200_showstr(11*MENU_CHAR_W,5,"m_d:");
    ips200_showstr(11*MENU_CHAR_W,6,"m_t:");
    ips200_showstr(11*MENU_CHAR_W,7,"s_p:");
    ips200_showstr(11*MENU_CHAR_W,8,"s_i:");
    ips200_showstr(11*MENU_CHAR_W,9,"s_d:");
    ips200_showfloat(15*MENU_CHAR_W,3,mmpid[0],3,1);
    ips200_showfloat(15*MENU_CHAR_W,4,mmpid[1],3,1);
    ips200_showfloat(15*MENU_CHAR_W,5,mmpid[2],3,1);
    ips200_showint16(15*MENU_CHAR_W,6,mtar);
    ips200_showfloat(15*MENU_CHAR_W,7,sspid[0],3,1);
    ips200_showfloat(15*MENU_CHAR_W,8,sspid[1],3,1);
    ips200_showfloat(15*MENU_CHAR_W,9,sspid[2],3,1);
}

void Pidui_Refresh0(void)
{
    ips200_showstr(15*MENU_CHAR_W,3,"      ");
    ips200_showfloat(15*MENU_CHAR_W,3,mmpid[0],3,1);
}

void Pidui_Refresh1(void)
{
    ips200_showstr(15*MENU_CHAR_W,4,"      ");
    ips200_showfloat(15*MENU_CHAR_W,4,mmpid[1],3,1);
}

void Pidui_Refresh2(void)
{
    ips200_showstr(15*MENU_CHAR_W,5,"      ");
    ips200_showfloat(15*MENU_CHAR_W,5,mmpid[2],3,1);
}

void Pidui_Refresh3(void)
{
    ips200_showstr(15*MENU_CHAR_W,6,"      ");
    ips200_showint16(15*MENU_CHAR_W,6,mtar);
}

void Pidui_Refresh4(void)
{
    ips200_showstr(15*MENU_CHAR_W,7,"      ");
    ips200_showfloat(15*MENU_CHAR_W,7,sspid[0],3,1);
}

void Pidui_Refresh5(void)
{
    ips200_showstr(15*MENU_CHAR_W,8,"      ");
    ips200_showfloat(15*MENU_CHAR_W,8,sspid[1],3,1);
}

void Pidui_Refresh6(void)
{
    ips200_showstr(15*MENU_CHAR_W,9,"      ");
    ips200_showfloat(15*MENU_CHAR_W,9,sspid[2],3,1);
}

void Pidui_Handle(void)
{
    uint16 choice,clast;
    uint32 write_buf;
    Pidui_Init();
    Pidui_Draw();
    clast=choice=0;
    ips200_showstr(9*MENU_CHAR_W,3+choice,">");
    while(1)
    {
        Key_Scan(&ki);
        systick_delay_ms(STM1,10);
        if(ki.key_now[KEY_UP])
        {
            switch(choice)
            {
                case 0:
                    mmpid[0]+=0.1;
                    Pidui_Refresh0();
                    break;
                case 1:
                    mmpid[1]+=0.1;
                    Pidui_Refresh1();
                    break;
                case 2:
                    mmpid[2]+=0.1;
                    Pidui_Refresh2();
                    break;
                case 3:
                    mtar+=1;
                    Pidui_Refresh3();
                    break;
                case 4:
                    sspid[0]+=0.1;
                    Pidui_Refresh4();
                    break;
                case 5:
                    sspid[1]+=0.1;
                    Pidui_Refresh5();
                    break;
                case 6:
                    sspid[2]+=0.1;
                    Pidui_Refresh6();
                    break;
            }
        }
        else if(ki.key_now[KEY_DOWN])
        {
            switch(choice)
            {
                case 0:
                    mmpid[0]-=0.1;
                    Pidui_Refresh0();
                    break;
                case 1:
                    mmpid[1]-=0.1;
                    Pidui_Refresh1();
                    break;
                case 2:
                    mmpid[2]-=0.1;
                    Pidui_Refresh2();
                    break;
                case 3:
                    mtar-=1;
                    Pidui_Refresh3();
                    break;
                case 4:
                    sspid[0]-=0.1;
                    Pidui_Refresh4();
                    break;
                case 5:
                    sspid[1]-=0.1;
                    Pidui_Refresh5();
                    break;
                case 6:
                    sspid[2]-=0.1;
                    Pidui_Refresh6();
                    break;
            }
        }
        else if(ki.key_now[KEY_ENTER]&&(!ki.key_last[KEY_ENTER]))
        {
            clast=choice;
            choice++;
            if(choice>=7)
                choice=0;
            ips200_showstr(9*MENU_CHAR_W,3+clast," ");
            ips200_showstr(9*MENU_CHAR_W,3+choice,">");
        }
        else if(ki.key_now[KEY_CANCEL]&&(!ki.key_last[KEY_CANCEL]))
            break;
    }
    if(mmpid[0]!=mpid.m_p
     ||mmpid[1]!=mpid.m_i
     ||mmpid[2]!=mpid.m_d
     ||mtar!=mpid.spd_tar
     ||sspid[0]!=spid.s_p
     ||sspid[1]!=spid.s_i
     ||sspid[2]!=spid.s_d)
    {
        if(flash_check(SCT_PID,PGE_MP)
         ||flash_check(SCT_PID,PGE_MI)
         ||flash_check(SCT_PID,PGE_MD)
         ||flash_check(SCT_PID,PGE_MT)
         ||flash_check(SCT_PID,PGE_SP)
         ||flash_check(SCT_PID,PGE_SI)
         ||flash_check(SCT_PID,PGE_SD))
            eeprom_erase_sector(SCT_PID);
        if(mmpid[0]!=mpid.m_p)
            mpid.m_p=mmpid[0];
        if(mmpid[1]!=mpid.m_i)
            mpid.m_i=mmpid[1];
        if(mmpid[2]!=mpid.m_d)
            mpid.m_d=mmpid[2];
        if(mtar!=mpid.spd_tar)
            mpid.spd_tar=mtar;
        if(sspid[0]!=spid.s_p)
            spid.s_p=sspid[0];
        if(sspid[1]!=spid.s_i)
            spid.s_i=sspid[1];
        if(sspid[2]!=spid.s_d)
            spid.s_d=sspid[2];
        write_buf=float_conversion_uint32(mmpid[0]);
        eeprom_page_program(SCT_PID,PGE_MP,&write_buf);
        write_buf=float_conversion_uint32(mmpid[1]);
        eeprom_page_program(SCT_PID,PGE_MI,&write_buf);
        write_buf=float_conversion_uint32(mmpid[2]);
        eeprom_page_program(SCT_PID,PGE_MD,&write_buf);
        write_buf=mtar;
        eeprom_page_program(SCT_PID,PGE_MT,&write_buf);
        write_buf=float_conversion_uint32(sspid[0]);
        eeprom_page_program(SCT_PID,PGE_SP,&write_buf);
        write_buf=float_conversion_uint32(sspid[1]);
        eeprom_page_program(SCT_PID,PGE_SI,&write_buf);
        write_buf=float_conversion_uint32(sspid[2]);
        eeprom_page_program(SCT_PID,PGE_SD,&write_buf);
    }
}
