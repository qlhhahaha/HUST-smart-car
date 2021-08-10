/*
 * Dream-Seekers-menu.h
 * 
 *  Created on:
 *      Author:
 *     Version: V1.0
 *        Core: TC264D
 *
 *   	  Name: �༶�˵���ʾ����
 *		 Brief: ��ʾ�༶�˵�
 *	      Note: V2.0
 */

#ifndef _MENU_H_
#define _MENU_H_

/************************************* UI�����ӿڵ����� *************************************/
//ѡ��ʹ�õ� LCD
#ifndef USE_SCREEN
	#define OLED			1
	#define VCANLCD			2
	#define IPS200			3

	#define USE_SCREEN      IPS200
#endif

//���ö�Ӧ���ܰ���(��������Ϊ���������ļ�ģʽ��������Ϊ����ģʽʱ�����¼���ѡ��һ����һ��ע�͵�����������)
#include "key.h"							//��Ӷ�Ӧͷ�ļ�
#define MY_KEY_UP			KEY0			//��
#define MY_KEY_DOWN			KEY1//KEY3			//��
#define MY_KEY_ENTER		KEY2			//����
#define MY_KEY_CANCLE		KEY3//KEY1			//�˳�

//���ð����ӿ�(ע�⣬������ʼ�������UI����ǰ���ã��ú�ֱ�ӿն��弴��)
#define GET_KEYCODE()		KEY_Scan()		//����ɨ�躯�������뷵��Ϊ����Ĺ��ܰ���ֵ��
#define SET_KEYINIT()		KeyInit()		//������ʼ������

//����ϵͳ��ʱ����
#include "zf_stm_systick.h"					//��Ӷ�Ӧͷ�ļ�
#define TimeDly(ms)			delay_ms(ms)	//��ʱ��ms)


/************************************* UI��������� *************************************/
//���������С(Ĭ���ֿ���֧��3216/3015/2814/2613/2412/2211/2010/1809/1608/1407/1206����)
//������������������С�����豣֤����ȣ���ȡģ��ʽΪ���� ����ʽ ����
#define FontH				26						//�����
#define FontW				13						//�����
#define FontTable			asc_2613				//���������
//���ò˵���ʾ��ɫ��OLED����Ҫ���ã�
#define UI_ForeColor		WHITE					//�˵�ǰ��ɫ��������ɫ��
#define UI_BKColor			BLACK					//�˵�����ɫ


/****************************** ��Ӧ��Ļ����������������޸ģ��û������������ ************************************/
#if USE_SCREEN == IPS200
#include "SEEKFREE_IPS200_PARALLEL8.h"
	#define INIT_SCREEN()			ips200_init()
	#define	CLEAR_SCREEN(N)			do{\
										int p=0,q=FontH;\
										ips200_address_set(0,(N-1)*FontH,IPS200_X_MAX-1,N*FontH-1);\
										while(q--){\
											for(p=0;p<IPS200_X_MAX;p++){\
												ips200_wr_data16(UI_BKColor);}\
										}\
									   }while(0);
	#define CLEAR_SCREEN_ALL()		do{\
										int i,j;\
										ips200_address_set(0,0,IPS200_X_MAX-1,IPS200_Y_MAX-1);\
										for(i=0;i<IPS200_X_MAX;i++){\
											for (j=0;j<IPS200_Y_MAX;j++){\
												ips200_wr_data16(UI_BKColor);}}\
									   }while(0);
	#define FontWidth				(FontW/8+((FontW%8)?1:0))
	#define SHOW_CHAR(x0,y0,chr,clear)	do{\
										int i,j,temp;\
										for(i=0; i<FontWidth*FontH; i++){\
											ips200_address_set(x0+i%FontWidth*8,y0+i/FontWidth,x0+i%FontWidth*8+7,y0+i/FontWidth);\
											temp = FontTable[(uint16)chr-32][i];\
											for(j=0; j<8; j++){\
												if(clear)	{if(temp&0x01)	ips200_wr_data16(UI_ForeColor);\
															 else			ips200_wr_data16(UI_BKColor);}\
												else		{if(temp&0x01)	ips200_wr_data16(UI_BKColor);\
															 else			ips200_wr_data16(UI_ForeColor);}\
												temp>>=1;}}\
									   }while(0);
	#define SHOW_LIST(FN,N,clear)	do{\
										int k=0;\
										char *dat = sOption[FN+N-1].KeyWord;\
										while(dat[k] != '\0'){\
											SHOW_CHAR(k*FontW,(N-1)*FontH,dat[k],clear);\
											k++;}\
								   	   }while(0);
	#define	SHOW_STR(X,Y,str)		do{\
										int k=0,kx=0,ky=0;\
										char *dat = str;\
										while(dat[k] != '\0'){\
											if(X+(k-kx)*FontW>(IPS200_X_MAX-(FontWidth*8))) {kx=k;ky++;}\
        									if(Y+ky*FontH>(IPS200_Y_MAX-FontH)){break;}\
											SHOW_CHAR(X+(k-kx)*FontW,Y+ky*FontH,dat[k],1);\
											k++;}\
								   	   }while(0);
	#define	SCREEN_Refresh()
#endif

/*********************** MENU�ӿ�(��������Զ���Ӧ)�����û�������� ******************************/
#define SINIT()				INIT_SCREEN()		//��Ļ��ʼ��
#define SCLEAR(N)			CLEAR_SCREEN(N)		//�����Ӧ�˵���
#define SCLEARALL()			CLEAR_SCREEN_ALL()	//����˵���ʾ��
#define SSHOWSTR(X,Y,str)	SHOW_STR(X,Y,str)	//��ʾ�ַ�
#define SSHOW(FN,N,clear)	SHOW_LIST(FN,N,clear)	//��ʾ��Ӧ�˵���
#define SREFRESH()			SCREEN_Refresh()	//ˢ��
#define SKEYSCAN()			GET_KEYCODE()		//��ð���ֵ
#define SKEYINIT()			SET_KEYINIT()		//��ʼ������

/*�˵��ṹ����*/
struct Option
{
    unsigned char KeyLevel;       	//�˵�ѡ�������˵����
    unsigned char EnterIndex;     	//ѡ�����������
    unsigned char CancelIndex;    	//ѡ���˳�������
    char *KeyWord;       			//�˵�ѡ����������ָ��
    void (*func)(int x);	  		//��ת����ָ�루������������ѭָ��淶��
};

/*��������*/
void Display_Menu (unsigned char mode);	//ˢ�²˵����ܺ���
void MenuUpOneOption (void);			//�˵�����һ���
void MenuDownOneOption (void);			//�˵�����һ���
void MenuCancelOption(void);			//�˵��˳����ܺ���
void MenuReflash (void);				//�˵����º���
void NoThisFunction(void);				//����ִ�еĺ�����ʾ

#endif

