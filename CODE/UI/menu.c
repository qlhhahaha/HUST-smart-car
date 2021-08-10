/*
 * Dream-Seekers-menu.c
 *
 *  Created on:
 *      Author:
 *     Version: V1.0
 *        Core: TC264D
 *
 *   	  Name: �༶�˵�����
 *		 Brief:
 *	      Note:
 */

#include "headfile.h"
#include "Font.h"

/*����һЩ�˵���ز���������д��һ��Ҫ�������Ӧ����ȻUI�����*/
#define MENULEVEL 		4      	//��ǰ�˵������
#define OPTIONMETE 		28 		//��ǰѡ�����������һ���ر���ѡ��������ѡ�������ټ�һ��///27
#define MaxDisplayNum	8		//��ҳ����ʾ�����˵���Ŀ(һ��С�ڵ��������������ѡ����Ŀ)

/********************** �������� ****************************/
const unsigned char Level[MENULEVEL][3] =
{
	/*��ʼ����  ��������  ѡ����Ŀ*/
	{0,		7,		8},
	{8,		14,		7},
	{15,	23,		9},
	{24,	26,		3},
};

/********************** �˵�ѡ��� ****************************/
struct Option sOption[OPTIONMETE] =
{
	/*�˵����    ��������    �˳�����    �˵���ʾ����   ��ת����ָ��*/
	{0, 8             , 0, "1.Running mode", 	&NoThisFunction},          	//0		��������ģʽ
	{0, 15            , 1, "2.Test status", 	&NoThisFunction},          	//1		����ģʽ
	{0, OPTIONMETE - 1, 2, "----------------", 	&NoThisFunction},        	//2
	{0, OPTIONMETE - 1, 3, "----------------", 	&NoThisFunction},        	//3
	{0, OPTIONMETE - 1, 3, "----------------", 	&NoThisFunction},        	//4
	{0, OPTIONMETE - 1, 3, "----------------", 	&NoThisFunction},        	//5
	{0, OPTIONMETE - 1, 3, "----------------", 	&NoThisFunction},        	//6
	{0, OPTIONMETE - 1, 3, "----------------", 	&NoThisFunction},        	//7

	//{1, OPTIONMETE - 1, 0, "1.Magnetic Track",  &Running},  //Magnetic Track //8		����
	//{1, OPTIONMETE - 1, 0, "2.Driver-2", 		&Running},         			//9		����
	{1, OPTIONMETE - 1, 0, "1.ADCRUN",          &ADCRUN},
	{1, OPTIONMETE - 1, 0, "2.Magnetic Track",  &Running},  //Magnetic Track //8        ����
	{1, OPTIONMETE - 1, 0, "3.Driver-3", 		&Running},          		//10	����
	{1, OPTIONMETE - 1, 0, "4.Driver-4", 		&Running},          		//11	����
	{1, OPTIONMETE - 1, 0, "5.Driver-5", 		&Running},          		//12	����
	{1, OPTIONMETE - 1, 0, "6.Driver-6", 		&Running},          		//13	����
	{1, OPTIONMETE - 1, 0, "7.Driver-1", 	    &Running},     				//14	���ѭ��                ///���ܲ���

	{2, OPTIONMETE - 1, 1, "1.Test motor", 		&TestMotor},             	//15	�������
	{2, OPTIONMETE - 1, 1, "2.Test servo", 		&TestServo},             	//16	���Զ��
	{2, OPTIONMETE - 1, 1, "3.Test ADC", 		&TestADC},			 		//17	����ADC�ɼ�
	{2, OPTIONMETE - 1, 1, "4.Test camera", 	&TestCamera},         		//18 	��������ͷ///TestCamera
	{2, OPTIONMETE - 1, 1, "5.Test encoder", 	&TestEncoder},         		//19 	���Ա�����
	{2, OPTIONMETE - 1, 1, "6.Test WIFI", 		&TestWIFI},         		//20 	����WIFI          ///���ܲ���
	{2, OPTIONMETE - 1, 1, "7.Test ultrasonic", &Test_Ultrasonic},         	//21 	���Գ��������        ///���ܲ���
	{2, OPTIONMETE - 1, 1, "8.Test Gyroscope", 	&Test_Gyroscope},         	//22 	����������
	{2, OPTIONMETE - 1, 1, "9.Test Anything",   &Test_Gyroscope},            //23    �����κ���Ŀ


	{3, OPTIONMETE - 1, 8, "1.Test L_Motor", 	&NoThisFunction},          	//24 	������·���
	{3, OPTIONMETE - 1, 8, "2.Test R_Motor", 	&NoThisFunction},          	//25 	������·���
	{3, OPTIONMETE - 1, 8, "3.Test two_motor", 	&NoThisFunction},        	//26 	����˫·���            ///ֻ��һ·���
};

//-------------------------------------------------------------------------------------------------------------------
//  @name    	FunctionAction
//  @brief   	���幦��ɢת����
//  @param  	switch_code: ִ�й��ܴ���
//  @return     unsigned char����Ч���������ͷ�ļ�����ĺ���ָ�룩
//  @since      v1.0
//  @note       �ڲ�����
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
unsigned char FunctionAction (int switch_code)
{
	struct Option NowSelect = sOption[switch_code];

	SCLEARALL();										//����
	TimeDly(10);										//��ʱ
	SREFRESH();											//д��GRAM
	//������庯��
	NowSelect.func(switch_code);
	return 0;
}

/*�˵���������*/
unsigned char FirstLineDisIndex = 0;    //��Ļ��һ����ʾ��������
unsigned char SelectLine_L = 1;		    //ԭ��ѡ����
unsigned char SelectLine = 1;      		//��ǰѡ�����
unsigned char SelectIndex = 0;     		//��ǰѡ���ж�Ӧ��������
unsigned char LastIndex = 0;      		//���빦�ܺ���ǰ�������ţ��жϾ��幦��ʹ��
//-------------------------------------------------------------------------------------------------------------------
//  @name    	Select_Line
//  @brief   	ѡ���з�����ʾ
//  @param  	line: ѡ����
//  @param  	clear: 1-ѡ����������� �� 0-ѡ���з�����ʾ
//  @return     void
//  @since      v1.0
//  @note       �ڲ�����
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void Show_Select_Line(unsigned char line, unsigned char clear)
{
	if(line<=MaxDisplayNum && line>=0)
	{
		SCLEAR(line);				//���ԭ������ʾ
		SSHOW(FirstLineDisIndex,line,clear);		//��ʾ�µĲ˵�
	}
	SREFRESH();					//ˢ��
	TimeDly(10);
}

//-------------------------------------------------------------------------------------------------------------------
//  @name    	Display_Menu
//  @brief   	ˢ�²˵�����
//  @param  	mode:��ʾģʽ ����0������������1����������2����������
//				����������������˳�ĳ���˵�ʱ�豻����
//				������������������������ʾ��Ŀ�����û��ƶ�������ʾ����ʱ������
//				����������������������ʱ������
//				ͨ����������ʽ�ĸı���Լ�����ʾ�����ٲ���ʱ�䣬ʹUI���и�����
//  @return     void
//  @since      v1.0
//  @note       �ڲ�����
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void Display_Menu (unsigned char mode)
{
	unsigned char ii=0,LineMete = Level[sOption[SelectIndex].KeyLevel][2]; 	//ѭ����,��ʾ����
	if(!mode)															//����˵��㼶�ı䣬����ȫ����,��������һЩ����
	{
		SCLEARALL();							//�����������������
		TimeDly(10);
	}
	if(mode!=2)
	{
		for(ii=1; ii<=MaxDisplayNum; ii++)		//�ֱ���ʾ���в˵���
		{
			if(mode==1)		SCLEAR(ii);			//�����������򵥲�ˢ��ǰ���ԭ������ʾ
			SSHOW(FirstLineDisIndex,ii,1);		//��ʾ�µĲ˵�
			if(--LineMete == 0) break;
		}
	}
	SREFRESH();								//ˢ��(������GRAM��ʾ��ʽ)
	Show_Select_Line(SelectLine_L, 1);		//������һ�з���
	Show_Select_Line(SelectLine, 0);		//������跴��
}

//-------------------------------------------------------------------------------------------------------------------
//  @name		MenuLevelStart
//  @brief		�жϵ�ǰ�����Ƿ��Ǳ���һ��
//  @param		void
//  @return		Yes-1; No-0
//  @since      v1.0
//  @note		�ڲ�����
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
unsigned char MenuLevelStart (void)
{
	unsigned char i = MENULEVEL;
	do
	{
		i--;
		if(SelectIndex == Level[i][0])
			return 1;
	}
	while(i);
	return 0;
}

//-------------------------------------------------------------------------------------------------------------------
//  @name		MenuLevelEnd
//  @brief		�жϵ�ǰ�����Ƿ��Ǳ����һ��
//  @param		void
//  @return		Yes-1; No-0
//  @since      v1.0
//  @note		�ڲ�����
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
unsigned char MenuLevelEnd (void)
{
	unsigned char i = MENULEVEL;
	do
	{
		i--;
		if(SelectIndex == Level[i][1])
			return 1;
	}
	while(i);
	return 0;
}

//-------------------------------------------------------------------------------------------------------------------
//  @name		MenuUpOneOption
//  @brief		���˵�����һ��
//  @param		void
//  @return		void
//  @since      v1.0
//  @note		�ڲ�����
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void MenuUpOneOption (void)
{
	if(MenuLevelStart ())   //�����ǰΪ����һ��
	{
		if(Level[sOption[SelectIndex].KeyLevel][2] >= MaxDisplayNum) //���ұ���ѡ����Ŀ���ڵ���MaxDisplayNum
		{
			FirstLineDisIndex = Level[sOption[SelectIndex].KeyLevel][1] - MaxDisplayNum + 1; 	//��һ����ʾ������Ϊ������MaxDisplayNum-1
			SelectIndex = Level[sOption[SelectIndex].KeyLevel][1];   							//ѡ������Ϊ�����һ��
			SelectLine_L = SelectLine;															//��¼ԭ����
			SelectLine = MaxDisplayNum;             											//���ѡ����Ϊ������
			if(Level[sOption[SelectIndex].KeyLevel][2] > MaxDisplayNum) //���ұ���ѡ����Ŀ���ڵ���MaxDisplayNum
				Display_Menu(1);    																//�����ģʽˢ����Ļ��ʾ
			else
				Display_Menu(2);    																//�������ģʽˢ����Ļ��ʾ
		}
		else           				 								//���ѡ����Ŀ��������MaxDisplayNum��
		{
			SelectIndex = Level[sOption[SelectIndex].KeyLevel][1];   							//ѡ������Ϊ��ǰ�����һ��
			SelectLine_L = SelectLine;															//��¼ԭ����
			SelectLine = Level[sOption[SelectIndex].KeyLevel][2];   							//��ʾ�б���Ŀ(���һ��)
			Display_Menu(2);    																//�������ģʽˢ����Ļ��ʾ
		}
	}
	else        //�����ǰ���ǿ�ʼ����
	{
		if(SelectLine == 1)  //�����Ѿ�����Ļ���ϱ�һ��
		{
			FirstLineDisIndex--;  //��ʾ��������
			SelectIndex--;    //ѡ�������Լ�
			SelectLine_L = SelectLine;	//��¼ԭ����
			SelectLine = 1;    //ѡ���л��ǵ�һ��
			Display_Menu(1);    																//�����ģʽˢ����Ļ��ʾ
		}
		else       //������ǵ�һ��
		{
			SelectLine_L = SelectLine;	//��¼ԭ����
			SelectLine--;    //ѡ�����Լ�
			SelectIndex--;    //ѡ�������Լ�
			Display_Menu(2);    																//�������ģʽˢ����Ļ��ʾ
		}
	}
}

//-------------------------------------------------------------------------------------------------------------------
//  @name		MenuDownOneOption
//  @brief		���˵�����һ��
//  @param		void
//  @return		void
//  @since      v1.0
//  @note		�ڲ�����
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void MenuDownOneOption (void)
{
	if(MenuLevelEnd ())        //�����ǰ�Ǳ����һ������
	{
		FirstLineDisIndex = Level[sOption[SelectIndex].KeyLevel][0]; //��һ����ʾ����Ϊ����һ��ѡ��
		SelectIndex = Level[sOption[SelectIndex].KeyLevel][0];   //ѡ������Ϊ����һ��ѡ������
		SelectLine_L = SelectLine;	//��¼ԭ����
		SelectLine = 1;             //ѡ����Ϊ��һ��
		if(Level[sOption[SelectIndex].KeyLevel][2] > MaxDisplayNum) //���ұ���ѡ����Ŀ���ڵ���MaxDisplayNum
			Display_Menu(1);    																//�����ģʽˢ����Ļ��ʾ
		else
			Display_Menu(2);    																//�������ģʽˢ����Ļ��ʾ
	}
	else           //���������������
	{
		if(SelectLine != MaxDisplayNum)     //�����ǰ������Ļ�����
		{
			SelectIndex++;       //ѡ�������Լ�
			SelectLine_L = SelectLine;	//��¼ԭ����
			SelectLine++;       //ѡ��������
			Display_Menu(2);    																//�������ģʽˢ����Ļ��ʾ
		}
		else          //�������Ļ�����
		{
			FirstLineDisIndex++;     //��һ����ʾ����
			SelectIndex++;       //ѡ�������Լ�
			Display_Menu(1);    																//�����ģʽˢ����Ļ��ʾ
		}
	}
}

//-------------------------------------------------------------------------------------------------------------------
//  @name   	MenuEnterOption
//  @brief   	������һ���˵�����
//  @param  	void
//  @return     void
//  @since      v1.0
//  @note      	�ڲ�����
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void MenuEnterOption (void)
{
	LastIndex = SelectIndex;           //��ǽ���ǰ�������ţ��Ա��жϾ��幦�ܣ�
	SelectIndex = sOption[SelectIndex].EnterIndex;      //����ѡ������Ϊ֮ǰ�����Ŷ�Ӧ��������
	if(SelectIndex != OPTIONMETE - 1)        //�����ǰ�������ǹ���ѡ������
	{
		FirstLineDisIndex = Level[sOption[SelectIndex].KeyLevel][0]; //��һ����ʾΪ�������һ��
		SelectLine_L = SelectLine;
		SelectLine = 1;             	//�趨��һ��Ϊѡ����
		Display_Menu (0);            	//��ȫģʽˢ�²˵�
	}
	else
	{
		FunctionAction (LastIndex);           //����ǹ���ѡ������빦�ܷ�֧�жϺ���
	}
}

//-------------------------------------------------------------------------------------------------------------------
//  @name    	MenuCancelOption
//  @brief   	�ص�ĸһ���˵�
//  @param  	void
//  @return     void
//  @since      v1.0
//  @note      	�ڲ�����
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void MenuCancelOption (void)
{
	if(SelectIndex != OPTIONMETE - 1)        //������Ǵӹ��ܷ���
	{
		SelectIndex = sOption[SelectIndex].CancelIndex;     //ѡ������Ϊѡ�������
	}
	else                //����Ǵӹ��ܷ���
	{
		SelectIndex = LastIndex;          //�������ڽ���ǰ��������
	}
	if(Level[sOption[SelectIndex].KeyLevel][2] >= MaxDisplayNum)    //������ر�ѡ����Ŀ����MaxDisplayNum��
	{
		if(SelectIndex > Level[sOption[SelectIndex].KeyLevel][1] - MaxDisplayNum+1) //���ݷ���ѡ��ȷ����ʾ����
		{
			FirstLineDisIndex = Level[sOption[SelectIndex].KeyLevel][1] - MaxDisplayNum+1;
			SelectLine_L = SelectLine;
			SelectLine =  MaxDisplayNum - (Level[sOption[SelectIndex].KeyLevel][1] - SelectIndex);
		}
		else               //һ����ʾ��ʽ
		{
			FirstLineDisIndex = SelectIndex;       //��һ����ʾ����
			SelectLine_L = SelectLine;
			SelectLine = 1;            //ѡ���һ��
		}
	}
	else                //������ر�ѡ����Ŀ����4��
	{
		FirstLineDisIndex = Level[sOption[SelectIndex].KeyLevel][0];   //��һ����ʾ����Ϊ����һ��
		SelectLine_L = SelectLine;
		SelectLine = SelectIndex -  Level[sOption[SelectIndex].KeyLevel][0] + 1; //ѡ���б�־Ϊ��ǰѡ��������Ӧ��
	}
	Display_Menu (0);               //��ȫģʽˢ�²˵�
}

//-------------------------------------------------------------------------------------------------------------------
//  @name    	NoThisFunction
//  @brief   	�޹���ӳ�䴦��
//  @param  	void
//  @return     void
//  @since      v1.0
//  @note      	�ڲ�����
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void NoThisFunction(void)
{
	SSHOWSTR(40, 16, "Sorry!");
	SSHOWSTR(0, 40, "Undeveloped function");
	SREFRESH();
}

//-------------------------------------------------------------------------------------------------------------------
//  @name    	KeyCodeAction
//  @brief   	��������ɢת
//  @param  	KeyCode - ��Ӧ��ֵ
//  @return     void
//  @since      v1.0
//  @note
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void KeyCodeAction (unsigned char KeyCode)
{
	switch (KeyCode)
	{
#ifdef 	MY_KEY_UP
		case MY_KEY_UP:
			MenuUpOneOption();
			break;
#endif
#ifdef	MY_KEY_DOWN
		case MY_KEY_DOWN:
			MenuDownOneOption();
			break;
#endif
		case MY_KEY_ENTER:
			MenuEnterOption();
			break;
		case MY_KEY_CANCLE:
			MenuCancelOption();
			break;
		default:
			break;
	}
}

//-------------------------------------------------------------------------------------------------------------------
//  @name   	MenuReflash
//  @brief   	�˵�ˢ�º���
//  @param  	void
//  @return    	void
//  @since      v1.0
//  @note
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void MenuReflash (void)
{
	static unsigned char s=0;
	if(!s)
	{
		s=1;
		SINIT();
		SKEYINIT();
		Display_Menu(0);
	};		//��һ�ν��루�����Ҫ��������Ļ�Ͱ����������������
	unsigned char Keytemp;       //���水���������
	Keytemp=SKEYSCAN();
	if(Keytemp)          //�������������Ч
	{
		KeyCodeAction (Keytemp);     //���ݰ���˵����ĸɢת
	}
}


