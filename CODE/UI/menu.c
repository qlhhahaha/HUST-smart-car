/*
 * Dream-Seekers-menu.c
 *
 *  Created on:
 *      Author:
 *     Version: V1.0
 *        Core: TC264D
 *
 *   	  Name: 多级菜单驱动
 *		 Brief:
 *	      Note:
 */

#include "headfile.h"
#include "Font.h"

/*定义一些菜单相关参数（这里写的一定要和下面对应，不然UI会崩）*/
#define MENULEVEL 		4      	//当前菜单表个数
#define OPTIONMETE 		28 		//当前选项个数，包括一个特别功能选项（整个表的选项总数再加一）///27
#define MaxDisplayNum	8		//单页面显示的最大菜单数目(一定小于等于索引表内最大选项数目)

/********************** 索引表定义 ****************************/
const unsigned char Level[MENULEVEL][3] =
{
	/*开始索引  结束索引  选项数目*/
	{0,		7,		8},
	{8,		14,		7},
	{15,	23,		9},
	{24,	26,		3},
};

/********************** 菜单选项定义 ****************************/
struct Option sOption[OPTIONMETE] =
{
	/*菜单表号    进入索引    退出索引    菜单显示文字   跳转函数指针*/
	{0, 8             , 0, "1.Running mode", 	&NoThisFunction},          	//0		正常运行模式
	{0, 15            , 1, "2.Test status", 	&NoThisFunction},          	//1		测试模式
	{0, OPTIONMETE - 1, 2, "----------------", 	&NoThisFunction},        	//2
	{0, OPTIONMETE - 1, 3, "----------------", 	&NoThisFunction},        	//3
	{0, OPTIONMETE - 1, 3, "----------------", 	&NoThisFunction},        	//4
	{0, OPTIONMETE - 1, 3, "----------------", 	&NoThisFunction},        	//5
	{0, OPTIONMETE - 1, 3, "----------------", 	&NoThisFunction},        	//6
	{0, OPTIONMETE - 1, 3, "----------------", 	&NoThisFunction},        	//7

	//{1, OPTIONMETE - 1, 0, "1.Magnetic Track",  &Running},  //Magnetic Track //8		高速
	//{1, OPTIONMETE - 1, 0, "2.Driver-2", 		&Running},         			//9		中速
	{1, OPTIONMETE - 1, 0, "1.ADCRUN",          &ADCRUN},
	{1, OPTIONMETE - 1, 0, "2.Magnetic Track",  &Running},  //Magnetic Track //8        高速
	{1, OPTIONMETE - 1, 0, "3.Driver-3", 		&Running},          		//10	低速
	{1, OPTIONMETE - 1, 0, "4.Driver-4", 		&Running},          		//11	低速
	{1, OPTIONMETE - 1, 0, "5.Driver-5", 		&Running},          		//12	低速
	{1, OPTIONMETE - 1, 0, "6.Driver-6", 		&Running},          		//13	低速
	{1, OPTIONMETE - 1, 0, "7.Driver-1", 	    &Running},     				//14	电磁循迹                ///可能不用

	{2, OPTIONMETE - 1, 1, "1.Test motor", 		&TestMotor},             	//15	测试马达
	{2, OPTIONMETE - 1, 1, "2.Test servo", 		&TestServo},             	//16	测试舵机
	{2, OPTIONMETE - 1, 1, "3.Test ADC", 		&TestADC},			 		//17	测试ADC采集
	{2, OPTIONMETE - 1, 1, "4.Test camera", 	&TestCamera},         		//18 	测试摄像头///TestCamera
	{2, OPTIONMETE - 1, 1, "5.Test encoder", 	&TestEncoder},         		//19 	测试编码器
	{2, OPTIONMETE - 1, 1, "6.Test WIFI", 		&TestWIFI},         		//20 	测试WIFI          ///可能不用
	{2, OPTIONMETE - 1, 1, "7.Test ultrasonic", &Test_Ultrasonic},         	//21 	测试超声波测距        ///可能不用
	{2, OPTIONMETE - 1, 1, "8.Test Gyroscope", 	&Test_Gyroscope},         	//22 	测试陀螺仪
	{2, OPTIONMETE - 1, 1, "9.Test Anything",   &Test_Gyroscope},            //23    测试任何项目


	{3, OPTIONMETE - 1, 8, "1.Test L_Motor", 	&NoThisFunction},          	//24 	测试左路电机
	{3, OPTIONMETE - 1, 8, "2.Test R_Motor", 	&NoThisFunction},          	//25 	测试右路电机
	{3, OPTIONMETE - 1, 8, "3.Test two_motor", 	&NoThisFunction},        	//26 	测试双路电机            ///只测一路电机
};

//-------------------------------------------------------------------------------------------------------------------
//  @name    	FunctionAction
//  @brief   	具体功能散转函数
//  @param  	switch_code: 执行功能代码
//  @return     unsigned char：无效参数（配合头文件定义的函数指针）
//  @since      v1.0
//  @note       内部调用
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
unsigned char FunctionAction (int switch_code)
{
	struct Option NowSelect = sOption[switch_code];

	SCLEARALL();										//清屏
	TimeDly(10);										//延时
	SREFRESH();											//写入GRAM
	//进入具体函数
	NowSelect.func(switch_code);
	return 0;
}

/*菜单索引定义*/
unsigned char FirstLineDisIndex = 0;    //屏幕第一行显示的索引号
unsigned char SelectLine_L = 1;		    //原来选中行
unsigned char SelectLine = 1;      		//当前选择的行
unsigned char SelectIndex = 0;     		//当前选定行对应的索引号
unsigned char LastIndex = 0;      		//进入功能函数前的索引号，判断具体功能使用
//-------------------------------------------------------------------------------------------------------------------
//  @name    	Select_Line
//  @brief   	选中行反白显示
//  @param  	line: 选中行
//  @param  	clear: 1-选中行清除反白 ； 0-选中行反白显示
//  @return     void
//  @since      v1.0
//  @note       内部调用
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void Show_Select_Line(unsigned char line, unsigned char clear)
{
	if(line<=MaxDisplayNum && line>=0)
	{
		SCLEAR(line);				//清除原来的显示
		SSHOW(FirstLineDisIndex,line,clear);		//显示新的菜单
	}
	SREFRESH();					//刷屏
	TimeDly(10);
}

//-------------------------------------------------------------------------------------------------------------------
//  @name    	Display_Menu
//  @brief   	刷新菜单功能
//  @param  	mode:显示模式 ——0：整屏清屏；1：表单清屏；2：单步清屏
//				整屏清屏：进入或退出某级菜单时需被调用
//				表单清屏：当表单数大于最大可显示数目，而用户移动超出显示区域时被调用
//				单步清屏：其它正常翻动时被调用
//				通过对清屏方式的改变可以加速显示，减少操作时间，使UI运行更流畅
//  @return     void
//  @since      v1.0
//  @note       内部调用
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void Display_Menu (unsigned char mode)
{
	unsigned char ii=0,LineMete = Level[sOption[SelectIndex].KeyLevel][2]; 	//循环量,显示行数
	if(!mode)															//如果菜单层级改变，则完全清屏,否则会出现一些乱码
	{
		SCLEARALL();							//整屏清屏在这里完成
		TimeDly(10);
	}
	if(mode!=2)
	{
		for(ii=1; ii<=MaxDisplayNum; ii++)		//分别显示各行菜单项
		{
			if(mode==1)		SCLEAR(ii);			//若表单清屏，则单步刷新前清除原来的显示
			SSHOW(FirstLineDisIndex,ii,1);		//显示新的菜单
			if(--LineMete == 0) break;
		}
	}
	SREFRESH();								//刷屏(适用于GRAM显示方式)
	Show_Select_Line(SelectLine_L, 1);		//消除上一行反白
	Show_Select_Line(SelectLine, 0);		//填充所需反白
}

//-------------------------------------------------------------------------------------------------------------------
//  @name		MenuLevelStart
//  @brief		判断当前索引是否是表单第一项
//  @param		void
//  @return		Yes-1; No-0
//  @since      v1.0
//  @note		内部调用
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
//  @brief		判断当前索引是否是表单最后一项
//  @param		void
//  @return		Yes-1; No-0
//  @since      v1.0
//  @note		内部调用
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
//  @brief		将菜单上移一项
//  @param		void
//  @return		void
//  @since      v1.0
//  @note		内部调用
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void MenuUpOneOption (void)
{
	if(MenuLevelStart ())   //如果当前为表单第一项
	{
		if(Level[sOption[SelectIndex].KeyLevel][2] >= MaxDisplayNum) //并且表单中选项数目大于等于MaxDisplayNum
		{
			FirstLineDisIndex = Level[sOption[SelectIndex].KeyLevel][1] - MaxDisplayNum + 1; 	//第一行显示索引号为倒数第MaxDisplayNum-1
			SelectIndex = Level[sOption[SelectIndex].KeyLevel][1];   							//选择索引为表单最后一项
			SelectLine_L = SelectLine;															//记录原来行
			SelectLine = MaxDisplayNum;             											//标记选择行为第四行
			if(Level[sOption[SelectIndex].KeyLevel][2] > MaxDisplayNum) //并且表单中选项数目大于等于MaxDisplayNum
				Display_Menu(1);    																//表单清除模式刷新屏幕显示
			else
				Display_Menu(2);    																//单步清除模式刷新屏幕显示
		}
		else           				 								//如果选项数目并不大于MaxDisplayNum个
		{
			SelectIndex = Level[sOption[SelectIndex].KeyLevel][1];   							//选择索引为当前表单最后一个
			SelectLine_L = SelectLine;															//记录原来行
			SelectLine = Level[sOption[SelectIndex].KeyLevel][2];   							//显示行表单数目(最后一个)
			Display_Menu(2);    																//单步清除模式刷新屏幕显示
		}
	}
	else        //如果当前不是开始索引
	{
		if(SelectLine == 1)  //并且已经在屏幕最上边一行
		{
			FirstLineDisIndex--;  //显示索引上移
			SelectIndex--;    //选择索引自减
			SelectLine_L = SelectLine;	//记录原来行
			SelectLine = 1;    //选择行还是第一行
			Display_Menu(1);    																//表单清除模式刷新屏幕显示
		}
		else       //如果不是第一行
		{
			SelectLine_L = SelectLine;	//记录原来行
			SelectLine--;    //选择行自减
			SelectIndex--;    //选择索引自减
			Display_Menu(2);    																//单步清除模式刷新屏幕显示
		}
	}
}

//-------------------------------------------------------------------------------------------------------------------
//  @name		MenuDownOneOption
//  @brief		将菜单下移一项
//  @param		void
//  @return		void
//  @since      v1.0
//  @note		内部调用
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void MenuDownOneOption (void)
{
	if(MenuLevelEnd ())        //如果当前是表单最后一个索引
	{
		FirstLineDisIndex = Level[sOption[SelectIndex].KeyLevel][0]; //第一行显示索引为表单第一个选项
		SelectIndex = Level[sOption[SelectIndex].KeyLevel][0];   //选择索引为表单第一个选项索引
		SelectLine_L = SelectLine;	//记录原来行
		SelectLine = 1;             //选择行为第一行
		if(Level[sOption[SelectIndex].KeyLevel][2] > MaxDisplayNum) //并且表单中选项数目大于等于MaxDisplayNum
			Display_Menu(1);    																//表单清除模式刷新屏幕显示
		else
			Display_Menu(2);    																//单步清除模式刷新屏幕显示
	}
	else           //如果不是最后的索引
	{
		if(SelectLine != MaxDisplayNum)     //如果当前不是屏幕最底行
		{
			SelectIndex++;       //选择索引自加
			SelectLine_L = SelectLine;	//记录原来行
			SelectLine++;       //选择行下移
			Display_Menu(2);    																//单步清除模式刷新屏幕显示
		}
		else          //如果是屏幕最低行
		{
			FirstLineDisIndex++;     //第一行显示下移
			SelectIndex++;       //选择索引自加
			Display_Menu(1);    																//表单清除模式刷新屏幕显示
		}
	}
}

//-------------------------------------------------------------------------------------------------------------------
//  @name   	MenuEnterOption
//  @brief   	进入子一级菜单或功能
//  @param  	void
//  @return     void
//  @since      v1.0
//  @note      	内部调用
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void MenuEnterOption (void)
{
	LastIndex = SelectIndex;           //标记进入前的索引号（以便判断具体功能）
	SelectIndex = sOption[SelectIndex].EnterIndex;      //更新选择索引为之前索引号对应进入索引
	if(SelectIndex != OPTIONMETE - 1)        //如果当前索引不是功能选择索引
	{
		FirstLineDisIndex = Level[sOption[SelectIndex].KeyLevel][0]; //第一行显示为进入表单第一项
		SelectLine_L = SelectLine;
		SelectLine = 1;             	//设定第一行为选择行
		Display_Menu (0);            	//完全模式刷新菜单
	}
	else
	{
		FunctionAction (LastIndex);           //如果是功能选择项，进入功能分支判断函数
	}
}

//-------------------------------------------------------------------------------------------------------------------
//  @name    	MenuCancelOption
//  @brief   	回到母一级菜单
//  @param  	void
//  @return     void
//  @since      v1.0
//  @note      	内部调用
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void MenuCancelOption (void)
{
	if(SelectIndex != OPTIONMETE - 1)        //如果不是从功能返回
	{
		SelectIndex = sOption[SelectIndex].CancelIndex;     //选择索引为选项返回索引
	}
	else                //如果是从功能返回
	{
		SelectIndex = LastIndex;          //索引等于进入前保存索引
	}
	if(Level[sOption[SelectIndex].KeyLevel][2] >= MaxDisplayNum)    //如果返回表单选项数目大于MaxDisplayNum个
	{
		if(SelectIndex > Level[sOption[SelectIndex].KeyLevel][1] - MaxDisplayNum+1) //根据返回选项确定显示首项
		{
			FirstLineDisIndex = Level[sOption[SelectIndex].KeyLevel][1] - MaxDisplayNum+1;
			SelectLine_L = SelectLine;
			SelectLine =  MaxDisplayNum - (Level[sOption[SelectIndex].KeyLevel][1] - SelectIndex);
		}
		else               //一般显示方式
		{
			FirstLineDisIndex = SelectIndex;       //第一行显示索引
			SelectLine_L = SelectLine;
			SelectLine = 1;            //选择第一行
		}
	}
	else                //如果返回表单选项数目不足4个
	{
		FirstLineDisIndex = Level[sOption[SelectIndex].KeyLevel][0];   //第一行显示索引为表单第一项
		SelectLine_L = SelectLine;
		SelectLine = SelectIndex -  Level[sOption[SelectIndex].KeyLevel][0] + 1; //选择行标志为当前选择索引对应行
	}
	Display_Menu (0);               //完全模式刷新菜单
}

//-------------------------------------------------------------------------------------------------------------------
//  @name    	NoThisFunction
//  @brief   	无功能映射处理
//  @param  	void
//  @return     void
//  @since      v1.0
//  @note      	内部调用
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
//  @brief   	按键处理散转
//  @param  	KeyCode - 对应键值
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
//  @brief   	菜单刷新函数
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
	};		//第一次进入（如果需要）启动屏幕和按键，并且载入界面
	unsigned char Keytemp;       //保存按键编码变量
	Keytemp=SKEYSCAN();
	if(Keytemp)          //如果按键编码有效
	{
		KeyCodeAction (Keytemp);     //根据按键说明字母散转
	}
}


