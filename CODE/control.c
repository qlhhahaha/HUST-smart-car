
#include "headfile.h"
#include "servo.h"
#include "Motor.h"
#include "encoder.h"

/************PID参数**************/
PID ServoPID = {0,0,0,3.5,0,10};	///{0,0,0,  2.72,       0,      8.00}

//0.256 0.66   0.306  0.68   1.697 7.11  1.79 7.87  1.71 7.82  1.56 7.71  1.48 6.42
///PID LeftMotorPID = 	{0,0,0,	58,		12,		0};		//48.9 12.7  40 10
PID RightMotorPID = {0,0,0,58,12,0};
/**********电机速度参数**********/
const GearSet GearCtl={
//	档位		    最大速度	   最小速度 	 提前系数	   差速系数	倒转标志
  .Defaults = 	{0},
  .Driver1 	= 	{380,		310,		4,			1.10,		0},	//档位1
  .Driver2 	= 	{360,		290,		4,			1.00,		0},	//档位2
  .Driver3 	= 	{340,		270,		2,			0.9,		0},	//档位3
  .Driver4 	= 	{320,		250,		2,			0.85,		0},	//档位4
  .Driver5 	= 	{300,		230,		2,			0.75,		0},	//档位5
  .Driver6 	= 	{120,		100 ,		2,			0.55,		0},	//档位6
  .Lose	   	= 	{80,		60,			2,			0.7,		1},	//断路速度
  .Magnetic	= 	{90,		40,			2,			0.4,		0},	//电磁速度
  .Circle 	=	{150,		100,		2,			0.90,		0},	//环岛速度
  .Block1	= 	{100,		40,			2,			0.99,		1},	//横断速度1
  .Block2	= 	{100,		80,			2,			0.65,		1},	//横断速度2
  .Zebra	= 	{300 ,		-300 ,		2,			2,			1},	//坡道速度
  .Ramp		= 	{20,		0,			2,			2,		    1},	//坡道速度
  .Stop 	= 	{1,			0,			0,			0,			1},	//停车
  .Reverse 	= 	{0,			0,			0,			0,			1},	//制动停车
};

ControlStruct SpeedParm,SelectMode;							//当前速度系数

/************其它参数**************/
u8 OpenParSetFlag = 0;                                		//PID调参标志位
u8 OpenSpeedSetFlag = 0;									//动态速度调参标志位
u8 ExitFlag = 0;											//运行退出标志位
u8 PIDWayFlag = 0;											//
u8 MagneticFlag=0;										//电磁循迹标志
u8 LoseRoadFlag	 = 0;										//断路标志
u8 RampFlag=0;												//坡道标志
float Ramp_Distance=0.0;
int32  cb_sum_error;                                       //差比和
int32 Ratio_add=30;
int32 Ratio_cut=10;

u16 ReceiveCode = 0;
int32 Distance=0,TimeCount=0;

u8 CircleDirFlag=0;											//环岛指定flag

u8 RoadBlockFlag=0;											//横断标志
u8 RoadPassDirFlag=0;										//横断通过方向标志
float Block_Distance=0.0;									//横断路程
float Block_Angle=0.0;										//横断角度

extern float LSpeedSet;
extern float RSpeedSet;
extern int16 LeftWheelSpeed;
extern int16 RightWheelSpeed;


extern uint16 LeftResult;
extern uint16 MiddleLResult;
//extern uint16 MiddleRResult;
//extern uint16 RightResult;
extern int16 ServoPWM;

//-------------------------------------------------------------------------------------------------------------------
//  @name    openring
//  @brief
//  @param
//  @return
//  @since      v1.0
//  @note
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
 void openring()
 {
     Motor_Init();
     Motor_Control(2000);
 }
//-------------------------------------------------------------------------------------------------------------------
//  @name    差比和
//  @brief
//  @param
//  @return
//  @since      v1.0
//  @note
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void Difference_over_sum_method()
{
     RunInitalize();
    int duty=0;
    cb_sum_error=(int16)(100*(LeftResult-MiddleLResult))/(LeftResult+MiddleLResult);
    cb_sum_error=(cb_sum_error>=100?100  :cb_sum_error); //差比和限幅
    cb_sum_error=(cb_sum_error<=-100?-100:cb_sum_error); //差比和限幅  0~100

    if(cb_sum_error>0)     //车子偏左  舵机右打角
     {
        duty = SERVO_MidDuty+(int16)(cb_sum_error*Ratio_add);
        pwm_duty(Servo_Module, duty, 1);
     }

    if(cb_sum_error<0)     //车子偏右  舵机左打角
     {
        cb_sum_error=-cb_sum_error;
        duty = SERVO_MidDuty-(int16)(cb_sum_error*Ratio_add);
        pwm_duty(Servo_Module, duty, 1);
     }

    if(cb_sum_error==0)    //在直道 舵机保持中值
     {
        duty = SERVO_MidDuty;
        pwm_duty(Servo_Module, duty, 1);
     }

    //Difference_over_sum_method_display();

}


extern uint8 wifi_buff[40];
void Running(int mode)
{
	switch (mode)              								//档位赋值
	{
	    case 8:     SelectMode = GearCtl.Magnetic;      MagneticFlag=1;  break; //电磁循迹
	    case 9:     SelectMode = GearCtl.Driver1;       break; //case 8
		////case 8:		SelectMode = GearCtl.Driver1;		break;
		////case 9:		SelectMode = GearCtl.Driver2;		break;
		case 10:	SelectMode = GearCtl.Driver3;		break;
		case 11:	SelectMode = GearCtl.Driver4;		break;
		case 12:	SelectMode = GearCtl.Driver5;		break;
		case 13:	SelectMode = GearCtl.Driver6;		break;
		////case 14:	SelectMode = GearCtl.Magnetic;		MagneticFlag=1;//电磁循迹
	}
#if OpenStartCheck!=1
	SpeedParm = SelectMode;
#endif
	RunInitalize();                                               //执行相关初始化
	while(1)
	{
//
//		extern uint16 NormalizeOfAD[5];
//		 char dat[30];
//		sprintf(dat,"Left: %4d",NormalizeOfAD[0]);
//		ips200_showstr(10,6,dat);
//		sprintf(dat,"Middle: %4d",NormalizeOfAD[1]);
//		ips200_showstr(10,8,dat);
//
//		if(GET_KEYCODE()==MY_KEY_UP)
//		{
//			if(flag==1)
//			{
//				num1+=0.1;
//				LeftMotorPID.Proportion=num1;
//				RightMotorPID.Proportion=num1;
//				sprintf(show,"KP : %3f",num1);
//				ips200_showstr(10,2,show);
//
//			}
//			else if(flag==2)
//			{
//				num2+=0.01;
//				LeftMotorPID.Integral=num2;
//				RightMotorPID.Integral=num2;
//				sprintf(show,"KI : %3f",num2);
//				ips200_showstr(10,3,show);
//			}
//			else if(flag==3)
//			{
//				num3+=0.1;
//				LeftMotorPID.Derivative=num3;
//				RightMotorPID.Derivative=num3;
//				sprintf(show,"KD : %3f",num3);
//				ips200_showstr(10,4,show);
//			}
//		}
//		else if(GET_KEYCODE()==MY_KEY_DOWN)
//		{
//			if(flag==1)
//			{
//				num1-=0.1;
//				LeftMotorPID.Proportion=num1;
//				RightMotorPID.Proportion=num1;
//				sprintf(show,"KP : %3f",num1);
//				ips200_showstr(10,2,show);
//
//			}
//			else if(flag==2)
//			{
//				num2-=0.01;
//				LeftMotorPID.Integral=num2;
//				RightMotorPID.Integral=num2;
//				sprintf(show,"KI : %3f",num2);
//				ips200_showstr(10,3,show);
//			}
//			else if(flag==3)
//			{
//				num3-=0.1;
//				LeftMotorPID.Derivative=num3;
//				RightMotorPID.Derivative=num3;
//				sprintf(show,"KD : %3f",num3);
//				ips200_showstr(10,4,show);
//			}
//		}
//		else if(GET_KEYCODE()==MY_KEY_ENTER)
//		{
//			if(flag==1)	flag=2;
//			else if(flag==2)	flag=3;
//			else if(flag==3)	flag=1;
//		}
//		else if(GET_KEYCODE()==MY_KEY_CANCLE)
//		{
//			Motor_Control(0, 0);
//			break;
//		}
//
//
//  	OledModeSelect();                                       //OLED显示设置
//	if(ExitFlag)
//	{
//		disable_irq(PIT1_IRQn);
//		if(!MagneticFlag)
//		{
//		  disable_irq(MT9V032_INTERRUPT_NUNBERS);
//		  disable_irq(DMA0_DMA16_IRQn);
//		}
//		else		MagneticFlag = 0;
//		StartFlag = WithoutRun;
//		LeftCircleFlag = NoCircle;
//		RightCircleFlag = NoCircle;
//		RoadBlockFlag = 0;
//		RampFlag = 0;
//		LoseRoadFlag = 0;
//
//		run_Motor(0,0);                                       //电机停转
//		Servo_Duty(Servo_Middle);                             //舵机归中
//		BuzzerReverse(3,50);                                  //蜂鸣器响3声
//		char txt[10];
//		/*sprintf((char*)txt,"%d",Distance);
//		LCD_P6x8Str(10,2,txt);
//		sprintf((char*)txt,"%d",TimeCount);
//		LCD_P6x8Str(10,3,txt);*/
//		ClearIncResult();                                     //清除电机增量式计算结果
//		BeepFlag = 0;
//		BuzzerOff();
//		ExitFlag = 0;
//		break;
//	}
	}
}


u8 Circleflag = 0;
u32 flag_10ms = 0;
void PITOnTimeHandle(void)//中断函数
{
	enableInterrupts();
	openring();
	//Difference_over_sum_method();
	 pwm_duty(Servo_Module, SERVO_MaxDuty, 1);

	ExitRunning();
	if(AbortedRoute())	ExitFlag = 1;

//	/********************************闭环控制**********************************/
	flag_10ms++;
}

void DMAOnTimeHandle(void)
{
	if(SpeedParm.MaxSpeed)
	{
		Camera_scan();										//图像扫描
		ServoControl(AverageCenter);
		MotorRun(ColumnMax/2-(int)AverageCenter);	//电机控制
	}
	mt9v03x_finish_flag= 0;
}

//void LED_TEST(void)
//{
//    LED_Init();
//    gpio_init(P15_0,GPO,0,PUSHPULL);
//while(1)
//{
//    delay_ms(1000);
//    gpio_set(P15_0,1);
//    LED_Ctrl(LEDCORE,RVS);
//    LED_Ctrl(LEDOUT,RVS);
//    delay_ms(1000);
//    gpio_set(P15_0,0);
//}
//}



//
///**
// * @file		主运行代码段
// * @note      	菜单选择后跳到这，没有处理任务的时候程序就呆在下面的循环里。
// * @author
// * @date
// */
//void Running(u8 mode)
//{
//	switch (mode)              								//档位赋值
//	{
//		case 0:		SelectMode = GearCtl.Defaults;		break;
//		case 1:		SelectMode = GearCtl.Driver2;		break;
//		case 2:		SelectMode = GearCtl.Driver4;		break;
//		case 3:		SelectMode = GearCtl.Driver6;		break;
//
//		case 11:	SelectMode = GearCtl.Driver1;		break;
//		case 12:	SelectMode = GearCtl.Driver2;		break;
//		case 13:	SelectMode = GearCtl.Driver3;		break;
//		case 14:	SelectMode = GearCtl.Driver4;		break;
//		case 15:	SelectMode = GearCtl.Driver5;		break;
//		case 16:	SelectMode = GearCtl.Driver6;		break;
//
//		case 5:		SelectMode = GearCtl.Magnetic;		MagneticFlag=1;
//	}
//#if OpenStartCheck!=1
//	SpeedParm = SelectMode;
//#endif
//  RunInitalize();                                               //执行相关初始化
//  while(1)
//  {
//  	OledModeSelect();                                       //OLED显示设置
//	if(ExitFlag)
//	{
//		pit_disable_irq(CCU6_0, PIT_CH1);
//		if(!MagneticFlag)
//		{
//		  disable_irq(MT9V032_INTERRUPT_NUNBERS);
//		  eru_disable_interrupt(DMA0_DMA16_IRQn);
//		}
//		else		MagneticFlag = 0;
//		StartFlag = WithoutRun;
//		LeftCircleFlag = NoCircle;
//		RightCircleFlag = NoCircle;
//		RoadBlockFlag = 0;
//		RampFlag = 0;
//		LoseRoadFlag = 0;
//
//		run_Motor(0,0);                                       //电机停转
//		Servo_Duty(Servo_Middle);                             //舵机归中
//		BuzzerReverse(3,50);                                  //蜂鸣器响3声
//		char txt[10];
//		/*sprintf((char*)txt,"%d",Distance);
//		LCD_P6x8Str(10,2,txt);
//		sprintf((char*)txt,"%d",TimeCount);
//		LCD_P6x8Str(10,3,txt);*/
//		ClearIncResult();                                     //清除电机增量式计算结果
//		BeepFlag = 0;
//		BuzzerOff();
//		ExitFlag = 0;
//		break;
//	}
//	if(BeepFlag == 0)	{BuzzerOff();}
//	if(BeepFlag == 1)	{BuzzerOn();}
//  }
//}
//
///*void Low_running_set(void)
//{
//#if 0
//    RunInitalize();                                               //执行相关初始化
//
//    LCD_Show_Frame100();    //画图像 LCDW*LCDH 外框
//    Low_scan();
//#else
//	SpeedParm = GearCtl.Driver3;						//设置速度档位3
//	Running();
//#endif
//}*/
//
///**
// * @file		运行控制
// * @note      	这段代码内嵌在dma中断（懒得改了，其实没必要，算了就这样吧）
// * @author
// * @date
// */
//void RunningHandle(void)
//{
//    if(LoseRoadFlag==2 || LoseRoadFlag == 3)				//断路开的话舵机放dma控制
//	{
//	  	ADC_Scan();											//电磁扫描
//	  	ADC_ServoControl();									//电磁控制
//	}
//    else if(RoadBlockFlag<2 && !MagneticFlag)
//	{
//	  	Camera_scan();										//图像扫描
//	  	ServoControl(AverageCenter);
//	}
//	Field_Over_Flag= 0;                                             			//场中断标志位清空
//}
//
///**
// * @file		初始化
// * @note      	一大堆的外设初始化-QAQ
// * @author
// * @date
// */
//void RunInitalize(void)
//{
//	Mpu6050Init();													//六轴模块初始化（最先）
//    Servo_Init();                                                 	//舵机频率设置
//    Motor_Init();                                                 	//电机频率设置
//    Encoder_Init();                                               	//编码器初始化
//	if(!MagneticFlag)	camera_init();								//总钻风初始化（纯电磁就不需要）
//    Buzzer_Init();                                                	//蜂鸣器初始化
//	GraySensorInit;													//灰度传感器初始化
//	PIT_Init(PIT1, 10);                                            	//PIT1初始化（10ms）
//	ADC0_Init();													//ADC采集初始化
//	UART_Init(UART_4,115200); 										//蓝牙初始化
//    #if !UploadMode
//  		UART_Irq_En(UART_4);         								//如果命令模式使能中断
//	#endif
//	set_irq_priority(PIT1_IRQn,2);       							//电机控制中断优先级2
//	//set_irq_priority(UART4_RX_TX_IRQn,0);       					//蓝牙控制中断优先级
//
//    LCD_Show_Frame100();                                         	//画图像 LCDW*LCDH 外框
//}
//
/**
 * @file		退出检测
 * @note
 * @author
 * @date
 */
void ExitRunning(void)
{
 //if((!KEY_Read(KEY2))|| ReceiveCode == Stop_mode) 				//退出运行

//ExitFlag = 1;
}


