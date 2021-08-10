#include "headfile.h"

/************PID参数**************/
//PID ServoPID = {0,0,0,50,3,6};//增量式 PID(倍数为10)
PID ServoPID = {0,0,0,15,0.0,0}; //位置式 PID(倍数为10)
/**********电机速度参数**********/
const GearSet GearCtl={
  .Driver1 	= 	{380 *0.5,		310 *0.5,		4 *0.5,			1.10 *0.5,		0},	//档位1
  .Driver2 	= 	{360 *0.5,		290 *0.5,		4 *0.5,			1.00 *0.5,		0},	//档位2
  .Driver3 	= 	{340 *0.5,		270 *0.5,		2 *0.5,			0.9 *0.5,		0},	//档位3
  .Driver4 	= 	{320 *0.5,		250 *0.5,		2 *0.5,			0.85 *0.5,		0},	//档位4
  .Driver5 	= 	{300 *0.5,		230 *0.5,		2 *0.5,			0.75 *0.5,		0},	//档位5
  .Driver6 	= 	{120 *0.5,		100  *0.5,		2 *0.5,			0.55 *0.5,		0},	//档位6
  .Lose	   = 	{80 *0.5,		60 *0.5,			2 *0.5,			0.7 *0.5,		1},	//断路速度
  .Magnetic	= 	{90 *0.5,		40 *0.5,			2 *0.5,			0.4 *0.5,		0},	//电磁速度
  .Circle 	=	{150 *0.5,		100 *0.5,		2 *0.5,			0.90 *0.5,		0},	//环岛速度
  .Block1	= 	{100 *0.5,		40 *0.5,			2 *0.5,			0.99 *0.5,		1},	//横断速度1
  .Block2	= 	{100 *0.5,		80 *0.5,			2 *0.5,			0.65 *0.5,		1},	//横断速度2
  .Zebra	= 	{300  *0.5,		-300  *0.5,		2 *0.5,			2 *0.5,			1},	//坡道速度
  .Ramp		= 	{20,		0,			2,			2,		    1},	//坡道速度
  .Stop 	= 	{1,			0,			0,			0,			1},	//停车
  .Reverse 	= 	{0,			0,			0,			0,			1},	//制动停车
};

ControlStruct SpeedParm ={300 *0.5,	230 *0.5, 2 *0.5, 0.75 *0.5, 0};//当前速度系数
ControlStruct SelectMode ={300 *0.5, 230 *0.5, 2 *0.5, 0.75 *0.5, 0};

/************其它参数**************/
uint16 ExitFlag = 0;		//运行退出标志位

extern float AverageCenter;
extern int16 ServoPWMAdd;
extern int16 ServoPWM;
extern KeyInfo ki;
extern int16 Test;
extern uint16 Image_Use[RowMax][ColumnMax];
extern int16 Threshold;
extern uint16 gray_filter[RowMax][ColumnMax]; 
void DMAOnTimeHandle(void)
{
	if(SpeedParm.MaxSpeed)
	{
		Camera_scan();								//图像扫描
		//ServoControl(AverageCenter);
		/*MotorRun(ColumnMax/2-(int)AverageCenter);	//电机控制*/
	}
	mt9v03x_finish_flag= 0;
}

void run_qlh()
{
    int16 count = 0;
    pwm_duty(MOTOR_PINA,3900);
    run_refresh();

    //Camui_Handle();
   // Draw_Road();
    
        //开启一个pit中断，处理图像函数
            pit_interrupt_ms(CCU6_1,PIT_CH0,7);
            pit_enable_interrupt(CCU6_1,PIT_CH0);
            pit_start(CCU6_1,PIT_CH0);

        //再开启一个pit中断，处理pid算法
            // pit_interrupt_ms(CCU6_0,PIT_CH1,10);
            // pit_enable_interrupt(CCU6_0,PIT_CH1);
            // pit_start(CCU6_0,PIT_CH1);

    ips200_clear(IPS200_BGCOLOR);
    while(1)
    {
        Key_Scan(&ki);
        systick_delay_us(STM1,333);
        count++;
        if(count>100)
        {
            count=0;
            run_refresh();
        }
        if(mt9v03x_finish_flag)
        {
            mt9v03x_finish_flag=0;

           // ShowZoomImage(&mt9v03x_image[0], 188, 120, 94, 60);//左上角画面显示函数
            ShowZoomImage_gray(&gray_filter[0], 188, 120, 94, 60);//左上角画面显示函数
           
            Draw_Road();///将二值化的图像以及中线、前瞻显示出来
           // DrawLine1((ColumnMax-2)/2,Test,0,0);
        }

        if(ki.key_now[KEY_CANCEL]&&(!ki.key_last[KEY_CANCEL]))
            break;
    }
}

void run_refresh(void)
{
    ips200_showstr(1*MENU_CHAR_W,199,"AverageCenter:");
    ips200_showint16(18*MENU_CHAR_W,199,(int16)AverageCenter);

    ips200_showstr(1*MENU_CHAR_W,200,"ServoPWM:");
    ips200_showint16(18*MENU_CHAR_W,200,(int16)ServoPWM);
    
    ips200_showstr(1*MENU_CHAR_W,201,"PID:");
    ips200_showint16(6*MENU_CHAR_W,201,(int16)ServoPID.Proportion);
    ips200_showint16(12*MENU_CHAR_W,201,(int16)ServoPID.Integral);
    ips200_showint16(18*MENU_CHAR_W,201,(int16)ServoPID.Derivative);
}


void camtest_qlh(void)
{
    
    ips200_clear(IPS200_BGCOLOR);
    while(1)
    {
        Key_Scan(&ki);
        if(mt9v03x_finish_flag)
        {
            mt9v03x_finish_flag=0;

            ShowZoomImage(&mt9v03x_image[0], 188, 120, 94, 60);//左上角画面显示函数
           
            Get_Use_Image();   //获取图像数据
            Threshold = /*adapt_otsuThreshold(Image_Use,ColumnMax,RowMax);*/Sobel(1,RowMax-1); //Sobel算子法，return的是sobel卷积后得到的阈值
            Pixle_Filter(FilterThreshold);		//对sobel图像噪点依据阈值剔除
            Get_01_Value(Threshold,RowMax,ColumnMax);	//二值化图像数据
            Draw_Road();///将二值化的图像以及中线、前瞻显示出来
           // DrawLine1((ColumnMax-2)/2,Test,0,0);
        }

        if(ki.key_now[KEY_CANCEL]&&(!ki.key_last[KEY_CANCEL]))
            break;
    }

}
