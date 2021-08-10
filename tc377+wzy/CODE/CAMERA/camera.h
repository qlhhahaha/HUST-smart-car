#ifndef CAMERA_H_
#define CAMERA_H_


#include "common.h"
#include "SEEKFREE_IPS200_PARALLEL8.h"

#define White_Point     1
#define Black_Point     0

#define IMAGEH	MT9V03X_H
#define IMAGEW	MT9V03X_W

#define RowMax	    		60	//120  //行数
#define ColumnMax			94   //188  //列数

#define CurveToStraight		1
#define InStraight			2
#define StraightToCurve		3
#define InCurve				4

#define NoCircle			0
#define StopCheckCircle		1
#define ShouldBeCircle		2
#define MustBeCircle		3
#define ReadyInCircle		4
#define InCircle			5
#define OutCircle			6

#define WithoutRun        	1
#define ReadyRun          	2
#define StartRun        	3
#define InRunning			4
#define FinishRun         	5


extern volatile uint8  Field_Over_Flag;
extern float AverageCenter;                        	//归一化中线数值
extern int16 LeftCircleFlag;				//左环岛识别标志位
extern int16 RightCircleFlag;			//右环岛识别标志位
extern int16 AvaliableLines;							//有效行
extern int16 LastLine;
extern int StartFlag;
extern int Track_Type;
extern int Foresight;
extern int16 StartLineCount;
extern float Cirlce_Angle;					//转向角度
extern float CircleDistance;
extern float OutCircleDistance;
extern int16 FilterThreshold;

extern int16 BlackEndMR;
extern int16 BlackEndML;
extern int16 BlackEndLL;
extern int16 BlackEndRR;
extern int16 BlackEndL;
extern int16 BlackEndM;
extern int16 BlackEndR;
extern int16 BlackEndMin;

void OnlyOnce(void);///自加，

void Spurroad_conduct(void);///岔路处理函数

void AddLine(uint16 *sptr,float X0,float Y0,float X1,float Y1,int16 Xstart,int16 Xfinish,uint8 flag);

void Zebra_Filter(void);
float Middle_Err_Filter(float middle_err);
void TrackJudge(void);
void SearchCenterBlackline(void);
void GetBlackEndParam(void);

void NormalCrossConduct(void);

void CircleConduct(void);
void StartCheck(void);///int StartCheck(void);
void WeightedAverageCalc(uint32 AdvanceNum);
void BinaryImage(uint32 tmImage[IMAGEH][IMAGEW],uint32 ThresholdV);

int16 FindOvalPoint(int16 col,int16 flag);
void DrawLine(int16 x0,int16 y0,int16 x1,int16 y1);///画二值化图像的黑直线

void DrawLine1(int16 x0,int16 y0,int16 x1,int16 y1);///画灰度图像的红线自加用于测试

void Draw_Road(void);
void Get_Use_Image(void);
void Get_01_Value(int limit,int height,int width);
void Pixle_Filter(int16 threshold);

int16 Sobel(int16 start,int16 finish);
float Camera_scan(void);

uint32 CountRowW2BSalation(uint32 line,uint32 start,uint32 finish);
uint32 CountRowB2WSalation(uint32 line,uint32 start,uint32 finish);

int16 adapt_otsuThreshold(int16 *image, int16 col, int16 row);

#define CameraInit()					mt9v03x_init()
#define ShowZoomImage(src,x,y,x1,y1)	ips200_displayimage032_zoom(&mt9v03x_image[0],x,y,x1,y1);

//wzy的图像算法
// image
#define IMAGE_HEIGHT        MT9V03X_H
#define IMAGE_WIDTH         MT9V03X_W

// search
#define SEARCH_LEFT     1
#define SEARCH_RIGHT    (IMAGE_WIDTH - 2)
#define SEARCH_BOT      (MT9V03X_H * 5 / 6)//100
#define SEARCH_TOP	(MT9V03X_H * 1 / 12)//20
#define SEARCH_STEP_Y	1
#define SEARCH_STEP_X	1
#define SEARCH_MARGIN	7				

// regression
#define REGRESSION_MIN_N	4

// midline enmu
#define MIDLINE_MAX		1
#define MIDLINE_MIN		2
#define MIDLINE_LEFT	3
#define MIDLINE_RIGHT	4

// downsample
#define DOWNSAMPLE_X	10
#define DOWNSAMPLE_Y	10
#define DOWNSAMPLE_H	((SEARCH_BOT - SEARCH_TOP + 1) / DOWNSAMPLE_Y)
#define DOWNSAMPLE_W	((SEARCH_RIGHT - SEARCH_LEFT + 1) / DOWNSAMPLE_Y)
#define DOWNSAMPLE_N	(DOWNSAMPLE_W * DOWNSAMPLE_H)
#define DOWNSAMPLE_C	5                           // color
#define DOWNSAMPLE_S	(256 / DOWNSAMPLE_C + 1)    // color

void image_update_thresvalue();
void image_fast_threshold();
void image_init_err_weight();
void image_calcu_err();
void image_find_midline(uint8 mode);
uint16 image_judge_cross();
void image_midline_regression();
void image_leftline_regression();
void image_rightline_regression();
void image_debug();
uint16 image_fast_otsu();
uint16 image_find_whiteline();
uint16 image_fastsearch_leftline();
uint16 image_fastsearch_rightline();
uint16 image_line_filter(uint16 line_type);
uint16 image_line_analyse(uint16 line_type);

void image_debug_red();

#endif 
