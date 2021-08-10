#ifndef CAMERA_H_
#define CAMERA_H_


#include "common.h"
#include "SEEKFREE_IPS200_PARALLEL8.h"

#define White_Point     1
#define Black_Point     0

#define FORK_LEFT     1
#define FORK_RIGHT     0

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

#define CameraInit()					mt9v03x_init();
#define ShowZoomImage(src,x,y,x1,y1)	ips200_displayimage032_zoom(&mt9v03x_image[0],x,y,x1,y1);
#define ShowZoomImage_gray(src,x,y,x1,y1)	ips200_displayimage032_zoom(&gray_filter[0],x,y,x1,y1);

//ch的图像算法
#define ROW 120
#define COL 188
#define IMG_BLACK 0
#define IMG_WHITE 1
#define ROAD_CENTER_POINT 46//道路理想中心线为第63列
#define BASE_LINE 5  //最近一行
#define ERR_USEROW 5 //取近处起前(64-ERR_USEROW)行算偏差 与BASE_LINE一致
#define GETEDGE_START 59 //从近处第GETEDGE_START行开始搜线
#define ROAD_CENTER_POINT 46//道路理想中心线为第63列
#define iFilterW 3
#define iFilterH 3
#define iFilterMX 2
#define iFilterMY 2
#define WIDTHBYTES(bits)    (((bits) + 31) / 32 * 4)



extern int gray_difference;
extern int crossroad_flag;
extern int fork_flag;
extern uint8 found_num;

void get_edge_center_new(void);//利用灰度差比和求边界
void crossroad_pass(void);//十字检测
void crossroad_patch(void);//十字补线
void fork_check(void);//检测岔口
void get_center(void);
int16 adapt_otsuThreshold(int16 *image, int16 col, int16 row);   //注意计算阈值的一定要是原图像
unsigned char GetMedianNum(int * bArray, int iFilterLen);
void MedianFilter(unsigned char *pImg1,unsigned char *pImg,int nWidth,int nHeight);
void gray_to_ori(uint16 image_ori[ROW][COL]);
void get_longest_whiteline();
#endif 
