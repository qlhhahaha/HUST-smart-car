/*
 * Dream-Seekers-camera.h
 * 
 *  Created on:
 *      Author:
 *     Version: V1.0
 *        Core: TC264D
 *
 *   	  Name:
 *		 Brief: 
 *	      Note:
 */

#ifndef CODE_CAMERA_H_
#define CODE_CAMERA_H_

#define White_Point     1
#define Black_Point     0

#define IMAGEH	MT9V03X_H
#define IMAGEW	MT9V03X_W

#define RowMax	    		60	  //行数
#define ColumnMax			94	  //列数

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

extern int16 Gyro_Z;
extern volatile uint8  Field_Over_Flag;
extern float AverageCenter;                        	//归一化中线数值
extern unsigned char	LeftCircleFlag;				//左环岛识别标志位
extern unsigned char	RightCircleFlag;			//右环岛识别标志位
extern int16 AvaliableLines;							//有效行
extern int16 LastLine;
extern unsigned char 	BeepFlag;
extern int StartFlag;
extern int Track_Type;
extern int Foresight;
extern int16 StartLineCount;
extern float Cirlce_Angle;					//转向角度
extern float CircleDistance;
extern float OutCircleDistance;
extern int16 FilterThreshold;
extern uint8  StartTurnFlag;

extern int16 BlackEndMR;
extern int16 BlackEndML;
extern int16 BlackEndLL;
extern int16 BlackEndRR;
extern int16 BlackEndL;
extern int16 BlackEndM;
extern int16 BlackEndR;
extern int16 BlackEndMaxMax;
extern int16 BlackEndMin;
extern int16 DropRow;

void OnlyOnce(void);///自加，

void Spurroad_conduct(void);///岔路处理函数

void AddLine(uint16 *sptr,float X0,float Y0,float X1,float Y1,int16 Xstart,int16 Xfinish,uint8 flag);

void LQMT9V034_Init(void);
void Get_way(void);
void CameraTest(int16 num);
void Zebra_Filter(void);
float Middle_Err_Filter(float middle_err);
void TrackJudge(void);
void SearchCenterBlackline(void);
void GetBlackEndParam(void);

void NormalCrossConduct(void);

void CircleConduct(void);
void TurningConduct(void);
void StartCheck(void);///int StartCheck(void);
void WeightedAverageCalc(uint16 AdvanceNum);
void oval(uint8 oval_x,uint8 oval_y,int8 zf);
void BinaryImage(uint8_t tmImage[IMAGEH][IMAGEW],uint8_t ThresholdV) ;

int16 FindOvalPoint(int16 col,int16 flag);
void jiaozheng(void);

void DrawLine(int16 x0,int16 y0,int16 x1,int16 y1);///画二值化图像的黑直线

void DrawLine1(int16 x0,int16 y0,int16 x1,int16 y1);///画灰度图像的红线自加用于测试

void Draw_fix_road(void);
void Draw_Road(void);
void Get_Use_Image(void);
void Get_01_Value(int limit,int height,int width);
void Pixle_Filter(int16 threshold);

//uint16_t iteration(u16 ERROR,volatile u8 Im[LCDH][LCDW]);
uint16_t Sobel(int16 start,int16 finish);
u8 LoseRoadConduct(u8 flag);


void Top_Bottom_Hat(int judge,int area_num);

int select_direction(void);
float Camera_scan(void);

u16 CountRowBlackPixle(u16 start,u16 finish);
u16 CountRowW2BSalation(u16 line,u16 start,u16 finish);
u16 CountRowB2WSalation(u16 line,u16 start,u16 finish);
u16 GetRowB2WSalation(u16 line,u16 start,u16 finish);
u16 CountColEnd(u16 Col);
void GetSalationLine(void);



#define CameraInit()					mt9v03x_init()
#define ShowZoomImage(src,x,y,x1,y1)	ips200_displayimage032_zoom(&mt9v03x_image[0],x,y,x1,y1);


void TestCamera(void);



#endif /* CODE_CAMERA_H_ */
