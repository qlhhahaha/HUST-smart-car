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

#define RowMax	    		60	//120  //����
#define ColumnMax			94   //188  //����

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
extern float AverageCenter;                        	//��һ��������ֵ
extern int16 LeftCircleFlag;				//�󻷵�ʶ���־λ
extern int16 RightCircleFlag;			//�һ���ʶ���־λ
extern int16 AvaliableLines;							//��Ч��
extern int16 LastLine;
extern int StartFlag;
extern int Track_Type;
extern int Foresight;
extern int16 StartLineCount;
extern float Cirlce_Angle;					//ת��Ƕ�
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

void OnlyOnce(void);///�Լӣ�

void Spurroad_conduct(void);///��·������

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
void DrawLine(int16 x0,int16 y0,int16 x1,int16 y1);///����ֵ��ͼ��ĺ�ֱ��

void DrawLine1(int16 x0,int16 y0,int16 x1,int16 y1);///���Ҷ�ͼ��ĺ����Լ����ڲ���

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

//ch��ͼ���㷨
#define ROW 120
#define COL 188
#define IMG_BLACK 0
#define IMG_WHITE 1
#define ROAD_CENTER_POINT 46//��·����������Ϊ��63��
#define BASE_LINE 5  //���һ��
#define ERR_USEROW 5 //ȡ������ǰ(64-ERR_USEROW)����ƫ�� ��BASE_LINEһ��
#define GETEDGE_START 59 //�ӽ�����GETEDGE_START�п�ʼ����
#define ROAD_CENTER_POINT 46//��·����������Ϊ��63��
#define iFilterW 3
#define iFilterH 3
#define iFilterMX 2
#define iFilterMY 2
#define WIDTHBYTES(bits)    (((bits) + 31) / 32 * 4)



extern int gray_difference;
extern int crossroad_flag;
extern int fork_flag;
extern uint8 found_num;

void get_edge_center_new(void);//���ûҶȲ�Ⱥ���߽�
void crossroad_pass(void);//ʮ�ּ��
void crossroad_patch(void);//ʮ�ֲ���
void fork_check(void);//�����
void get_center(void);
int16 adapt_otsuThreshold(int16 *image, int16 col, int16 row);   //ע�������ֵ��һ��Ҫ��ԭͼ��
unsigned char GetMedianNum(int * bArray, int iFilterLen);
void MedianFilter(unsigned char *pImg1,unsigned char *pImg,int nWidth,int nHeight);
void gray_to_ori(uint16 image_ori[ROW][COL]);
void get_longest_whiteline();
#endif 
