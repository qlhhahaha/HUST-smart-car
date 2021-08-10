/*
 * Dream-Seekers-camera.c
 */

/*�����˼��:�㷨�����ܼ򵥣�׼ȷ��
 * 1������·���ߵĺ���ͻȻ����Ǳ���Ƕ��ߣ�����ζ�Ž�������ڣ����ó������ߣ�������һȦ���ٴ��ж���������������ߡ�
 * 2�����Թ۲쵽���еĵ�·���ͣ�������ʮ��·�ȣ���ǰ����û���赲�����ߣ�����������Ҫ���⴦��������һ�������������߶�������ǰ���к��߾��ж�Ϊ������
 **         �����ܻ���ͣ�������ͻ������Ҫ�ǵó��������Ϊ���ܾ�Ϊ���ߣ�
 * 3,��취�ֱ������ߵ�ǰհ������·�ģ�ǰհ�����Ǽ⣨����ǰհ���������жϣ�
 * 4�����������ҽ��ҳ����ж�Ϊ����ں󣬷�������������ھʹ������һ������ڣ�һ�����������ڳ�����������ڣ�����������������ȷ������·��������
 */

/*
 * ����߿�ʼ���ߣ�˵�������ұ�ƫ����Ҫ��������ͬ���ұ�������ˣ�������Ϊ���������ж����ݡ�
 */


#include "headfile.h"

int Test = 5;

/*********ͼ����ȡ��س�ʼ��**********/
volatile uint16 Image_Use[RowMax][ColumnMax];     	 	//ѹ����ͼ������
volatile uint32 Pixle[RowMax][ColumnMax];    	//��ֵ��������OLED��ʾ������//
volatile uint16 Image_Sobel[RowMax][ColumnMax];     	//Sobelͼ������

/*********ͼ������س�ʼ��**********/
int16 Threshold;               
int16 FilterThreshold = 225;
int16 Camera_scan_finish = 0;
const int Width[RowMax+1]=  					//ͼ��ÿ��ֱ����Ӧ�п� ///��ͷ���Ƹ߶Ⱥ�����п�ҲҪ����Ӧ�ĸı䣬Ҳ����˵������Ҫ�ֶ����¡�
{
    /* 0 */	1,		1,		1,		1,		1,		1,		1,		2,		3,		3,
    /* 1 */	10,		10,		10,		10,		16,		17,		17,		19,		29,		31,
    /* 2 */	33,		35,		37,		39,		41,		42,		44,		46,		48,		50,
    /* 3 */	52,		54,		56,		57,		58,		60,		62,		63,		65,		66,
    /* 4 */	67,		69,		70,		71,		72,		73,		75,		75,		77,		77,
    /* 5 */	79,		79,		80,		81,		82,		82,		82,		82,		82,		82,		
    88
};

float Weight[60]=   							//������Ȩ�أ��������ѹ��,���������
{
   /* 0 */	0.1,	0.2,	0.3,	0.4,	0.5,	0.5,	0.5,	0.5,	0.5,	0.5,
    /* 1 */	0.6,	0.7,	0.8,	0.9,	1.0,	1.2,	1.2,	1.4,	1.4,	1.5,
    /* 2 */	1.8,	2.0,	2.1,	2.3,	2.5,	2.6,	2.8,	2.9,	3.0,	3.2,
    /* 3 */	3.4,	3.6,	3.8,	4.0,	4.2,	4.3,	4.4,	4.2,	4.0,	3.8,
    /* 4 */	3.6,	3.4,	3.2,	3.0,	2.8,	2.6,	2.4,	2.2,	2.0,	1.8,
    /* 5 */	1.6,	1.6,	1.5,	1.5,  	1.4,  	1.4,  	1.2,  	1.2,  	1.0,   	0.8
};

float 	AverageCenter=0;						//��һ������ֵ
float 	LastAverageCenter=0;					//��һ������
int16	LastAverageError[5]= {0};				//����ƫ���
int16	LeftLose       = 0;						//����������
int16	RightLose      = 0; 					//���Ҷ�������
int16	AllLose        = 0;						//����ȫ������������ʮ�֣�
int16	WhiteLose      = 0;						//����ȫ��������������ʧ������
int16	LeftLoseStart  = 0;						//��¼��߶��ߵĿ�ʼ��
int16	RightLoseStart = 0;						//��¼�ұ߱߶��ߵĿ�ʼ��
int16	WhiteLoseStart = 0;						//��¼ȫ�����ߵĿ�ʼ��
int16	FirstLoseAllLine=0;						//ȫ���׿�ʼ��///�����ʼ����Ҫ���¶���
int16	LastLoseAllLine=0;						//ȫ���׽�����
int16	StartLineCount = 0;						//�����߼�����
int16	LastLine = 0;							//ɨ�����һ��,��̬ǰհ
int16	AvaliableLines = 0;						//ɨ����Ч����
int16		SearchErrorFlag = 0;

int   Foresight		 = 0;					//ǰհ
int   Last_Foresight = 0;					//��һ��ǰհ
int16 Fictitious_Num = 0;					//����ֱ�������ȡ��ƫ��
int	  Track_ImgCount = 0;					//�����жϼ������
int	  Track_Type	 = 0;					//��������

uint32 MiddleLine[RowMax+1];					//���ߴ������
uint32 RightEdge[RowMax+1];  					//�ұ��ߴ����
uint32 LeftEdge[RowMax+1]; 					//����ߴ������
uint32 RealWidth[RowMax+1];					//����ʵ�ʿ��
uint32 Left_Add_Line[RowMax+1];				//���ߴ������
uint32 Right_Add_Line[RowMax+1];				//�Ҳ��ߴ������
uint32 Left_Add_Flag[RowMax];					//���߱�־λ
uint32 Right_Add_Flag[RowMax];					//�Ҳ��߱�־λ
uint32 Width_Add[RowMax+1];///uint32 Width_Add[RowMax];//���߿������
uint32 Width_Min;								//������С���
int16 Line_Count;							//��¼�ɹ�ʶ�𵽵���������
uint32 Left_Add_Start = 0;						//������ʼ
uint32 Left_Add_Stop = 0;						//���߽�����
uint32 Right_Add_Start = 0;					//�Ҳ�����ʼ
uint32 Right_Add_Stop = 0;						//�Ҳ��߽�����
float Left_Ka = 0, Right_Ka = 0;			//��С���˷�����
float Left_Kb = 1, Right_Kb = ColumnMax-1;

/*********��ֹ����ȡ������ʼ��**********/
int16 BlackEndMR      = 0;
int16 BlackEndML      = 0;
int16 BlackEndLLL     = 0;
int16 BlackEndLL      = 0;
int16 BlackEndRRR     = 0;
int16 BlackEndRR      = 0;
int16 BlackEndL       = 0;
int16 BlackEndM       = 0;
int16 BlackEndR       = 0;
int16 BlackEndMin     = 0;

int16 EndMArray[5];
int16 EndMLArray[5];
int16 EndMRArray[5];

int16 LastBlackEndLL	= 0;
int16 LastBlackEndRR	= 0;
int16 LastBlackEndLLL	= 0;
int16 LastBlackEndRRR	= 0;

/*********ʮ�ִ��������ʼ��**********/
unsigned char  CrossFlag  = 0;					//ʮ�ֱ�־
unsigned char  LastCrossFlag  = 0;
unsigned char  LastLastCrossFlag  = 0;
unsigned char  CrossNumber=0;
unsigned char  LoseCrossNumber=0;
unsigned char  CrossEnterFlag=0;
float CrossKL=0,CrossKR=0;

/*********�������������ʼ��**********/
float	Cirlce_Angle=0.0;				//�����Ƕ�
float 	CircleDistance = 0.0;
float 	OutCircleDistance=0.0;
int16	RightOvalFlag = 0;
int16	LeftOvalFlag = 0;
int16	LeftCircleFlag = 0;				//�󻷵�ʶ���־λ
int16	RightCircleFlag = 0;			//�һ���ʶ���־λ
int16 	DrawOvalFlag=0;					//��������־
int16   SalationFlag=0;
int16	LLSalationLine = 0;				//�������������ʼ��
int16	LRSalationLine = 0;				//�������������ʼ��
int16	RLSalationLine = 0;				//�ұ�����������ʼ��
int16	RRSalationLine = 0;				//�ұ�����������ʼ��

int16 Onlyonce = 1;///�Լӱ���

/*********��·������ʼ��**********/
float 	StationAngle = 0.0;
int 	StationLineNum = 0;
extern int16 LeftWheelSpeed;
//extern int16 RightWheelSpeed;

/*********����·����**********/
///
unsigned char Road_widening = 0;

unsigned char Road_vanish = 0;

int16 spurroadtriangle_i = 0;

int16 spurroadtriangle_j = 0;

int16  left_right_lost_i = 0;                      ///����ȫ��������

unsigned char  SpurroadFlag  = 0;                  //����ڱ�־

unsigned char  LastSpurroadFlag  = 0;

unsigned char  LastLastSpurroadFlag  = 0;

float SpurroadKL=0,SpurroadKR=0;

/*********����������ʼ��**********/
int StartFlag =WithoutRun;						//�����߱�־λ

/**********��غ궨��************/
#define StartLine       57				//����ɨ����ʼ��

#define FinishLine      18				//����ɨ�������

#define StepLine		1				//��Ե׷��ɨ������

#define DivideLine		StartLine-10	//ȫ��ɨ��ͱ�Ե׷�ٷָ���///47

#define MIDVALUE		48

#define ImageMode      		2         	//ѡ��ͼ��ģʽ��0-OSTU/1-ITERATION/2-Sobel��

#define CheckWhitePixle(x,y) Image_Sobel[x][y] >= Threshold	//�׵��ж�
#define CheckBlackPixle(x,y) Image_Sobel[x][y] < Threshold	//�ڵ��ж�
#define CheckLeft(x,y)	CheckBlackPixle(x,y) && CheckWhitePixle(x,y-1) //&& CheckWhitePixle(x,y-2)
#define CheckRight(x,y) CheckBlackPixle(x,y) && CheckWhitePixle(x,y+1) //&& CheckWhitePixle(x,y+2)
#define CheckEnd(x,y)	CheckWhitePixle(x,y) //&& CheckWhitePixle(x-1,y)
#define pi 3.14

/*************************wzy�ı���***********************/
#define ABS(a) (a > 0 ? a: -a)
#define DELT(a, b) (a - b)
#define ABS_DELT(a, b) (a > b ? a - b: b - a)
#define DIFFER(a, b) ((a - b) * 100 / (a + b))
#define FAST_DIFFER(a, b) (((a - b) << 7 ) / (a + b + 1))
#define ABS_DIFFER(a, b) a >= b ? (((a - b) << 7 ) / (a + b + 1) : ((b - a) << 7 ) / (a + b + 1)) 

static uint16 g_mid_x = 0;
static uint16 g_left_curve = 0;
static uint16 g_right_curve = 0;
static uint16 g_left_count = 0;
static uint16 g_right_count = 0;
static uint16 g_left_line[IMAGE_HEIGHT];
static uint16 g_right_line[IMAGE_HEIGHT];
static uint16 g_mid_line[IMAGE_HEIGHT];
static uint16 g_err_weight[IMAGE_HEIGHT]=
{
  1,1,1,1,1,1,1,1,1,1,   //111-120
  1,1,1,1,1,1,1,1,1,1,   //101-110
  2,2,2,2,2,2,2,2,2,2,   //91-100
  3,3,3,3,3,3,3,3,3,3,   //81-90
  3,3,3,3,3,3,3,3,3,3,   //71-80
  3,3,3,3,3,3,3,3,3,3,   //61-70
  2,2,2,2,2,2,2,2,2,2,   //51-60
  2,2,2,2,2,2,2,2,2,2,   //41-50
  2,2,2,2,2,2,2,2,2,2,   //31-40
  1,1,1,1,1,1,1,1,1,1,   //21-30
  1,1,1,1,1,1,1,1,1,1,
  1,1,1,1,1,1,1,1,1,1,

};
static uint16 g_weight_sum = 0;
static uint16 x_conditon_left = 0;
static uint16 x_conditon_right = 0;

/**********************************************************/

//ͼ�����ܺ���
float Camera_scan(void)
{
    Get_Use_Image();                                        //��ȡͼ������
    Threshold = /*adapt_otsuThreshold(Image_Use,ColumnMax,RowMax);*/ Sobel(1,RowMax-1);  //Sobel���ӷ���return����sobel�����õ�����ֵ
    Pixle_Filter(FilterThreshold);		//��sobelͼ�����������ֵ�޳�
    GetBlackEndParam();               //������ȡ���߽�ֹ��
    // if(DrawOvalFlag)
    // {
    //     if(RightCircleFlag == OutCircle)
    //         DrawLine(RowMax-1,5,FindOvalPoint(ColumnMax/2,1),75);///�ӣ�59��5������15~20��75��
    //     else if(LeftCircleFlag == OutCircle)
    //         DrawLine(RowMax-1,90,FindOvalPoint(ColumnMax/2,3),20);///�ӣ�59��90������15~20��20��
    // }

    Get_01_Value(Threshold,RowMax,ColumnMax);				//��ֵ��ͼ������
    //SearchCenterBlackline();								//Ѱ����

   // gray_to_ori(Image_Sobel[ROW][COL]); //��120*128�ĻҶ�ͼѹ����64*128

    // CircleConduct();										//�Ի������д���
    // StartCheck();                                         	//�����߼��
    WeightedAverageCalc(SpeedParm.AdvanceParm);				//���߼�Ȩ��һ

//    if(DrawOvalFlag)//��CircleConduct()���л�����⣬�л���ʱ��DrawOvalFlag��ֵΪ1
//    {
//        if(RightCircleFlag==MustBeCircle || RightCircleFlag==ShouldBeCircle)		//�ҵ�����������Բ
//        {
//            AverageCenter = 93;//���߸���������
//        }
//        else if(LeftCircleFlag==MustBeCircle  || RightCircleFlag==ShouldBeCircle)	//�󵺽���������Բ
//        {
//            AverageCenter = 1;//���߸���������
//        }
//    }

#if 0
    if(StartLineCount>=10)								//�ٿ���������ͣ��
    {
        int i = 0;
        StationLineNum = 0;
        for(i=StartLine; i>FinishLine; i--) //60///��57��19
        {
            if(Image_Use[i][0]+30<Image_Use[i][ColumnMax-1])
                StationLineNum--;
            else if(Image_Use[i][0]>Image_Use[i][ColumnMax-1]+30)
                StationLineNum++;
        }
        if(StationLineNum > 20 && AvaliableLines < 20)
        {
            StartFlag = FinishRun;
            SpeedParm = GearCtl.Stop;
        }
        else if(StationLineNum < -20 && AvaliableLines < 20)
        {
            StartFlag = FinishRun+1;
            SpeedParm = GearCtl.Stop;
        }
//		if(ABS(StationLineNum) > 10 && AvaliableLines<20)
//			SpeedParm = GearCtl.Stop;
    }

    if(StartFlag >= FinishRun)///ʹ���������ǵ�����
    {
//		if(ABS(LeftWheelSpeed) + ABS(RightWheelSpeed) <120)
//		  SpeedParm = GearCtl.Zebra;
        StationAngle += (float)GYRO.Z*0.00064;		//��ת��ǶȽ��л��� ���ƽ����̶�

        if(StationAngle < 45 && StartFlag == FinishRun)
//			DrawLine(RowMax-1,90,FindOvalPoint(ColumnMax/2,3),1);
            AverageCenter = 1;
        else if(StationAngle > -45 && StartFlag == FinishRun+1)
//			DrawLine(RowMax-1,5,FindOvalPoint(ColumnMax/2,1),93);
            AverageCenter = 93;
//		if(AvaliableLines < 30 && StartLineCount < 5 && ABS(StationAngle) > 40)
        if(ABS(StationAngle) > 30 && ExitFlag)
        {
//			if(AvaliableLines < 30)
//			{
            SpeedParm = GearCtl.Stop;							//ͣ��
            StationAngle = 0.0;
            StationLineNum = 0;
            StartFlag = 0;
//			}
        }
    }
#endif

    //Middle_Err_Filter(AverageCenter);						//�����˲�
    TrackJudge();											//���������ж�
    LastAverageCenter = AverageCenter;						//���߱���
//	Draw_Road();

    Camera_scan_finish = 1;

    return AverageCenter;
}


/**
 * @file		����ǰϵ����Ȩƽ��
 * @note
 
 */
void WeightedAverageCalc(uint32 AdvanceNum)
{
    int16 i=0, PointAdvance=0;
    float sum=0;
    for(sum=0,i=StartLine; i>= 1 /*LastLine*/; i--) ///��57��0
    {
        PointAdvance = i-AdvanceNum;///PointAdvance��iʼ�ղ�һ��AdvanceNum
        if (PointAdvance>59)	PointAdvance=59;///��59��Ķ���59����
        AverageCenter += (g_centerline[PointAdvance]*Weight[i]);///*���ߴ������*�е������������Ȩ����˺��ۼ�///��ǰAdvanceNum����ȡ��������
        sum += Weight[i];///��������Ȩ���ۼ�
    }
    AverageCenter = AverageCenter/sum;              //��Ȩƽ��
    AverageCenter = RANGEfloat(AverageCenter, 0, ColumnMax-1);///�޶���Χ
}

/**
 * @file		����ƫ���˲�
 * @note
 
 */
float Middle_Err_Filter(float middle_err)
{
    float Middle_Err_Fltered;
    Middle_Err_Fltered = middle_err*0.8 + LastAverageCenter*0.2;///���ȡ0.8���ϴε�0.2���
    return Middle_Err_Fltered;
}
/**
 * @file		��ʼ�������˲�
 * @note
 
 */
void Zebra_Filter(void)
{
    int16 ZebraFilterLoc[StartLine+2][40];//û����ô�������      ///57*40������
    int16 i=0,j=0,k=0,num=1;
    for(i=StartLine+1; i>=FinishLine-1; i--) ///58��18
    {
        for(j=25; j<ColumnMax-25-1; j++) ///25��67
        {
            if(CheckWhitePixle(i,j))///����ǰ׵�
                ZebraFilterLoc[i][num++]=j;	//�ڶ�λ��ʼ�浥�������λ��///��j��¼���õ��Ӧ����������
        }
        for(k=num; k<40; k++) ZebraFilterLoc[i][k]=0; ///����׵����������40����ִ��������䣬����numǰ�浽40ȫ����
        ZebraFilterLoc[i][0]=num-1;		//��һλ�����������
        num=1;
    }
    for(i=StartLine+1; i>=FinishLine-1; i--) ///58��18
    {
        for(j=1; j<=ZebraFilterLoc[i][0]; j++) ///1���׵�����
        {
            if(ZebraFilterLoc[i][ZebraFilterLoc[i][0]]!=0&&							//������һ������0һ������û�ҵ������
                    ABS(ZebraFilterLoc[i][ZebraFilterLoc[i][0]]-ZebraFilterLoc[i][1])<30)///���һ���㲻����0�����һ���Ĳ�ֵС��30��
            {
                for(k=ZebraFilterLoc[i][1]; k<=ZebraFilterLoc[i][ZebraFilterLoc[i][0]]; k++) ///�ӵ�һ�������׵�
                {
                    Image_Sobel[i][k] = 0;
                }
            }
        }
    }
}

/**
 * @file		��С���˷����б��
 */
float Slope_Calculate(int16 begin,int16 end,uint32 *p)
{
    float xsum=0,ysum=0,xysum=0,x2sum=0;
    int16 i=0;
    float result=0;
    static float resultlast;
    p=p+begin;
    for(i=begin; i<end; i++)
    {
        if((*p)<ColumnMax-3&&(*p)>3)
        {
            xsum+=i;
            ysum+=(float)*p;
            xysum+=i*(float)(*p);
            x2sum+=i*i;
            p=p+1;
        }
    }
    if((end-begin)*x2sum-xsum*xsum) //�жϳ����Ƿ�Ϊ��
    {
        result=((end-begin)*xysum-xsum*ysum)/((end-begin)*x2sum-xsum*xsum);
        resultlast=result;
    }
    else
    {
        result=resultlast;
    }
    return result;
}

/**
 * @file		����ֱ�����
 * @note     	��������ֱ�ߣ�����������֮�Ƚϣ��������ƶ�
 */
int16 Fictitious_Beeline(void)
{
    float KA,KB;
    int16 i;
    int16 Sum=0;
    int16 Num;
    float res;
    int16 Result;

    KA = 1.0*(MiddleLine[StartLine] - MiddleLine[FinishLine]) / (StartLine-FinishLine);
    KB = 1.0*MiddleLine[FinishLine] - (KA * FinishLine);

    for(i=StartLine; i>LastLine; i-=StepLine)
    {
        res = i * KA + KB;
        Result = RANGE16(res,2,ColumnMax-2);
        Num = MiddleLine[i]- Result;
        Sum+=Num;
    }
    return Sum;
}

/**
 * @file		·���ж�
 * @note     	����ǰհ���ж���������
 * @author		AHNU�����ߣ�CYX��---�ο���������
 * @date		2019
 */
void TrackJudge(void)
{
    //ʹ����Զ��ƫ��ͼ�Ȩƫ��ȷ��ǰհ
    Foresight = 0.7 * ABS(MiddleLine[LastLine+2]-ColumnMax/2)+ 0.3 * ABS(AverageCenter-ColumnMax/2);
    Last_Foresight=Foresight;

    Fictitious_Num = Fictitious_Beeline();
    Track_ImgCount++;
    if(Track_ImgCount==5)
    {
        Track_ImgCount=0;
        LastAverageError[0] = LastAverageError[1];
        LastAverageError[1] = LastAverageError[2];
        LastAverageError[2] = LastAverageError[3];
        LastAverageError[3] = LastAverageError[4];
        LastAverageError[4] = ABS(AverageCenter-ColumnMax/2);
    }
    //�����ֱ��
    if(LastAverageError[0]>7 && LastAverageError[1]>6 && LastAverageError[2]>5 && LastAverageError[4] < 4
            && BlackEndM > 30 && ABS(Fictitious_Num)<30 )
    {
        Track_Type = CurveToStraight;
    }
    //ֱ��
    else if(LastAverageError[0]<4 && LastAverageError[1]<4 && LastAverageError[2]<4 && Foresight<=5)
    {
        Track_Type = InStraight;
    }
    //ֱ�������
    else if(LastAverageError[0]<6 && LastAverageError[1]<6 && LastAverageError[2]<6 && Foresight>=5)
    {
        Track_Type = StraightToCurve;
    }
    //�����
    else
    {
        Track_Type = InCurve;
    }
}

/**
 * @file		���㲹������
 * @note     	ʹ�����㷨������ϳ��Ĳ�������
 * @author		AHNU�����ߣ�CYX��---�ο���������
 * @date		2019
 */
int16 Calculate_Add(int16 i, float Ka, float Kb)	// ���㲹������
{
    float res;
    int16 Result;

    res = i * Ka + Kb;
    Result = RANGE16((int32)res, 1, ColumnMax-1);

    return Result;
}

/**
 * @file		���㷨���ֱ��
 * @note     	����ʼ�������һ�������ߵ�б�ʴ�������ĵ�
				���ֱ�� y = Ka * x + Kb
				Mode == 1������߽磬Mode == 2�����ұ߽�
 */
void Curve_Fitting(float *Ka, float *Kb, int16 *Start, int16 *Line, int16 *Add_Flag, int16 Mode)
{
    *Start += StepLine;///start++
    if (Add_Flag[*Start] == 1)///���start+1��Ҫ����
    {
        if (*Start <= 51)///��start+1С�ڵ���51
            *Start += StepLine;///start��ԭʼֵ��2
        if (Mode == 2)
        {
            *Ka = 1.0*(Line[*Start+StepLine] - Line[*Start]) / 2;///Line����ԭʼֵ��3��-line����ԭʼֵ��2��///(����y+1)-��y)/2  б��
            if (*Ka < 0)		*Ka = 0;
        }
        if (Mode == 1)
        {
            *Ka = 1.0*(Line[*Start+StepLine] - Line[*Start]) / 2;
            if (*Ka > 0)		*Ka = 0;
        }
    }
    else///����
    {
        if(Mode == 2)
            *Ka = 1.0*(Line[*Start+StepLine] - Line[*Start]) / 2;///Line����ԭʼֵ��2��-line����ԭʼֵ��1��
        if(Mode == 1)
            *Ka = 1.0*(Line[*Start+StepLine] - Line[*Start]) / 2;
    }
    *Kb = 1.0*Line[*Start] - (*Ka * (*Start));
}

/**
 * @file		�����޸�
 * @note     	��ʼ���ղ�ʹ�ã�ֱ��ʹ������б�ʽ��в���
 *              Mode == 1������߽磬Mode == 2�����ұ߽�
 */
void Line_Repair(int16 Start, int16 Stop,int16 *Line, int16 *Line_Add, int16 *Add_Flag, int16 Mode)
{
    float res;
    int16 i;	// ������
    float Ka, Kb;

    if ((Mode == 1) && (Right_Add_Start <= Stop) && Start <= StartLine-5)	// ֻ����߽粹��
    {
        if (Start <= StartLine-StepLine)///Start���������26��Start�Լ�
        {
            Start +=StepLine;
        }
        for (i = Start+StepLine; i >= Stop+StepLine;)
        {
            i -= StepLine;///һ��ʼ����ѭ�����ֳ�ʼֵ��֮����Լ���Stop+Stepline
            Line_Add[i] = RANGE16(Right_Add_Line[i] - Width_Add[i+StepLine], 1, ColumnMax-1);///���ߵĳ����޷�
            if(  ABS(Line_Add[i]-Line_Add[i+1])  >5)	Line_Add[i] = Line_Add[i+1];///���еĲ��߳��Ⱥ���һ�еĲ�ֵ�ľ���ֵ����5��������һ�еĲ�������
            Width_Add[i] = Width_Add[i+StepLine];///���߿����������
        }
    }
///***************************************�Լ�****************************************
    else
    {
        if (Stop)   // ��ʼ����
        {
            if ((Right_Add_Stop >= MIDVALUE && Left_Add_Stop >= MIDVALUE) ||
                    (Right_Add_Stop >= MIDVALUE && Left_Add_Start <= Right_Add_Stop) ||
                    (Left_Add_Stop >= MIDVALUE && Right_Add_Start <= Left_Add_Stop))    // ֻ�н�������Ҫ���ߣ�������б�ʣ�ֱ����ֱ���²���
            {
                for (i = Stop-2; i <= 57; i += StepLine)
                    Line_Add[i] = Line_Add[Stop];
            }
            else    // ����ʼ�кͽ����м���б�ʲ���
            {
                if (Start <= StartLine)
                {
                    Start +=StepLine;
                }
                Ka = 1.0*(Line_Add[Start] - Line_Add[Stop]) / (Start - Stop);
                Kb = 1.0*Line_Add[Start] - (Ka * Start);

                for (i = Stop; i <= Start; i += StepLine)
                {
                    res = i * Ka + Kb;
                    Line_Add[i] = RANGE16((int32)res, 1, ColumnMax-1);
                }
            }
        }
    }
///***************************************�Լ�****************************************
    if ((Mode == 2) && (Left_Add_Start <= Stop) && Start <= StartLine-5)	// ֻ���ұ߽粹��
    {
        if (Start <= StartLine-StepLine)
        {
            Start +=StepLine;
        }
        for (i = Start+StepLine; i >= Stop+StepLine;)
        {
            i -= StepLine;
            Line_Add[i] = RANGE16(Left_Add_Line[i] + Width_Add[i+StepLine], 1, ColumnMax-1);
            if(ABS(Line_Add[i]-Line_Add[i+1])>5)	Line_Add[i] = Line_Add[i+1];
            Width_Add[i] = Width_Add[i+StepLine];
        }
    }
    else
    {
        if (Stop)	// ��ʼ����
        {
            if ((Right_Add_Stop >= MIDVALUE && Left_Add_Stop >= MIDVALUE) ||
                    (Right_Add_Stop >= MIDVALUE && Left_Add_Start <= Right_Add_Stop) ||
                    (Left_Add_Stop >= MIDVALUE && Right_Add_Start <= Left_Add_Stop))	// ֻ�н�������Ҫ���ߣ�������б�ʣ�ֱ����ֱ���²���
            {
                for (i = Stop-2; i <= 57; i += StepLine)
                    Line_Add[i] = Line_Add[Stop];
            }
            else	// ����ʼ�кͽ����м���б�ʲ���
            {
                if (Start <= StartLine)
                {
                    Start +=StepLine;
                }
                Ka = 1.0*(Line_Add[Start] - Line_Add[Stop]) / (Start - Stop);
                Kb = 1.0*Line_Add[Start] - (Ka * Start);

                for (i = Stop; i <= Start; i += StepLine)
                {
                    res = i * Ka + Kb;
                    Line_Add[i] = RANGE16((int32)res, 1, ColumnMax-1);
                }
            }
        }
    }
}


void OnlyOnce(void)///�Լ�
{
    if(Onlyonce)
    {
        MiddleLine[RowMax]  = ColumnMax/2;
        LeftEdge[RowMax]    = 0;
        RightEdge[RowMax]   = ColumnMax;
        MiddleLine[57]  = ColumnMax/2;
        LeftEdge[57]    = 0;
        RightEdge[57]   = ColumnMax;
        MiddleLine[58]  = ColumnMax/2;
        LeftEdge[58]    = 0;
        RightEdge[58]   = ColumnMax;
        MiddleLine[56]  = ColumnMax/2;
        LeftEdge[56]    = 0;
        RightEdge[56]   = ColumnMax;
    }

    Onlyonce = 0;

};

/**
 * @file		ȫ��ɨ��ͱ�Ե�����ȡ����������
 * @note     	���ߺ����㷨
 
 */
void SearchCenterBlackline(void)
{
    int16 i = 0, j = 0, k =0;
    int16 LeftFilterStart=0;
    int16 RightFilterStart=0;
    int16 StartColumn  = 0;
    int16 FinishColumn = 0;
    int16 numfind=0;
    //ȫ�ֱ�������
    LeftLose        = 0;
    RightLose       = 0;
    AllLose         = 0;
    StartLineCount  = 0;
    SearchErrorFlag = 0;
    WhiteLose = 0;
    WhiteLoseStart = 0;
    LeftLoseStart = 0;
    RightLoseStart = 0;
    FirstLoseAllLine = 0;
    LastLoseAllLine = 0;
    Line_Count = 0;
    SalationFlag=0;
    //��λ������ʼ������
    Left_Add_Start = 0;
    Right_Add_Start = 0;
    Left_Add_Stop = 0;
    Right_Add_Stop = 0;
    //��λ��60������
    MiddleLine[RowMax-1]  = ColumnMax/2;///47
    LeftEdge[RowMax-1]    = 0;
    RightEdge[RowMax-1]   = ColumnMax;///94

    OnlyOnce();///�Լ�

    /*************************************************************************************/
    /*                           һ�α�����ʼ��ȫ��+��Ե��                                                          */
    /*************************************************************************************/
    for(i=StartLine; i>DivideLine; i--) ///57��47				//������ǰN�У�ȫ��ɨ��
    {

        //���ұ߽綼������λ����һ����
        /***********************************������߽�********************************************/
        if(i == StartLine)  		j = MiddleLine[RowMax];	//���о�����һ��ͼ��������Ϊɨ�����///���i=StartLineʱ��MiddleLine[RowMax]����Ч���ݣ�������Ҫdebug
        else if(AllLose > 5)		j = ColumnMax/2;
        else  					j = MiddleLine[i+1]; 	//���������һ���е��λ����Ϊ����ɨ�����
        if(j < 3)     			j = 3;     				//j>=3��Ч��Χ������
        while(j >= 3)									//j>=3��Ч��Χ�ڽ�����Ѱ
        {
            //���������ҵ��׺ں�����
            if(CheckLeft(i,j))
            {
                LeftEdge[i] = j;						//�ҵ���ֵ �Ҳ�������ԭֵ0
                break;									//��������Ѱ��
            }
            j--;										//���������ƶ�
        }
        if(j < 3)
            LeftEdge[i] =1; 								//�Ҳ�������߾�����Ϊ1
        /***********************************�����ұ߽�********************************************/
        if(i==StartLine)   		j = MiddleLine[RowMax];	//���о�����һ��ͼ��������Ϊɨ�����
        else if(AllLose>5)		j = ColumnMax/2;
        else					j = MiddleLine[i+1]; 	//�������һ������λ�ÿ�ʼ��Ѱ
        if(j >ColumnMax-3)		j = ColumnMax-3; 		//j <=ColumnMax-3��Ч��Χ������
        while(j <= ColumnMax-3)
        {
            //���������ҵ��׺ں������
            if(CheckRight(i,j))
            {
                RightEdge[i] = j;					//�ҵ���ֵ   �Ҳ�������ԭֵ
                break;								//��������Ѱ��
            }
            j++;										//���������ƶ�
        }
        if(j >= ColumnMax-3)
            RightEdge[i] = ColumnMax-1;					//�Ҳ����ұ��߾�����ΪColumnMax-1

        /************************************�˲�********************************************/
        if(LeftEdge[i]==1&&LeftEdge[i+1]==1&&LeftEdge[i+2]==1&&LeftEdge[i+3]==1&&LeftEdge[i+4]==1)
        ///һֱ�Ҳ��������
        {
            LeftFilterStart=i;///LeftFilterStart��Ϊi��߽��˲���ʼ��
        }
        if(RightEdge[i]==ColumnMax-1&&RightEdge[i+1]==ColumnMax-1&&RightEdge[i+2]==ColumnMax-1&&RightEdge[i+3]==ColumnMax-1&&RightEdge[i+4]==ColumnMax-1)
        ///һֱ�Ҳ����ұ���
        {
            RightFilterStart=i;
        }
        /************************************�����ж�********************************************/
        Left_Add_Line[i] = LeftEdge[i];						//��¼ʵ����߽�Ϊ������߽�
        Right_Add_Line[i] = RightEdge[i];					//��¼ʵ���ұ߽�Ϊ������߽�
        if((RightEdge[i]-LeftEdge[i]) >= (RightEdge[i+1]-LeftEdge[i+1]+2) && i<StartLine)//���������
            ///((RightEdge[i]-LeftEdge[i]) >= (RightEdge[i+1]-LeftEdge[i+1]+2) && i<StartLine)
        {
            Left_Add_Flag[i]=1;///��һ�����߱�־��1
            Right_Add_Flag[i]=1;
            AllLose++;///���в�������������ж�ʮ��
            if(AllLose>=2||AllLose<=5)
                FirstLoseAllLine = i;
            left_right_lost_i = i;
            MiddleLine[i] = MiddleLine[i+1];//����һ��
        }
        else if(	(LLSalationLine && RightEdge[i]==ColumnMax-1)||
                    (RRSalationLine && LeftEdge[i]==1))  	//ȫ��
        {
            Left_Add_Flag[i]=1;
            Right_Add_Flag[i]=1;
            AllLose++;
            if(AllLose>=2||AllLose<=5)
                FirstLoseAllLine = i;
            LastLoseAllLine = i;
            left_right_lost_i = i;
            MiddleLine[i]=MiddleLine[i+1];
        }
        else if(LeftEdge[i]!=1 && RightEdge[i]!=ColumnMax-1)   	//û�ж���
        {
            Left_Add_Flag[i]=0;
            Right_Add_Flag[i]=0;
            MiddleLine[i] = (LeftEdge[i] + RightEdge[i])/2;///���߼�Ϊ�������е�
            if((RightEdge[i]-LeftEdge[i]) >= (RightEdge[i+1]-LeftEdge[i+1]+1) && i<StartLine)//���������
                MiddleLine[i] = MiddleLine[i+1];//����һ��
        }
        else if(LeftEdge[i]==1 && RightEdge[i]!=ColumnMax-1)	//��������
        {
            if(!LeftLoseStart)	LeftLoseStart = i;///��¼��߶��ߵĿ�ʼ��
            Left_Add_Flag[i]=1; ///��һ�����߱�־��1
            Right_Add_Flag[i]=0;///��һ���Ҳ��߱�־��0
            if(i<=RowMax-1 && RightEdge[i] > Width[i]/2 + 1)	//�����Ļ����ð��///�ұ߽����ÿ�е�·�߽��һ����Ϊ������
                ///���˾����ұ߽�ҲӦ��С��width[i]///����߽綪ʧʱ���ұ߽粻���ܴ���width[i]��
                MiddleLine[i] = RightEdge[i] - Width[i]/2;
            else												//Խ������������
                MiddleLine[i] = 1;///���߶�Ϊ1��Ϊ��Ҫ�������ת��
            LeftLose++;///����������
        }
        else if(LeftEdge[i]!=1 && RightEdge[i]==ColumnMax-1)	//��������
        {
            Left_Add_Flag[i]=0;
            Right_Add_Flag[i]=1;
            if(!RightLoseStart)	RightLoseStart = i;
            if(i<=RowMax-1 && LeftEdge[i] + Width[i]/2 <  ColumnMax-1)		//�����Ļ����ð��
                MiddleLine[i] = LeftEdge[i] + Width[i]/2;
            else														//Խ�������ҽ����
                MiddleLine[i] = ColumnMax-1;
            BlackEndM++;//��¼ֻ�����߶�������///���Գ���˵ӦΪ�������д���
            RightLose++;///���Ҷ�������
        }
        else if((LeftEdge[i]==1 && RightEdge[i]==ColumnMax-1)) //ȫ��
        {
            Left_Add_Flag[i]=1;
            Right_Add_Flag[i]=1;
            AllLose++;
            if(AllLose>=2||AllLose<=5)
                FirstLoseAllLine = i;
            LastLoseAllLine = i;///ȫ�������л�����i�ļ��ٶ��仯�������С�
            left_right_lost_i = i;
            MiddleLine[i]=MiddleLine[i+1];
        }
        if(Left_Add_Flag[i])														//��߽���Ҫ����
        {
            if (i >= StartLine-6 && i < StartLine-1)													//ǰ6��
                Left_Add_Line[i] = LeftEdge[StartLine-1];								//ʹ�õ�������
            else if(i < StartLine-6)                    												//ʣ�µ�
                Left_Add_Line[i] = Left_Add_Line[i+1];						//ʹ��ǰ1����߽���Ϊ������߽�
        }
        if(Right_Add_Flag[i])														//�ұ߽���Ҫ����
        {
            if (i >= StartLine-6 && i < StartLine-1)													//ǰ6��
                Right_Add_Line[i] = RightEdge[StartLine-1];							//ʹ�õ�������
            else if(i < StartLine-6)      																//ʣ�µ�
                Right_Add_Line[i] = Right_Add_Line[i+1];						//ʹ��ǰ1���ұ߽���Ϊ�����ұ߽�
        }
        if(RightEdge[i] > LeftEdge[i])		RealWidth[i]=RightEdge[i]-LeftEdge[i];	//����ʵ���������
        else								RealWidth[i]=0;
        if(Right_Add_Line[i] > Left_Add_Line[i])									//���㲹���������
            Width_Add[i] = Right_Add_Line[i] - Left_Add_Line[i];
        else
            Width_Add[i] = 0;

        if(!RRSalationLine && i<StartLine-5 &&///���������RightEdge[i]ʼ������С�ģ��������ֻ��˵����������䣬����������㡣Ŀǰ����ֻ����һ�Ρ�
                RightEdge[i]==MIN(MIN(RightEdge[i+4],RightEdge[i+3]),RightEdge[i+2])&&
                RightEdge[i]==MIN(MIN(RightEdge[i  ],RightEdge[i+1]),RightEdge[i+2]))
        {
            RRSalationLine = i+2;///�ұ�����������ʼ��
        }
        else if(!LLSalationLine && i<StartLine-5 &&
                LeftEdge[i]==MAX(MAX(LeftEdge[i+4],LeftEdge[i+3]),LeftEdge[i+2])&&
                LeftEdge[i]==MAX(MAX(LeftEdge[i+2],LeftEdge[i+1]),LeftEdge[i]))
        {
            LLSalationLine = i+2;
        }
        /*******************************��ʼ�����ݴ���**************************************/
        if(i==StartLine)///ֻ������ʼ������
        {
            LeftEdge      [RowMax]     = LeftEdge      [StartLine];		///���ݱ��浽60�ж�Ӧ����
            RightEdge     [RowMax]     = RightEdge     [StartLine];
            Left_Add_Line [RowMax]     = Left_Add_Line [StartLine];
            Right_Add_Line[RowMax]     = Right_Add_Line[StartLine];
            Left_Add_Line [StartLine+1]= Left_Add_Line [StartLine];
            Right_Add_Line[StartLine+1]= Right_Add_Line[StartLine];


            if (Left_Add_Flag[StartLine] && Right_Add_Flag[StartLine])///��ʼ��������Ҫ����
            {
                MiddleLine[StartLine] = MiddleLine[RowMax];///��60�е����ߴ��濪ʼ��
                FirstLoseAllLine = 0;///ȫ���׿�ʼ������
            }
            else
            {
                MiddleLine[RowMax] = MiddleLine[StartLine];	// ���µ�60�������е㣬������һ֡ͼ��ʹ��
            }

            MiddleLine[RowMax] = RANGE16(MiddleLine[RowMax],15,80);///�����޷���15��80֮��

            RealWidth[RowMax] = RealWidth[StartLine];

            Width_Add[RowMax] = Width_Add[StartLine];

            Width_Add[StartLine+1] = Width_Add[StartLine];

            Width_Min = Width_Add[StartLine]; //Width_Min��ʵ�ǵ�һ���������Ҳ����
            //û�н������ν���ͼ�����������
        }
    }
    /*�������������������������������������ָ��ߡ���������������������������������������*/
    for(i=DivideLine; i>=FinishLine; i-=StepLine)//����ʣ����///47��18
    {
        LastLine = i;///ɨ�����һ��
        /*******************************�ж����ұ߽練������*************************************/
        if(   (!SearchErrorFlag) && (   (LeftEdge[i+1]>=ColumnMax/2+5) || (RightEdge[i+1]<=ColumnMax/2-5)   )   )
        {
            ///ColumnMax/2+5=52,ColumnMax/2-5=42///��߽���ұ߽��쳣
            if(LeftEdge[i+1]>=ColumnMax/2+10 || RightEdge[i+1]<=ColumnMax/2-10 )///��߽����57�����ұ߽�С��37
            {
                SearchErrorFlag = i+2;///��¼�쳣�У�֮���˳�ѭ����
                break;
            }
            else if(LeftEdge[i+1]>=ColumnMax/2+5)///52
            {
                if(RightEdge[i+1] == ColumnMax-1)///93///δ�ҵ��ұ߽�
                {
                    FinishColumn = ColumnMax-1;
                    while(FinishColumn >= LeftEdge[i+1])///������������߽�
                    {
                        if(CheckLeft(i,FinishColumn))
                        {
                            LeftEdge[i] = FinishColumn;///�ҵ��ͼ�¼����
                            break;
                        }
                        FinishColumn--;
                    }
                    if(FinishColumn < LeftEdge[i+1])///���������һ�е���߽�
                        LeftEdge[i] = LeftEdge[i+1];
                }
                else
                {
                    LeftEdge[i] = LeftEdge[i+1];///�ҵ��ұ߽磬��߽�����һ�е�
                }
            }
            else if(RightEdge[i+1]<=ColumnMax/2-5)///42
            {
                if(LeftEdge[i+1] == 1)
                {
                    FinishColumn = 1;
                    while(FinishColumn <= RightEdge[i+1])
                    {
                        if(CheckRight(i,FinishColumn))
                        {
                            RightEdge[i] = FinishColumn;
                            break;
                        }
                        FinishColumn++;
                    }
                    if(FinishColumn > RightEdge[i+1])
                        RightEdge[i] = RightEdge[i+1];
                }
                else
                {
                    RightEdge[i] = RightEdge[i+1];
                }
            }
        }
        /***************************��һ��û�ҵ��߽磬ȫ��ȫ��ɨ��*******************************/
        if((LeftEdge[i+1]==1 && RightEdge[i+1]==ColumnMax-1)||
                LeftLose>5||RightLose>5||AllLose>5)
        {
            ////////////////////////��ͨ��ȫ��ɨ������߽�///////////////////////////
            if(AllLose>5)
                FinishColumn = MiddleLine[RowMax];///���ȫ����60�������
            else
                FinishColumn = MiddleLine[i+1];///��������һ�е�����
            if(FinishColumn <= 2)
            {
                FinishColumn= 3;
            }
            while(FinishColumn >= 3)
            {
                if(CheckLeft(i,FinishColumn))
                {
                    LeftEdge[i] = FinishColumn;///�ҵõ���������
                    break;
                }
                FinishColumn--;
                if(FinishColumn < 3)
                    LeftEdge[i] = 1;///�����������
            }
            ////////////////////////��ͨ��ȫ��ɨ�����ұ߽�///////////////////////////
            if(AllLose>5)
                FinishColumn = MiddleLine[RowMax];
            else
                FinishColumn = MiddleLine[i+1];
            if(FinishColumn<LeftEdge[i])		FinishColumn = LeftEdge[i]+1;
            if(FinishColumn >= ColumnMax-3)
            {
                FinishColumn = ColumnMax-3;
            }
            while(FinishColumn <= ColumnMax-3)
            {
                if(CheckRight(i,FinishColumn))
                {
                    RightEdge[i] = FinishColumn;
                    break;
                }
                FinishColumn++;
                if(FinishColumn > ColumnMax-3)
                    RightEdge[i] = ColumnMax - 1;
            }
        }


#if 0//���Լ����ε���һ����


        /***************************��һ�����߶��ҵ� ȫ������ɨ��*******************************/
        else if(LeftEdge[i+1]!=1 && RightEdge[i+1]!=ColumnMax-1)
        {
            //////////////////////////////������߽�///////////////////////////////////
            FinishColumn = ((LeftEdge[i+1]+10) >= ColumnMax-3)? ColumnMax-3:(LeftEdge[i+1]+10);///��ColumnMax-3����ColumnMax-3
            StartColumn = ((LeftEdge[i+1]-5) <= 3)? 3:(LeftEdge[i+1]-5);///��3С��3
            while(FinishColumn >= StartColumn)
            {
                if(CheckLeft(i,FinishColumn))
                {
                    LeftEdge[i] = FinishColumn;
                    break;
                }
                FinishColumn--;
                if(FinishColumn < StartColumn)
                    LeftEdge[i] = 1;
            }
            //////////////////////////////�����ұ߽�///////////////////////////////////
            FinishColumn = ((RightEdge[i+1]-10) <= 3)? 3:(RightEdge[i+1]-10);
            StartColumn = ((RightEdge[i+1]+5) >= ColumnMax-3)? ColumnMax-3:(RightEdge[i+1]+5);
            while(FinishColumn <= StartColumn)
            {
                if(CheckRight(i,FinishColumn))
                {
                    RightEdge[i] = FinishColumn;
                    break;
                }
                FinishColumn++;
                if(FinishColumn > StartColumn)
                    RightEdge[i] = ColumnMax - 1;
            }
        }
        /***************************��һ��ֻ�ҵ���߽� ����+ȫ��ɨ��*******************************/
        else if(LeftEdge[i+1]!=1 && RightEdge[i+1]==ColumnMax-1)
        {
            ////////////////////////��ͨ������ɨ������߽�///////////////////////////
            FinishColumn = ((LeftEdge[i+1]+10) >=ColumnMax-2)? ColumnMax-2:(LeftEdge[i+1]+10);///��ColumnMax-2����ColumnMax-2
            StartColumn = ((LeftEdge[i+1]-5) <= 1)? 1:(LeftEdge[i+1]-5);///��1С��1
            while(FinishColumn >= StartColumn)
            {
                if(CheckLeft(i,FinishColumn))
                {
                    LeftEdge[i] = FinishColumn;
                    break;
                }
                FinishColumn--;
                if(FinishColumn < StartColumn)
                    LeftEdge[i] = 1;
            }
            ////////////////////////��ͨ��ȫ��ɨ�����ұ߽�///////////////////////////
            FinishColumn = MiddleLine[i+1];
            if(FinishColumn<LeftEdge[i])		FinishColumn = LeftEdge[i]+1;
            if(FinishColumn >= ColumnMax-3)
            {
                FinishColumn = ColumnMax-3;
            }
            while(FinishColumn <= ColumnMax-3)
            {
                if(CheckRight(i,FinishColumn))
                {
                    RightEdge[i] = FinishColumn;
                    break;
                }
                FinishColumn++;
                if(FinishColumn > ColumnMax-3)
                    RightEdge[i] = ColumnMax - 1;
            }
        }
        /***************************��һ��ֻ�ҵ��ұ߽� ����+ȫ��ɨ��*******************************/
        else if(LeftEdge[i+1]==1 && RightEdge[i+1]!=ColumnMax-1)
        {
            ////////////////////////��ͨ������ɨ�����ұ߽�//////////////////////////
            FinishColumn = ((RightEdge[i+1]-10) <= 1)? 1:(RightEdge[i+1]-10);
            StartColumn = ((RightEdge[i+1]+5) >= ColumnMax-2)? ColumnMax-2:(RightEdge[i+1]+5);
            while(FinishColumn <= StartColumn)
            {
                if(CheckRight(i,FinishColumn))
                {
                    RightEdge[i] = FinishColumn;
                    break;
                }
                FinishColumn++;
                if(FinishColumn > StartColumn)
                    RightEdge[i] = ColumnMax - 1;
            }
            ////////////////////////��ͨ��ȫ��ɨ������߽�///////////////////////////
            FinishColumn = MiddleLine[i+1];
            if(FinishColumn>RightEdge[i])		FinishColumn = RightEdge[i]-1;
            if(FinishColumn <= 2)
            {
                FinishColumn= 3;
            }
            while(FinishColumn >= 3)
            {
                if(CheckLeft(i,FinishColumn))
                {
                    LeftEdge[i] = FinishColumn;
                    break;
                }
                FinishColumn--;
                if(FinishColumn < 3)
                    LeftEdge[i] = 1;
            }
        }
#endif

        /************************************�����ж�********************************************/
        Left_Add_Line[i] = LeftEdge[i];						//��¼ʵ����߽�Ϊ������߽�
        Right_Add_Line[i] = RightEdge[i];					//��¼ʵ���ұ߽�Ϊ������߽�
        if((RightEdge[i]-LeftEdge[i]) >= (RightEdge[i+1]-LeftEdge[i+1]+3)///(RightEdge[i]-LeftEdge[i]) >= (RightEdge[i+1]-LeftEdge[i+1]+3
                && i<StartLine)//���������
        {
            if(LeftLose>10||RightLose>10||AllLose>10)///�����������ȫ���ߴ���10
                SalationFlag = 1;
        }
        if(SalationFlag)
        {
            Left_Add_Flag[i]=1;
            Right_Add_Flag[i]=1;
            AllLose++;
            if(AllLose>=2||AllLose<=5)
                FirstLoseAllLine = i;
            LeftEdge[i] = LeftEdge[i+1];
            RightEdge[i] = RightEdge[i+1];
            MiddleLine[i] = MiddleLine[i+1];//����һ��
        }
        else if(	(LLSalationLine && RightEdge[i]==ColumnMax-1)||
                    (RRSalationLine && LeftEdge[i]==1))  	//ȫ��
        {
            Left_Add_Flag[i]=1;
            Right_Add_Flag[i]=1;
            AllLose++;
            if(AllLose>=2||AllLose<=5)
                FirstLoseAllLine = i;
            LastLoseAllLine = i;
            left_right_lost_i = i;
            MiddleLine[i]=MiddleLine[i+1];
        }
        else if(LeftEdge[i]!=1 && RightEdge[i]!=ColumnMax-1)   	//û�ж���
        {
            Left_Add_Flag[i]=0;
            Right_Add_Flag[i]=0;
            MiddleLine[i] = (LeftEdge[i] + RightEdge[i])/2;
        }
        else if(LeftEdge[i]==1 && RightEdge[i]!=ColumnMax-1)	//��������
        {
            if(!LeftLoseStart)	LeftLoseStart = i;
            Left_Add_Flag[i]=1;
            Right_Add_Flag[i]=0;
            if(i<=RowMax-1 && RightEdge[i] > Width[i]/2 + 1)	//�����Ļ����ð��
                MiddleLine[i] = RightEdge[i] - Width[i]/2;
            else												//Խ������������
                MiddleLine[i] = 1;
            LeftLose++;
        }
        else if(LeftEdge[i]!=1 && RightEdge[i]==ColumnMax-1)	//��������
        {
            if(!RightLoseStart)	RightLoseStart = i;
            Left_Add_Flag[i]=0;
            Right_Add_Flag[i]=1;
            if(i<=RowMax-1 && LeftEdge[i] + Width[i]/2 <  ColumnMax-1)		//�����Ļ����ð��
                MiddleLine[i] = LeftEdge[i] + Width[i]/2;
            else														//Խ�������ҽ����
                MiddleLine[i] = ColumnMax-1;
            RightLose++;//��¼ֻ�����߶�������
        }
        else if((LeftEdge[i]==1 && RightEdge[i]==ColumnMax-1) ||
                (LLSalationLine && RightEdge[i]==ColumnMax-1) ||
                (RRSalationLine && LeftEdge[i]==1))  	//ȫ��
        {
            Left_Add_Flag[i]=1;
            Right_Add_Flag[i]=1;
            AllLose++;
            if(AllLose>=2||AllLose<=5)
                FirstLoseAllLine = i;
            MiddleLine[i]=MiddleLine[i+1];
        }
        if(Left_Add_Flag[i])													//��߽���Ҫ����
        {
            if (i >= StartLine-6 && i < StartLine-1)							//ǰ6��ʹ�õ�������
                Left_Add_Line[i] = LeftEdge[StartLine-1];
            else if(i < StartLine-6)											//��6��ʹ��ǰ1����߽���Ϊ������߽�
                Left_Add_Line[i] = Left_Add_Line[i+1];
        }
        if(Right_Add_Flag[i])													//�ұ߽���Ҫ����
        {
            if (i >= StartLine-6 && i < StartLine-1)							//ǰ6��ʹ�õ�������
                Right_Add_Line[i] = RightEdge[StartLine-1];
            else if(i < StartLine-6)      										//��6��ʹ��ǰ1����߽���Ϊ������߽�
                Right_Add_Line[i] = Right_Add_Line[i+1];
        }
        if(RightEdge[i] > LeftEdge[i])			RealWidth[i]=RightEdge[i]-LeftEdge[i];	//����ʵ���������
        else									RealWidth[i]=0;							//��Խ����ֱ����0
        if(Right_Add_Line[i] > Left_Add_Line[i])										//���㲹���������
            Width_Add[i] = Right_Add_Line[i] - Left_Add_Line[i];
        else
            Width_Add[i] = 0;

        SearchErrorFlag = 0;
    }
    /*************************************************************************************/
    /*                              һ�α�������                                         */
    /*************************************************************************************/
    /*************************************************************************************/
    /*                      ���α�����ʼ�����в���+��̬ǰհ��                             */
    /*************************************************************************************/
    // for(i=StartLine; i>LastLine; i-=StepLine) ///��57��0
    // {
    //     /********************************* ���߼�⿪ʼ *************************************/
    //     if (RealWidth[i] > Width_Min+1)	// ������ȱ��������ʮ�ֻ�·��///����������·
    //     {
    //         //Width_Min��ʵ�ǵ�һ���������Ҳ����
    //         //û�н������ν���ͼ�����������

    //         Road_widening = 1;///��·����־�������ж�����·

    //         if (Left_Add_Line[i] < Left_Add_Line[i+2] &&
    //                 Left_Add_Line[i+2] < Left_Add_Line[i+4] && i<StartLine) 	//������������Զ�����խ��Left_Add_Line[i]�ȽϽ����Ĵ�
    //         {
    //             if (!Left_Add_Flag[i])
    //                 Left_Add_Flag[i] = 1;	//��������Ϊ0��ǿ���϶�Ϊ��Ҫ����
    //         }
    //         if (Right_Add_Line[i] > Right_Add_Line[i+2] &&
    //                 Right_Add_Line[i+2] > Right_Add_Line[i+4] &&i<StartLine)  	//�������෴
    //         {
    //             if (!Right_Add_Flag[i])
    //                 Right_Add_Flag[i] = 1;	//��������Ϊ0��ǿ���϶�Ϊ��Ҫ����
    //         }
    //         if ((Left_Add_Flag[i] || Right_Add_Flag[i] || SearchErrorFlag == i) && (i<FinishLine+10))///(Left_Add_Flag[i] || Right_Add_Flag[i] || SearchErrorFlag == i && i<FinishLine+10)
    //         {
    //             if (Left_Add_Stop || Right_Add_Stop || SearchErrorFlag == i)
    //             {
    //                 break;
    //             }
    //         }
    //     }
    //     /******************************** ��һ�ֲ��߿�ʼ ************************************/
    //     if (Left_Add_Flag[i] && i<StartLine)	// �����Ҫ����
    //     {
    //         if (i >= StartLine-3 && i < StartLine)	// ǰ���в��߲���
    //         {
    //             if (!Left_Add_Start && Left_Add_Flag[i-1] && Left_Add_Flag[i-2])///i��54��56֮����ǰ������Ҫ����
    //             {
    //                 Left_Add_Start = i;	// ��¼���߿�ʼ��
    //                 Left_Ka = 0;
    //                 Left_Kb = Left_Add_Line[i];
    //             }
    //             Left_Add_Line[i] = Calculate_Add(i, Left_Ka, Left_Kb); ///���ճ���Գ�ԭ���Լ�
    //         }
    //         else if(i < StartLine-3)///iС��54
    //         {
    //             if (!Left_Add_Start && Left_Add_Flag[i-1] && Left_Add_Flag[i-2])	// ֮ǰû�в���
    //             {
    //                 Left_Add_Start = i;	// ��¼��ಹ�߿�ʼ��
    //                 Curve_Fitting(&Left_Ka, &Left_Kb, &Left_Add_Start, Left_Add_Line, Left_Add_Flag, 1);	// ʹ�����㷨���ֱ��///����µ�Ka��Kb
    //             }
    //         }
    //         Left_Add_Line[i] = Calculate_Add(i, Left_Ka, Left_Kb);	// ʹ��ǰһ��ͼ����߽�б�ʲ���///��һ�����߳��򣬻�һ����
    //     }
    //     else
    //     {
    //         if (Left_Add_Start)	// �Ѿ���ʼ���߶��ҿ�ʼ������
    //         {
    //             if (!Left_Add_Stop &&
    //                     !Left_Add_Flag[i+2*StepLine] &&
    //                     !Left_Add_Flag[i+4*StepLine]
    //                     && i<StartLine-5*StepLine)///�ں����кͺ����в�������iС��52
    //             {
    //                 if ((Left_Add_Line[i] >= Left_Add_Line[i+2*StepLine] &&
    //                         Left_Add_Line[i+2*StepLine] >= Left_Add_Line[i+4*StepLine]&&
    //                         Left_Add_Line[i]!=1 &&
    //                         Left_Add_Line[i+1*StepLine]!=1 &&
    //                         Left_Add_Line[i+2*StepLine]!=1))///��i��i+1��i+2������߲���1�ҵ�i��i+2,i+4�е���������α��˵���������
    //                 {
    //                     Left_Add_Stop = i+4*StepLine;	// ��¼��ಹ�߽�����

    //                     //Line_Repair(Left_Add_Start, Left_Add_Stop, LeftEdge, Left_Add_Line, Left_Add_Flag, 1);
    //                 }
    //             }
    //         }
    //     }
    //     if (Right_Add_Flag[i] && i<StartLine)	// �Ҳ���Ҫ����
    //     {
    //         if (i >= StartLine-3 && i < StartLine)	// ǰ���в��߲���
    //         {
    //             if (!Right_Add_Start && Right_Add_Flag[i-1] && Right_Add_Flag[i-2])
    //             {
    //                 Right_Add_Start = i;	// ��¼���߿�ʼ��
    //                 Right_Ka = 0;
    //                 Right_Kb = Right_Add_Line[i];
    //             }
    //             Right_Add_Line[i] = Calculate_Add(i, Right_Ka, Right_Kb);	// ʹ��ǰһ֡ͼ���ұ߽�б�ʲ���
    //         }
    //         else if(i < StartLine-3)

    //         {
    //             if (!Right_Add_Start)	// ֮ǰû�в���
    //             {
    //                 Right_Add_Start = i;	// ��¼�Ҳಹ�߿�ʼ��
    //                 Curve_Fitting(&Right_Ka, &Right_Kb, &Right_Add_Start, Right_Add_Line, Right_Add_Flag, 2);	// ʹ�����㷨���ֱ��
    //             }
    //             Right_Add_Line[i] = Calculate_Add(i, Right_Ka, Right_Kb);	// �������
    //         }
    //     }
    //     else
    //     {
    //         if (Right_Add_Start)	// �Ѿ���ʼ����
    //         {
    //             if (!Right_Add_Stop &&
    //                     !Right_Add_Flag[i+StepLine] &&
    //                     !Right_Add_Flag[i+2*StepLine] &&
    //                     i<StartLine-5*StepLine)
    //             {
    //                 if ((Right_Add_Line[i] <= Right_Add_Line[i+1*StepLine] &&
    //                         Right_Add_Line[i+1*StepLine] <= Right_Add_Line[i+2*StepLine]
    //                         && Right_Add_Line[i]!=ColumnMax-1
    //                         && Right_Add_Line[i+1*StepLine]!=ColumnMax-1
    //                         && Right_Add_Line[i+2*StepLine]!=ColumnMax-1))
    //                 {
    //                     Right_Add_Stop = i+4*StepLine;	// ��¼�Ҳಹ�߽�����
    //                     //Right_Add_Start = 0;
    //                     //Line_Repair(Right_Add_Start, Right_Add_Stop, RightEdge, Right_Add_Line, Right_Add_Flag, 2);
    //                 }
    //             }
    //         }
    //     }
    //     /********************************* ��һ�ֲ��߽��� **********************************/
    //     if(Right_Add_Line[i] >= Left_Add_Line[i])
    //         Width_Add[i] = Right_Add_Line[i] - Left_Add_Line[i];	// ���¼����������
    //     else
    //         Width_Add[i] = 0;
    //     if ((Left_Add_Flag[i] && Right_Add_Flag[i]) || (!Left_Add_Flag[i] && !Right_Add_Flag[i]))///����ͬʱ��Ҫ���߻���ͬʱ����Ҫ
    //         MiddleLine[i] = (Right_Add_Line[i] + Left_Add_Line[i]) / 2;	// ��������
    //     else if(i<StartLine)
    //         MiddleLine[i] = MiddleLine[i+StepLine];
    //     if (Width_Add[i] < Width_Min)
    //     {
    //         Width_Min = Width_Add[i];				// ������С�������
    //     }
    //     Line_Count = i;                          	//Line_Count����
    // }
    // /*************************************************************************************/
    // /*                              ���α�������                                         */
    // /*************************************************************************************/

    // /*************************************************************************************/
    // /*                     ���α�����ʼ�������޸�+�����޸���                              */
    // /*************************************************************************************/
    // /******************************* �����޸���ʼ ********************************/

    // if (Left_Add_Start)		// ��߽���Ҫ����
    // {
    //     Line_Repair(Left_Add_Start, Left_Add_Stop, LeftEdge, Left_Add_Line, Left_Add_Flag, 1);
    // }
    // if (Right_Add_Start)	// �ұ߽���Ҫ����
    // {
    //     Line_Repair(Right_Add_Start, Right_Add_Stop, RightEdge, Right_Add_Line, Right_Add_Flag, 2);
    // }
    // /******************************* �����޸����� ********************************/
    // /******************************* �����޸���ʼ ********************************/
    // for(i=StartLine; i>=Line_Count; i-=StepLine)
    // {
    //     MiddleLine[i] = (Right_Add_Line[i] + Left_Add_Line[i]) / 2;	// ���������е�
    //     if(Right_Add_Line[i] >= Left_Add_Line[i])
    //         Width_Add[i] = Right_Add_Line[i] - Left_Add_Line[i];	// ���¼����������
    //     else
    //         Width_Add[i] = 0;
    //     if(SearchErrorFlag == i || CheckEnd(i-1,MiddleLine[i]) || i == FinishLine)///�������̬ǰհ�ж�����ڵ�����
    //     {
    //         LastLine = i;//���һ�У���̬ǰհ
    //         spurroadtriangle_i = i;
    //         spurroadtriangle_j = MiddleLine[i];
    //         AvaliableLines = StartLine - i;//��Ч����
    //         break;
    //     }
    // }

    /*************************************************************************************/
    /*                              ���α�������                                         */
    /*************************************************************************************/
    for(k = FinishLine; k<=StartLine; k++)
    {
        for(j = 0; j<ColumnMax; j++)
        {
            if(j==MiddleLine[k] || k==LastLine)

                Pixle[k][j] = 1;
        }

    }
}

/**
 * @file		��ȡͼ�����������ȡ���߽�ֹ��
 * @note     	ѡȡ���У���ͼ��׽�����Զɨ��
 
 */
void GetBlackEndParam(void)
{
    unsigned char LLLEndFlag = 0;///�׵��־λ
    unsigned char LLEndFlag = 0;
    unsigned char LEndFlag  = 0;
    unsigned char MLEndFlag = 0;
    unsigned char MEndFlag  = 0;
    unsigned char MREndFlag = 0;
    unsigned char REndFlag  = 0;
    unsigned char RREndFlag = 0;
    unsigned char RRREndFlag = 0;

    int i=0;

    LastBlackEndLL = BlackEndLL;
    LastBlackEndLLL= BlackEndLLL;
    LastBlackEndRR = BlackEndRR;
    LastBlackEndRRR= BlackEndRRR;

//����
    BlackEndLLL  = 0;
    BlackEndLL   = 0;
    BlackEndL    = 0;
    BlackEndML   = 0;
    BlackEndM    = 0;
    BlackEndMR   = 0;
    BlackEndR    = 0;
    BlackEndRR   = 0;
    BlackEndRRR  = 0;

#if ImageMode<=1
    for (i = StartLine; i >0; i--)
    {
        if(CheckWhitePixle(i,ColumnMax/12) && !LLLEndFlag )//10
            BlackEndLLL++;
        else if(i > 1 && CheckBlackPixle(i-1,ColumnMax/12) && CheckBlackPixle(i-2,ColumnMax/12))
            LLLEndFlag = 1;

        if(CheckWhitePixle(i,ColumnMax/8) && !LLEndFlag)//10
            BlackEndLL++;
        else if(i > 1 && CheckBlackPixle(i-1,ColumnMax/8) && CheckBlackPixle(i-2,ColumnMax/8))
            LLEndFlag = 1;

        if(CheckWhitePixle(i,ColumnMax*2/8) && !LEndFlag)//10
            BlackEndL++;//����߽�����
        else if(i > 1 && CheckBlackPixle(i-1,ColumnMax*2/8) && CheckBlackPixle(i-2,ColumnMax*2/8))
            LEndFlag = 1;

        if(CheckWhitePixle(i,ColumnMax*3/8) && !MLEndFlag)//10
            BlackEndML++;
        else if(i > 1 && CheckBlackPixle(i-1,ColumnMax*3/8) && CheckBlackPixle(i-2,ColumnMax*3/8))
            MLEndFlag = 1;

        if(CheckWhitePixle(i,ColumnMax*4/8) && !MEndFlag)//10
            BlackEndM++;//�к��߽�����
        else if(i > 1 && CheckBlackPixle(i-1,ColumnMax*4/8) && CheckBlackPixle(i-2,ColumnMax*4/8))
            MEndFlag = 1;

        if(CheckWhitePixle(i,ColumnMax*5/8) && !MREndFlag)//10
            BlackEndMR++;
        else if(i > 1 && CheckBlackPixle(i-1,ColumnMax*5/8) && CheckBlackPixle(i-2,ColumnMax*5/8))
            MREndFlag = 1;

        if(CheckWhitePixle(i,ColumnMax*6/8) && !REndFlag)//10
            BlackEndR++;//�Һ��߽�����
        else if(i > 1 && CheckBlackPixle(i-1,ColumnMax*6/8) && CheckBlackPixle(i-2,ColumnMax*6/8))
            REndFlag = 1;

        if(CheckWhitePixle(i,ColumnMax*7/8) && !RREndFlag)//10
            BlackEndRR++;
        else if(i > 1 && CheckBlackPixle(i-1,ColumnMax*7/8) && CheckBlackPixle(i-2,ColumnMax*7/8))
            RREndFlag = 1;

        if(CheckWhitePixle(i,ColumnMax*11/12) && !RRREndFlag)//10
            BlackEndRRR++;
        else if(i > 1 && CheckBlackPixle(i-1,ColumnMax*11/12) && CheckBlackPixle(i-2,ColumnMax*11/12))
            RRREndFlag = 1;
    }
#else
    for (i = RowMax-1; i >0; i--)///��59��0��///�ҵ��׵�����ҵ��ڵ�,�ڵ���������
    {
        if(CheckBlackPixle(i,ColumnMax/12) && !LLLEndFlag )//94/12=7.83
            BlackEndLLL++;//��i�У���7���Ǻڵ�
        else if(i > 1 && CheckEnd(i,ColumnMax/12))
            LLLEndFlag = 1;//��i�У���7���ǰ׵�

        if(CheckBlackPixle(i,ColumnMax/8) && !LLEndFlag)//94/8=11.75
            BlackEndLL++;
        else if(i > 1 && CheckEnd(i,ColumnMax/8))
            LLEndFlag = 1;

        if(CheckBlackPixle(i,ColumnMax*2/8) && !LEndFlag)//23.5
            BlackEndL++;//����߽�����
        else if(i > 1 && CheckEnd(i,ColumnMax*2/8))
            LEndFlag = 1;

        if(CheckBlackPixle(i,ColumnMax*3/8) && !MLEndFlag)//35.25
            BlackEndML++;
        else if(i > 1 && CheckEnd(i,ColumnMax*3/8))
            MLEndFlag = 1;

        if(CheckBlackPixle(i,ColumnMax*4/8) && !MEndFlag)///47
            BlackEndM++;//�к��߽�����
        else if(i > 1 && CheckEnd(i,ColumnMax*4/8))
            MEndFlag = 1;

        if(CheckBlackPixle(i,ColumnMax*5/8) && !MREndFlag)///58.75
            BlackEndMR++;
        else if(i > 1 && CheckEnd(i,ColumnMax*5/8))
            MREndFlag = 1;

        if(CheckBlackPixle(i,ColumnMax*6/8) && !REndFlag)///70.5
            BlackEndR++;//�Һ��߽�����
        else if(i > 1 && CheckEnd(i,ColumnMax*6/8))
            REndFlag = 1;

        if(CheckBlackPixle(i,ColumnMax*7/8) && !RREndFlag)///82.25
            BlackEndRR++;
        else if(i > 1 && CheckEnd(i,ColumnMax*7/8))
            RREndFlag = 1;

        if(CheckBlackPixle(i,ColumnMax*11/12) && !RRREndFlag)///86.17
            BlackEndRRR++;
        else if(i > 1 && CheckEnd(i,ColumnMax*11/12))
            RRREndFlag = 1;
    }

    if(!LLLEndFlag)		BlackEndLLL = 0;///����

    if(!LLEndFlag)		BlackEndLL = 0;

    if(!LEndFlag)			BlackEndL = 0;

    if(!MLEndFlag)		BlackEndML = 0;

    if(!MEndFlag)			BlackEndM = 0;

    if(!MREndFlag)		BlackEndMR = 0;

    if(!REndFlag)			BlackEndR = 0;

    if(!RREndFlag)		BlackEndRR = 0;

    if(!RRREndFlag)		BlackEndRRR = 0;

#endif

    EndMArray[4]=EndMArray[3];
    EndMArray[3]=EndMArray[2];
    EndMArray[2]=EndMArray[1];
    EndMArray[1]=EndMArray[0];
    EndMArray[0]=BlackEndM;

    EndMLArray[4]=EndMLArray[3];
    EndMLArray[3]=EndMLArray[2];
    EndMLArray[2]=EndMLArray[1];
    EndMLArray[1]=EndMLArray[0];
    EndMLArray[0]=BlackEndML;

    EndMRArray[4]=EndMRArray[3];
    EndMRArray[3]=EndMRArray[2];
    EndMRArray[2]=EndMRArray[1];
    EndMRArray[1]=EndMRArray[0];
    EndMRArray[0]=BlackEndMR;
    BlackEndMin =MIN(BlackEndL,BlackEndM);//ȡ��ֵ
    BlackEndMin =MIN(BlackEndMin,BlackEndR);
}

//��ʮ�ֽ���ʶ��ͽ������⴦��
void NormalCrossConduct(void)
{
    ///unsigned char i;
    int j=0;
    LastLastCrossFlag=LastCrossFlag;//��¼���ϴ��Ƿ���ʮ��
    LastCrossFlag=CrossFlag;//��¼��һ���Ƿ���ʮ��
    CrossFlag=0;//����

    if((AllLose>=5))//����ͬʱ����
    {
        CrossFlag=1;//���ʮ��

        Road_vanish = 1;///�ж�����·����·��ʧ��
    }
    if((AllLose<=1))//���Ҳ�ͬʱ����
    {
        CrossFlag=0;//������ʮ��

        Road_vanish = 0;///�������·��·��ʧ��־

        if(CrossNumber>3) LoseCrossNumber++;       //�������ʮ�ּ������е��������
    }
    if(CrossFlag && LastCrossFlag && LastLastCrossFlag)  //�������Ҷ���
    {
        CrossNumber++;                                    //ʮ�ּ���
    }
    if(LoseCrossNumber>3)                                //�Ѿ�����Բ����
    {
        CrossEnterFlag = 1;                               //��Բ����־λ����
        CrossNumber = 0;                                  //Բ��ͳ�����

    }
    if(CrossEnterFlag && CrossNumber>3)                  //��Բ��
    {

    }

    if(CrossFlag)
    {
        CrossKL = (float)((LeftEdge[FirstLoseAllLine] - LeftEdge[LastLoseAllLine]))/(FirstLoseAllLine - LastLoseAllLine);
        CrossKR = (float)((RightEdge[FirstLoseAllLine] - RightEdge[LastLoseAllLine]))/(FirstLoseAllLine - LastLoseAllLine);

        for(j=FirstLoseAllLine-1; j>LastLoseAllLine; j--)
        {
            Pixle[j][MiddleLine[j]] = White_Point;
            MiddleLine[j]=(((int)((j-FirstLoseAllLine)*CrossKL)+LeftEdge[FirstLoseAllLine])+
                           ((int)((j-FirstLoseAllLine)*CrossKR)+RightEdge[FirstLoseAllLine]))/2;
        }
    }
}

/**
 * @file		������ڽ��д���
 * @note     	�ж��������֮�������ﴦ��
 * @date		2021��2��24��
 */
void Spurroad_conduct(void)///1���ж��Ƿ��·���2���ж��Ƿ������������ߣ�3���ж�ǰհ�Ƿ����ÿ����ֵ��
///4����ǰհ����������һ�߲��߻���ֱ����ĳһ����ת��֮�����������Ѱ���ˡ�
{
    /// unsigned char i;
    int j=0,k=0;
    int L1,L2,R1,R2;
    LastLastSpurroadFlag=LastSpurroadFlag;//��¼���ϴ��Ƿ���ʮ��
    LastSpurroadFlag=SpurroadFlag;//��¼��һ���Ƿ���ʮ��
    //SpurroadFlag=0;//����

    if(Road_widening == 1 && Road_vanish == 1 && (AvaliableLines<20))
    {
        Road_widening = 0;
        Road_vanish = 0;
        SpurroadFlag=1;
    }

    if(SpurroadFlag)
    {
        SpurroadKL = (float)((LeftEdge[FirstLoseAllLine] - LeftEdge[LastLoseAllLine]))/(FirstLoseAllLine - LastLoseAllLine);
        SpurroadKR = (float)((RightEdge[FirstLoseAllLine] - RightEdge[LastLoseAllLine]))/(FirstLoseAllLine - LastLoseAllLine);
        for(j=FirstLoseAllLine-1; j>LastLoseAllLine; j--)
        {
            Pixle[j][MiddleLine[j]] = White_Point;
            MiddleLine[j]=(((int)((j-FirstLoseAllLine)*SpurroadKL)+LeftEdge[FirstLoseAllLine])+
                           ((int)((j-FirstLoseAllLine)*SpurroadKR)+RightEdge[FirstLoseAllLine]))/2;
        }
    }

}

/**
 * @file		�Ի������д���
 * @note     	�ж��ǻ���֮�������ﴦ��
 */
void CircleConduct(void)
{
    static int ImgCount=0,count=0,LastRight=0;
    int16 mincol=0;
    int num1[ColumnMax];

    if((LeftCircleFlag >= ShouldBeCircle || RightCircleFlag >= ShouldBeCircle))
        SpeedParm = SelectMode;
    if(LeftCircleFlag == ShouldBeCircle && RightCircleFlag == ShouldBeCircle)
    {
        if(LeftLose > 15)///��������������15
        {
            LeftCircleFlag = MustBeCircle;
            RightCircleFlag = NoCircle;
            DrawOvalFlag=1;
            CircleDistance = 0;
        }
        else if(RightLose > 15)
        {
            RightCircleFlag = MustBeCircle;
            LeftCircleFlag = NoCircle;
            DrawOvalFlag=1;
            CircleDistance = 0;
        }
        else if(CircleDistance>50)///Բ���������50
        {
            LeftCircleFlag = NoCircle;
            RightCircleFlag = NoCircle;
            CircleDistance = 0;
            SpeedParm = SelectMode;
        }
    }

    /*******************��********************/
    if(LeftCircleFlag == MustBeCircle)
    {
        if(Cirlce_Angle>45)
        {
            SpeedParm = SelectMode;
            LeftOvalFlag = 0;
            LeftCircleFlag = InCircle;
            ImgCount=count = 0;
        }
    }
    /*****************************ȷ���뻷**********************************/
    else if(LeftCircleFlag == InCircle)
    {

        if(Cirlce_Angle>140)
        {
            ImgCount=count=0;
            LeftCircleFlag = OutCircle;					//���Ҷ����߳���ʮ����ʼ����
        }
    }
    /*****************************�жϳ���**********************************/
    else if(LeftCircleFlag == OutCircle)
    {
        if(Cirlce_Angle>240)
        {
            DrawOvalFlag=0;
            LeftCircleFlag = StopCheckCircle;	//�������
            ImgCount=count=0;    	//����Ϊ100��ͼ���ڸ��·������
        }
    }
    /********************�����ɹ�������ʱ����ɨ��־λ**************************/
    else if(LeftCircleFlag == StopCheckCircle)
    {
        if(OutCircleDistance*2>=CircleDistance)
        {
            OutCircleDistance = CircleDistance = 0;///���OutCircleDistance����û������
            ImgCount=count=0;
            LeftCircleFlag = NoCircle;    //һ��֮��Ͳ�����
        }
    }
    /*******************               �һ�               ********************/
    /*****************************׼������***********************************/
    if(RightCircleFlag == MustBeCircle)
    {
        if(Cirlce_Angle<-45)
        {
            SpeedParm = SelectMode;
            RightCircleFlag = InCircle;
            RightOvalFlag = 0;
            ImgCount=count = 0;
        }
    }
    /*****************************ȷ���뻷**********************************/
    else if(RightCircleFlag == InCircle)
    {
        if(Cirlce_Angle<-140)
        {
            ImgCount=count=0;
            RightCircleFlag = OutCircle;					//���Ҷ����߳���ʮ����ʼ����
        }
    }
    /*****************************�жϳ���**********************************/
    else if(RightCircleFlag == OutCircle)
    {
        if(Cirlce_Angle<-240)
        {
            DrawOvalFlag=0;
            RightCircleFlag = StopCheckCircle;	//�������
            ImgCount=count=0;    	//����Ϊ100��ͼ���ڸ��·������
        }
    }
    /********************�����ɹ�������ʱ����ɨ��־λ**************************/
    else if(RightCircleFlag == StopCheckCircle)
    {
        if(OutCircleDistance*2>=CircleDistance)
        {
            OutCircleDistance = CircleDistance = 0;
            ImgCount=count=0;
            RightCircleFlag = NoCircle;   //һ��֮��Ͳ�����
        }
    }
    ImgCount++;

    if(ImgCount>1500)
    {
        ImgCount=0;
    }
}

/**
 * @file		�����߼��
 * @note     	�ж��ǲ���������
 
 */
void StartCheck(void)///int StartCheck(void)
{
    for(uint32 i =StartLine; i>FinishLine; i--) ///��57��19 
    {
        if(CountRowB2WSalation(i,1,ColumnMax-1)>5 && CountRowW2BSalation(i,1,ColumnMax-1)>5)///��������ɨ��
        //CountRowB2WSalation��return�ǴӺ����ұ�һ��ͻ�䵽�׵ĵ�ĸ���
        //CountRowW2BSalation���෴
        //��������������ʱ�����ɫ�ĵ�·�м���һ�ڿ������
        {
            StartLineCount++;
        }
    }
}

/**
 * @file		��Ѱ��������Բ��ʼ����
 * @note     	�Խ��뻷������������Ϊ���ݻ���Բ
 
 */
int16 FindOvalPoint(int16 col,int16 flag)///��������һ�뿪ʼ
{
    int16 i,j,num=0;
    int16 MREndFlag=0,REndFlag=0,RREndFlag=0;
    int16 MLEndFlag=0,LEndFlag=0,LLEndFlag=0;
    int16 CheckMR=0,CheckR=0,CheckRR=0;
    int16 CheckML=0,CheckL=0,CheckLL=0;

    if(flag==1)
    {
        for(i=30; i>15; i--) ///��30��16
        {
            for(j=col; j<=ColumnMax-1; j++) ///��col��93
            {
                if(CheckWhitePixle(i,j) &&
                        Image_Use[i+2,j]>=Image_Use[i,j] &&
                        Image_Use[i+4,j]>=Image_Use[i+2,j])
                {
                    num++;///�ӣ�i��j������i+4��j����Ϊ��ɫ��Խ��Խ�ף���num�Լ�
                }
            }
            if(num>15)		return i;
            else			num = 0;
        }
        return 20;
    }

    else if(flag==2)///ԭ����ע�͵���
    {
        for (i = col; i >10; i--)
        {
            if(CheckBlackPixle(i,ColumnMax*5/8) && !MREndFlag )//10
                CheckMR++;
            else if(i > 1 && CheckEnd(i,ColumnMax*5/8))
                MREndFlag = 1;
            if(CheckBlackPixle(i,ColumnMax*6/8) && !REndFlag )//10
                CheckR++;
            else if(i > 1 && CheckEnd(i,ColumnMax*6/8))
                REndFlag = 1;
            if(CheckBlackPixle(i,ColumnMax*7/8) && !RREndFlag )//10
                CheckRR++;
            else if(i > 1 && CheckEnd(i,ColumnMax*7/8))
                RREndFlag = 1;
        }
        num = MAX(MAX(CheckMR,CheckR),CheckRR);
        num = RowMax-num;
        num=RANGE(num,25,30);
        MREndFlag = 0;
        REndFlag = 0;
        RREndFlag = 0;
        CheckMR = 0;
        CheckR = 0;
        CheckRR = 0;
        return num;

    }
    else if(flag==3)
    {
        for(i=30; i>=15; i--)
        {
            for(j=1; j<=col; j++)
            {
                if(CheckWhitePixle(i,j) &&
                        Image_Use[i+2,j]>=Image_Use[i,j] &&
                        Image_Use[i+4,j]>=Image_Use[i+2,j])
                {
                    num++;
                }
            }
            if(num>15)		return i;
            else			num = 0;
        }
        return 20;
    }
    else if(flag==4)
    {
        for (i = col; i >10; i--)
        {
            if(CheckBlackPixle(i,ColumnMax*3/8) && !MLEndFlag )//10
                CheckML++;
            else if(i > 1 && CheckEnd(i,ColumnMax*3/8))
                MLEndFlag = 1;
            if(CheckBlackPixle(i,ColumnMax*2/8) && !LEndFlag )//10
                CheckL++;
            else if(i > 1 && CheckEnd(i,ColumnMax*2/8))
                LEndFlag = 1;
            if(CheckBlackPixle(i,ColumnMax*1/8) && !LLEndFlag )//10
                CheckLL++;
            else if(i > 1 && CheckEnd(i,ColumnMax*1/8))
                LLEndFlag = 1;
        }
        num = MAX(MAX(CheckML,CheckL),CheckLL);
        num = RowMax-num;
        num=RANGE(num,25,30);
        MLEndFlag = 0;
        LEndFlag = 0;
        LLEndFlag = 0;
        CheckML = 0;
        CheckL = 0;
        CheckLL = 0;
        return num;
    }
}

void DrawLine(int16 x0,int16 y0,int16 x1,int16 y1)
{
    int16 x,y,xx;
    float k;
    x0=RANGE16(x0,2,RowMax-1);
    x1=RANGE16(x1,2,RowMax-1);
    y0=RANGE16(y0,2,ColumnMax-2);
    y1=RANGE16(y1,2,ColumnMax-2);
    k=(float)(y1-y0)/(x1-x0)*1.0;
    for(x=x0; x>x1; x--)
    {
        y=(int16)((float)(x-x0)*k+y0);
        Image_Sobel[x  ][y  ]=255;
        Image_Sobel[x  ][y-1]=255;
        Image_Sobel[x  ][y-2]=255;
        Image_Sobel[x-1][y  ]=255;
        Image_Sobel[x-1][y-1]=255;
        Image_Sobel[x-1][y-2]=255;
        Image_Sobel[x-2][y  ]=255;
        Image_Sobel[x-2][y-1]=255;
        Image_Sobel[x-2][y-2]=255;

        for(xx=x-3; xx>=0; xx--)
        {
            Image_Sobel[xx][y  ]=0;
            Image_Sobel[xx][y-1]=0;
            Image_Sobel[xx][y-2]=0;
        }

    }
}//�ҷ�������Ǵӣ�x0��y0��������x1��y1����kΪб�ʻ�һ����ɫ�߽磬�ٽ�������ൽ0�����ص��Ϊ��ɫ��


void DrawLine1(int16 y0,int16 x0,int16 y1,int16 x1)
{
    int16 x,y;
    float k;
    x0=RANGE16(x0,2,ColumnMax-2);
    x1=RANGE16(x1,2,ColumnMax-2);
    y0=RANGE16(y0,2,RowMax-1);
    y1=RANGE16(y1,2,RowMax-1);
    k=(float)(y1-y0)/(x1-x0)*1.0;
    for(x=x0; x>x1; x--)
    {
        y=(int16)((float)(x-x0)*k+y0);
        ips200_drawpoint(x, y, RED);
    }
}

uint32 CountRowW2BSalation(uint32 line,uint32 start,uint32 finish)
{
    uint32 num=0;
    start=RANGE16(start,0,ColumnMax-1);
    finish=RANGE16(finish,0,ColumnMax-1);
    for(int j=start; j<finish; j++)
    {
        if(CheckWhitePixle(line,j) && CheckBlackPixle(line,j+1) )//&& CheckBlackPixle(line,j+2))
            num++;
    }
    return num;
}

uint32 CountRowB2WSalation(uint32 line,uint32 start,uint32 finish)
{
    uint32 num=0;
    start=RANGE16(start,0,ColumnMax-1);
    finish=RANGE16(finish,0,ColumnMax-1);
    for(int j=start; j<finish; j++)
    {
        if(CheckBlackPixle(line,j) && CheckWhitePixle(line,j+1) )//&& CheckWhitePixle(line,j+2))
            num++;
    }
    return num;
}

/***************************************************************
* �������ƣ�void Draw_Road(void)
***************************************************************/
void Draw_Road(void)
{
    int16 i = 0, j = 0,temp=0;
    char txt[10];
    ips200_address_set(0,200,ColumnMax-1,200+RowMax-1);//������ʾ����
    for(i=0; i<RowMax; i++) //6*8=48��
    {
        for(j=0; j<ColumnMax; j++) //����
        {
            temp=0;
            if(Pixle[i][j])
                ips200_wr_data16(0x0000);
            else
                ips200_wr_data16(0xFFFF);
        }
    }
    for(i=0; i<RowMax; i++) //�������ѭ����ʾ
    {
        ips200_drawpoint((int16)AverageCenter, i, RED);//������
    }
}

/***************************************************************
* �������ƣ�void Get_01_Value(void)
* ����˵�������վ�ֵ�ı������ж�ֵ��
* ����˵���� limit-��ֵ����ֵ��height/width���������С
***************************************************************/
void Get_01_Value(int limit,int height,int width)
{
    int i = 0,j = 0;

   for(i = 0; i < height; i++)
   {
       for(j = 0; j < width; j++)
       {
           if(Image_Sobel[i][j] >limit)
               Pixle[i][j] =1;
           else if(Image_Sobel[i][j] <=limit)
               Pixle[i][j]=0;
        //�ڵ�Ϊ0���Ҷ�ֵ��ͣ����׵㷴֮
       }
   }
}

/***************************************************************
* �������ƣ�void Pixle_Filter(void)
* ����˵������sobelͼ�����������ֵ�޳�
* ����˵����
* �������أ�
* �޸�ʱ�䣺2019
* �� ע��
***************************************************************/
void Pixle_Filter(int16 threshold)
{
    int i,j;
    for(i=StartLine+1; i>=FinishLine-1; i--) ///18��58
    {
        for(j=1; j<ColumnMax-1; j++) ///1��93
        {
            if(CheckWhitePixle(i,j) && Image_Use[i][j]>threshold)
                Image_Sobel[i][j] = 0;///�����ҵ���ⲻӦ������Ϊ0��ɫ///�����㷨

            if(	CheckBlackPixle(i-1,j-1)&&
                    CheckBlackPixle(i-1,j  )&&
                    CheckBlackPixle(i-1,j+1)&&
                    CheckBlackPixle(i  ,j-1)&&
                    CheckBlackPixle(i  ,j+1)&&
                    CheckWhitePixle(i  ,j  )&&
                    CheckBlackPixle(i+1,j-1)&&
                    CheckBlackPixle(i+1,j  )&&
                    CheckBlackPixle(i+1,j+1))
                Image_Sobel[i][j] = 0;///�Һڵ� 3*3
            //�׵㱻�ڵ��Χʱ����������Ϊ�ڵ㣨��ͽ������㷨�𣿣�
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  @name    	Sobel
//  @brief   	Sobel����ͼ���Ե��ȡ
//				sobel���Ӱ�������3x3�ľ��󣬷ֱ�Ϊ��������
//				֮��ͼ����ƽ���������ɷֱ�ó�������������Ȳ�ֽ���ֵ��
//  @param		start-��ʼ�У�finish-��ֹ��
//  @return     ��õ���ֵ
//  @since      v1.0
//  @note
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
int16 Sobel(int16 start,int16 finish)
{
    int tempx=0,tempy=0,temp=0,i=0,j=0,k=0;
    long long int g_sum=0;
    start=RANGE16(start,1,RowMax-1);///1��59����
    finish=RANGE16(finish,1,RowMax-1);
    for(i=start; i <finish; i++) ///�ӿ�ʼ������
    {
        for(j=1; j<ColumnMax-1; j++) ///1��93����
        {
            //��x��������ӽ��о��
            tempx=(-  	Image_Use[i-1][j-1])
                  +(-2*Image_Use[i ][j-1])
                  +(-  Image_Use[i+1][j-1])
                  +(   Image_Use[i-1][j+1])
                  +( 2*Image_Use[i  ][j+1])
                  +(   Image_Use[i+1][j+1]);
            if(tempx<0)		tempx=-tempx;

            //��y��������ӽ��о��
            tempy=(		Image_Use[i+1][j-1])
                  +( 2*Image_Use[i+1][j  ])
                  +(   Image_Use[i+1][j+1])
                  +(-  Image_Use[i-1][j-1])
                  +(-2*Image_Use[i-1][j  ])
                  +(-  Image_Use[i-1][j+1]);
            if(tempy<0)		tempy=-tempy;

            //��xy������Ľ��������ӣ��õ��õ�ĻҶ�ֵ�������趨����ֵ���бȽϣ��ж����Ƿ�Ϊ��Ե��
            temp=tempx+tempy;
            if(temp>255)	temp=255;

            g_sum+=temp;
            k++;
            Image_Sobel[i][j]=temp;
        }
    }
    g_sum=2*g_sum/k;//��ֵ��ȡ

    return((int16)g_sum);
}

/***************************************************************
* �������ƣ�void BinaryImage(uint32 tmImage[IMAGEH][IMAGEW])
* ����˵����ͼ�����ݶ�ֵ��
***************************************************************/
void BinaryImage(uint32 tmImage[IMAGEH][IMAGEW],uint32 ThresholdV)
{
    int i = 0, j = 0;
    for(i = 0; i < IMAGEH; i++)
    {
        for(j = 0; j< IMAGEW; j++)
        {
            if(tmImage[i][j] >= ThresholdV)
            {
                tmImage[i][j] = 0xFE;
            }
            else
            {
                tmImage[i][j] = 0X00;
            }
        }
    }
}

/***************************************************************
* �������ƣ� void Get_Use_Image(void)
* ����˵������ȡ��Ҫ��ͼ������
***************************************************************/
void Get_Use_Image(void)
{
    int i = 0,j = 0,row = 0,line = 0;///row�Ǹߣ�line�ǿ�

    for(i = 0; i  < IMAGEH; i+=2)  //120�У�ÿ2�вɼ�һ�У�///ֻȡԭͼ���1/2
    {
        for(j = 0; j < IMAGEW; j+=2) //188��
        {
            Image_Use[row][line] = mt9v03x_image[i][j];
            line++;
        }
        line = 0;
        row++;
    }
}

//**********�������ֵ*****************//
int16 adapt_otsuThreshold(int16 *image, int16 col, int16 row)   //ע�������ֵ��һ��Ҫ��ԭͼ��
{
   #define GrayScale 256
    int16 width = col;
    int16 height = row;
    int pixelCount[GrayScale];
    float pixelPro[GrayScale];
    int i, j, pixelSum = width * height/4;
    int16 threshold = 0;
    int16* data = image;  //ָ���������ݵ�ָ��
    for (i = 0; i < GrayScale; i++)
    {
        pixelCount[i] = 0;
        pixelPro[i] = 0;
    }
    
    int16 gray_sum=0;
    //ͳ�ƻҶȼ���ÿ������������ͼ���еĸ���  
    for (i = 0; i < height; i+=2)
    {
        for (j = 0; j < width; j+=2)
        {
            pixelCount[(int)data[i * width + j]]++;  //����ǰ�ĵ������ֵ��Ϊ����������±�
            gray_sum+=(int)data[i * width + j];       //�Ҷ�ֵ�ܺ�
        }
    }
                      
    //����ÿ������ֵ�ĵ�������ͼ���еı���  
  
    for (i = 0; i < GrayScale; i++)
    {
        pixelPro[i] = (float)pixelCount[i] / pixelSum;
        
    }

    //�����Ҷȼ�[0,255]  
    float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;
    
    w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;
    for (j = 0; j < GrayScale; j++)         
    {
    
        w0 += pixelPro[j];  //��������ÿ���Ҷ�ֵ�����ص���ռ����֮��   ���������ֵı���
        u0tmp += j * pixelPro[j];  //�������� ÿ���Ҷ�ֵ�ĵ�ı��� *�Ҷ�ֵ 
    
        w1=1-w0;
        u1tmp=gray_sum/pixelSum-u0tmp;

        u0 = u0tmp / w0;              //����ƽ���Ҷ�
        u1 = u1tmp / w1;              //ǰ��ƽ���Ҷ�
        u = u0tmp + u1tmp;            //ȫ��ƽ���Ҷ�
        deltaTmp = w0 * pow((u0 - u), 2) + w1 * pow((u1 - u), 2);
        if (deltaTmp > deltaMax)
        {
            deltaMax = deltaTmp;
            threshold = j;
        }
        if (deltaTmp < deltaMax)
        {
            break;
        }
        
    }
    return threshold;
}

void image_update_thresvalue()
{
	static uint8 last_threshold_value = 0;
	if (last_threshold_value != g_thres_value)
	{
		for (uint16 i = 0; i < g_thres_value; i++)
		{
			thresvalue[i] = 0;
		}
		for (uint16 i = g_thres_value; i < 256; i++)
		{
			thresvalue[i] = 1;
		}
		last_threshold_value = g_thres_value;
	}
}

// ���ٶ�ֵ��
void image_fast_threshold()
{
	uint8* src = g_image[0];
	uint8* dst = g_mask[0];
	uint16 count = IMAGE_HEIGHT * IMAGE_WIDTH;

	while (count--)
	{
		// ʹ�������������������жϣ�����һ��
		*dst++ = thresvalue[*src++];
	}
}

// ��ʼ�����Ȩ��
void image_init_err_weight()
{
	g_weight_sum = 0;
	for (uint16 y = SEARCH_BOT; y >= SEARCH_TOP; y -= SEARCH_STEP_Y)//91��
	{
		//g_err_weight[y] = 1;
		g_weight_sum += g_err_weight[y];
	}
}

// �������
void image_calcu_err()
{
	int16 err_sum = 0;
	//static float last_image_err = 0.0;

	for (uint16 y = SEARCH_BOT; y >= SEARCH_TOP; y -= SEARCH_STEP_Y)
	{
		err_sum += g_err_weight[y] * (g_mid_line[y] - IMAGE_WIDTH / 2);
	}

	g_image_err = -err_sum / (float)g_weight_sum;
}

// Ѱ������
void image_find_midline(uint8 mode)
{
	if (mode == MIDLINE_MAX)//���е��߶�����ģʽ
	{
		for (uint16 y = SEARCH_BOT; y >= SEARCH_TOP; y -= SEARCH_STEP_Y)
		{
			if (g_left_line[y] && g_right_line[y])
			{
				g_mid_line[y] = (g_right_line[y] + g_left_line[y]) >> 1;

			}
			else if (g_left_line[y])
			{
				g_mid_line[y] = (g_left_line[y] + SEARCH_RIGHT) >> 1;
			}
			else if (g_right_line[y])
			{
				g_mid_line[y] = (g_right_line[y] + SEARCH_LEFT) >> 1;
			}
			else
			{
				g_mid_line[y] = IMAGE_WIDTH / 2;
			}
		}
	}
	else if (mode == MIDLINE_MIN)
	{
		for (uint16 y = SEARCH_BOT; y >= SEARCH_TOP; y -= SEARCH_STEP_Y)
		{
			if (g_left_line[y] && g_right_line[y])
			{
				g_mid_line[y] = (g_right_line[y] + g_left_line[y]) >> 1;
			}
			else
			{
				g_mid_line[y] = 0;
			}
		}
	}
	else if (mode == MIDLINE_LEFT)
	{
		for (uint16 y = SEARCH_BOT; y >= SEARCH_TOP; y -= SEARCH_STEP_Y)
		{
			if (g_left_line[y] && g_right_line[y])
			{
				g_mid_line[y] = (g_right_line[y] + g_left_line[y]) >> 1;
			}
			else if (g_right_line[y])
			{
				g_mid_line[y] = (g_right_line[y] + SEARCH_LEFT) >> 1;
			}
			else
			{
				g_mid_line[y] = 0;
			}
		}
	}
	else if (mode == MIDLINE_RIGHT)
	{
		for (uint16 y = SEARCH_BOT; y >= SEARCH_TOP; y -= SEARCH_STEP_Y)
		{
			if (g_left_line[y] && g_right_line[y])
			{
				g_mid_line[y] = (g_right_line[y] + g_left_line[y]) >> 1;
			}
			else if (g_left_line[y])
			{
				g_mid_line[y] = (g_left_line[y] + SEARCH_RIGHT) >> 1;
			}
			else
			{
				g_mid_line[y] = 0;
			}
		}
	}
}

// ԭͼ�ж�ʮ��·��
uint16 image_judge_cross()
{
	g_cross.count = 0;

	for (uint16 y = SEARCH_BOT; y >= SEARCH_TOP; y -= SEARCH_STEP_Y)
	{
		// ��¼δ�ѵ�������
		g_cross.count += !(g_left_line[y] | g_right_line[y]);
	}

	if (g_cross.count > g_cross_count_thres)
	{
		g_cross.flag = 1;
	}
	else
	{
		g_cross.flag = 0;
	}

	return g_cross.flag;
}

// ���߻ع�
void image_midline_regression()
{
	uint16 y_sum = 0;
	uint16 x_sum = 0;
	uint32 xy_sum = 0;
	uint16 n_sum = 0;
	uint32 x2_sum = 0;

	// ��С���˷����
	for (uint16 y = SEARCH_BOT; y >= SEARCH_TOP; y -= SEARCH_STEP_Y)
	{
		// ֻѡȡ��Ч����
		if (g_mid_line[y])
		{
			x_sum += y;
			y_sum += g_mid_line[y];
			xy_sum += y * g_mid_line[y];
			n_sum ++;
			x2_sum += y * y;
		}
	}

	// �����Ч���ݹ���
	if (n_sum < REGRESSION_MIN_N)
	{
		// ʹ��Ĭ��������
		// TODO: �Ż��ȶ�
		for (uint16 y = SEARCH_BOT; y >= SEARCH_TOP; y -= SEARCH_STEP_Y)
		{
			if (!g_mid_line[y])
			{
				g_mid_line[y] = IMAGE_WIDTH / 2;
			}
		}
	}
	
	float x_mean = x_sum / (float)n_sum;
	float k = (xy_sum - x_mean * y_sum) / (x2_sum - x_sum * x_mean);
	float b = y_sum / (float)n_sum - k * x_mean;

	// ��Ч���ݲ�ֵ
	for (uint16 y = SEARCH_BOT; y >= SEARCH_TOP; y -= SEARCH_STEP_Y)
	{
		if (!g_mid_line[y])
		{
			// ��ӷ�ֵ���ƣ���ֹԽ��
			int16 mid_line = (int16)(k * y + b);
			g_mid_line[y] = LIMITER(mid_line, SEARCH_LEFT, SEARCH_RIGHT);
		}
	}
}

// ���߻ع�
void image_leftline_regression()
{
	uint16 y_sum = 0;
	uint16 x_sum = 0;
	uint32 xy_sum = 0;
	uint16 n_sum = 0;
	uint32 x2_sum = 0;

	for (uint16 y = SEARCH_BOT; y >= SEARCH_TOP; y -= SEARCH_STEP_Y)
	{
		if (g_left_line[y])
		{
			x_sum += y;
			y_sum += g_left_line[y];
			xy_sum += y * g_left_line[y];
			n_sum ++;
			x2_sum += y * y;
		}
	}

	if (n_sum < REGRESSION_MIN_N)
	{
		for (uint16 y = SEARCH_BOT; y >= SEARCH_TOP; y -= SEARCH_STEP_Y)
		{
			if (!g_left_line[y])
			{
				g_left_line[y] = IMAGE_WIDTH / 4;
			}
		}
	}
	
	double x_mean = x_sum / (float)n_sum;
	double k = (xy_sum - x_mean * y_sum) / (x2_sum - x_sum * x_mean);
	double b = y_sum / (float)n_sum - k * x_mean;

	for (uint16 y = SEARCH_BOT; y >= SEARCH_TOP; y -= SEARCH_STEP_Y)
	{
		if (!g_left_line[y])
		{
			int16 left_line = (int16)(k * y + b);
			g_left_line[y] = LIMITER(left_line, SEARCH_LEFT, SEARCH_RIGHT);
		}
	}
}

// ���߻ع�
void image_rightline_regression()
{
	uint16 y_sum = 0;
	uint16 x_sum = 0;
	uint32 xy_sum = 0;
	uint16 n_sum = 0;
	uint32 x2_sum = 0;

	for (uint16 y = SEARCH_BOT - 20; y >= SEARCH_TOP; y -= SEARCH_STEP_Y)
	{
		if (g_right_line[y])
		{
			x_sum += y;
			y_sum += g_right_line[y];
			xy_sum += y * g_right_line[y];
			n_sum ++;
			x2_sum += y * y;
		}
	}

	if (n_sum < REGRESSION_MIN_N)
	{
		for (uint16 y = SEARCH_BOT; y >= SEARCH_TOP; y -= SEARCH_STEP_Y)
		{
			if (!g_right_line[y])
			{
				g_right_line[y] = 3 * IMAGE_WIDTH / 4;
			}
		}
	}
	
	double x_mean = x_sum / (float)n_sum;
	double k = (xy_sum - x_mean * y_sum) / (x2_sum - x_sum * x_mean);
	double b = y_sum / (float)n_sum - k * x_mean;

	for (uint16 y = SEARCH_BOT; y >= SEARCH_TOP; y -= SEARCH_STEP_Y)
	{
		if (!g_right_line[y])
		{
			int16 right_line = (int16)(k * y + b);
			g_right_line[y] = LIMITER(right_line, SEARCH_LEFT, SEARCH_RIGHT);
		}
	}
}

// ����
void image_debug()
{
	memcpy(g_watch, g_mask, IMAGE_HEIGHT * IMAGE_WIDTH); // 150ns

	for (uint16 y = SEARCH_BOT; y >= SEARCH_TOP; y -= 3)
	{
		g_watch[y][g_mid_x] = 5;
	}
	for (uint16 y = SEARCH_BOT; y >= SEARCH_TOP; y -= SEARCH_STEP_Y)
	{
		if (g_left_line[y])
		{
			g_watch[y][g_left_line[y]] = 3;
			g_watch[y][g_left_line[y] + 1] = 3;
		}
		if (g_right_line[y])
		{
			g_watch[y][g_right_line[y]] = 4;
			g_watch[y][g_right_line[y] - 1] = 4;
		}
		if (g_mid_line[y])
		{
			g_watch[y][g_mid_line[y]] = 2;
			g_watch[y][g_mid_line[y] + 1] = 2;
		}
	}
	for (uint16 x = 0; x < IMAGE_WIDTH; x += 3)
	{
		g_watch[SEARCH_TOP][x] = 5;
		g_watch[SEARCH_BOT][x] = 5;
	}
}

void image_debug_red()
{
	memcpy(g_watch, g_mask, IMAGE_HEIGHT * IMAGE_WIDTH); // 150ns

	for (uint16 y = SEARCH_BOT; y >= SEARCH_TOP; y -= 3)
	{
		g_watch[y][g_mid_x] = 4;
	}
	for (uint16 y = SEARCH_BOT; y >= SEARCH_TOP; y -= SEARCH_STEP_Y)
	{
		if (g_left_line[y])
		{
			g_watch[y][g_left_line[y]] = 4;
			g_watch[y][g_left_line[y] + 1] = 4;
		}
		if (g_right_line[y])
		{
			g_watch[y][g_right_line[y]] = 4;
			g_watch[y][g_right_line[y] - 1] = 4;
		}
		if (g_mid_line[y])
		{
			g_watch[y][g_mid_line[y]] = 4;
			g_watch[y][g_mid_line[y] + 1] = 4;
		}
	}
	for (uint16 x = 0; x < IMAGE_WIDTH; x += 3)
	{
		g_watch[SEARCH_TOP][x] = 4;
		g_watch[SEARCH_BOT][x] = 4;
	}
}


// ����OTSU�����ֵ����ԭͼ��ѹ��10��������ʱ��̣�����ÿ֡������һ�Σ�
uint16 image_fast_otsu()
{
	uint16 histogram[DOWNSAMPLE_S] = { 0 };
	uint16 histstd[DOWNSAMPLE_S] = { 0 };

	// ֱ��ͼͳ��
	for (uint16 y = SEARCH_TOP; y <= SEARCH_BOT; y += DOWNSAMPLE_Y)
	{
		for (uint16 x = SEARCH_LEFT; x <= SEARCH_RIGHT; x += DOWNSAMPLE_X)
		{
			histogram[g_image[y][x] / DOWNSAMPLE_C]++;
		}
	}

	uint16 background_std = 0;
	uint16 foreground_std = 0;
	uint16 background_mean = 0;
	uint16 foreground_mean = 0;
	uint16 background_sum = 0;
	uint16 foreground_sum = 0;
	uint16 background_n = 0;
	uint16 foreground_n = 0;
	uint16 i = g_thres_min / DOWNSAMPLE_C;

	// ��ʼ�����ֱ�
	for (uint16 k = 0; k < i; k++)
	{
		background_n += histogram[k];
		background_sum += histogram[k] * k;
	}

	for (uint16 k = i; k < DOWNSAMPLE_S; k++)
	{
		foreground_n += histogram[k];
		foreground_sum += histogram[k] * k;
	}

	// �����������ڷ���
	for (; i <= g_thres_max / DOWNSAMPLE_C; i++)
	{
		background_n += histogram[i];
		foreground_n -= histogram[i];
		background_sum += histogram[i] * i;
		foreground_sum -= histogram[i] * i;

		if (!background_n)
		{
			continue;
		}
		if (!foreground_n)
		{
			break;
		}

		background_mean = background_sum / background_n;
		foreground_mean = foreground_sum / foreground_n;

		background_std = 0;
		for (int16 k = 0; k <= i; k++)
		{
			background_std += histogram[k] * (k - background_mean) * (k - background_mean);
		}
		background_std = background_std / background_n;

		foreground_std = 0;
		for (int16 k = i + 1; k < DOWNSAMPLE_S; k++)
		{
			foreground_std += histogram[k] * (k - foreground_mean) * (k - foreground_mean);
		}
		foreground_std = foreground_std / foreground_n;

		histstd[i] = background_std + foreground_std;
	}

	uint16 temp = 0xffff;
	uint16 thres = 0;
	uint16 thres_n = 0;

	// Ѱ�ҷ�����С�ĻҶȼ�������ж����ȡƽ��
	for (uint16 i = g_thres_min / DOWNSAMPLE_C; i <= g_thres_max / DOWNSAMPLE_C; i++)
	{
		if (histstd[i])
		{
			if (histstd[i] < temp)
			{
				temp = histstd[i];
				thres = i;
				thres_n = 1;
			}
			// TODO�������쳣��
			else if (histstd[i] == temp)
			{
				thres += i;
				thres_n++;
			}
		}
	}

	// ��ֹ��Ч���
	if (thres)
	{
		g_thres_value = thres * DOWNSAMPLE_C / thres_n;
	}

	return 1;
}

// Ѱ������У��ö�ֵ����ͼ������У�
uint16 image_find_whiteline()
{
	uint16 y_sum[DOWNSAMPLE_W + 1] = {0};
	uint16 y_sum_left = 0;
	uint16 y_sum_mid = 0;
	uint16 y_sum_right = 0;
	uint16 mid_x = 0;
	uint16 mid_n = 0;
	uint16 temp = 0;

	// ���� y ���ۼ�
 	for (uint16 x = SEARCH_LEFT; x <= SEARCH_RIGHT; x += DOWNSAMPLE_X)
 	{
 		uint16* sum = &y_sum[(x - SEARCH_LEFT) / DOWNSAMPLE_X];//DOWNSAMPLE_X:10 ��С10��
 		uint8* src = &g_image[0][x];
 		for (uint16 y = SEARCH_TOP; y <= SEARCH_BOT; y += DOWNSAMPLE_Y)
 		{
 			*sum += thresvalue[*(src + IMAGE_WIDTH * y)];
 		}
 	}

	// �ۼӻ���
	for (uint16 x = 0; x < DOWNSAMPLE_W / 3; x++)
	{
		y_sum_left += y_sum[x];
		y_sum_mid += y_sum[x + DOWNSAMPLE_W / 3];
		y_sum_right += y_sum[x + 2 * DOWNSAMPLE_W / 3];
	}
	
	// �жϲ��
	if (y_sum_left > y_sum_mid && y_sum_right > y_sum_mid)
	{
		if (g_fork_direction == DIR_LEFT)
		{
			temp = 0;
			for (uint16 x = 0; x < DOWNSAMPLE_X / 2; x++)
			{
				if (y_sum[x] > temp)
				{
					temp = y_sum[x];
					mid_x = x;
				}
				else if (y_sum[x] == temp)
				{
					mid_x += x;
					mid_n++;
				}
			}
		}
		else
		{
			temp = 0;
			for (uint16 x = DOWNSAMPLE_X / 2; x < DOWNSAMPLE_X; x++)
			{
				if (y_sum[x] > temp)
				{
					temp = y_sum[x];
					mid_x = x;
				}
				else if (y_sum[x] == temp)
				{
					mid_x += x;
					mid_n++;
				}
			}
		}
		mid_x = mid_x / mid_n;
		g_mid_x = mid_x * DOWNSAMPLE_X + SEARCH_LEFT;
		return 0;
	}
	else
	{
		temp = 0;
		for (uint16 x = 0; x <= DOWNSAMPLE_W; x++)
		{
			if (y_sum[x] > temp)
			{
				temp = y_sum[x];
				mid_x = x;
				mid_n = 1;
			}
			else if(y_sum[x] == temp)
			{
				mid_x += x;
				mid_n++;
			}
		}
		mid_x = mid_x / mid_n;
		g_mid_x = mid_x * DOWNSAMPLE_X + SEARCH_LEFT;
		return 1;
	}
}

//���߻����ò�Ⱥ�
uint16 image_fastsearch_leftline()
{
	int16 end_x = SEARCH_LEFT;
	uint16 count = 0;
	uint16 times = 0;
	uint16 diff_thres = 0;
	uint16 find_flag = 0x0000;
	uint16 last_x = g_mid_x;//��Դʱ����ʼ�����㣬����������߿�ʼ
	uint16 margin = SEARCH_MARGIN;
	x_conditon_left = SEARCH_LEFT + 1 * (last_x - SEARCH_LEFT) / 2;

	int16 x;
	uint16 y = SEARCH_BOT;
	while (y >= SEARCH_TOP)
	{
		// ��̬��ֵ��������ֵ��Զ����ֵС����ΪԶ���ĵ�Ҷ�ֵ�ձ��С��
		diff_thres = (g_differ_threshold1 - g_differ_threshold2) * (y - SEARCH_TOP) / (SEARCH_BOT - SEARCH_TOP) + g_differ_threshold2;
		// �����ȫ�ף�������
		if (thresvalue[g_image[y][SEARCH_LEFT]] && thresvalue[g_image[y][x_conditon_left]])
		{
			g_left_line[y] = 0;
			y -= SEARCH_STEP_Y;
			continue;
		}
		// �����������
		else
		{
			times++;
			// ��һ��Ѱ�Ҹ���֮ǰ�Ľ��
			if (times == 1)//��һ������û�ѵ��������ڶ��Σ������Ѿ��ѹ����߲�������
			{
				x = last_x + margin;
				end_x = SEARCH_LEFT;
			}
			// �ڶ���Ѱ������Χ
			else
			{
				x = g_mid_x;
				end_x = last_x + margin + 1;
			}
			// ��ֹ x Խ��
			if (x > SEARCH_RIGHT)
			{
				x = SEARCH_RIGHT;
			}
			// ���Ѱ�ҽ��
			find_flag = 0x0000;
			// ��ʼ����
			for (; x >= end_x; x -= SEARCH_STEP_X)
			{
				// if (thresvalue[g_image[y][x + 1]] && !thresvalue[g_image[y][x]])
				if (FAST_DIFFER((int16)g_image[y][x + 1], (int16)g_image[y][x]) > diff_thres)
				{
					g_left_line[y] = x;
					last_x = x;
					find_flag = 0xffff;
					count++;
					times = 0;
					y -= SEARCH_STEP_Y;
					break;
				}
				else
				{
					continue;
				}
			}
			g_left_line[y] = find_flag & g_left_line[y];
			// �������û�ҵ���������һ��
			if (!find_flag && times == 2)
			{
				times = 0;
				y -= SEARCH_STEP_Y;
				break;
			}
		}
	}

	return count;
}

uint16 image_fastsearch_rightline()
{
	int16 end_x = SEARCH_RIGHT;
	uint16 count = 0;
	uint16 times = 0;
	uint16 diff_thres = 0;
	uint16 last_x = g_mid_x;
	uint16 find_flag = 0x0000;
	uint16 margin = SEARCH_MARGIN;
	x_conditon_right = last_x + 1 * (SEARCH_RIGHT - last_x) / 2;

	int16 x;
	uint16 y = SEARCH_BOT;
	while (y >= SEARCH_TOP)
	{
		diff_thres = (g_differ_threshold1 - g_differ_threshold2) * (y - SEARCH_TOP) / (SEARCH_BOT - SEARCH_TOP) + g_differ_threshold2;
		if (thresvalue[g_image[y][SEARCH_RIGHT]] && thresvalue[g_image[y][x_conditon_right]])
		{
			g_right_line[y] = 0;
			y -= SEARCH_STEP_Y;
			continue;
		}
		else
		{
			times++;
			if (times == 1)
			{
				x = last_x - margin;
				end_x = SEARCH_RIGHT;
			}
			else
			{
				x = g_mid_x;
				end_x = last_x - margin + 1;
			}
			if (x < SEARCH_LEFT)
			{
				x = SEARCH_LEFT;
			}
			find_flag = 0x0000;
			for (; x <= end_x; x += SEARCH_STEP_X)
			{
				//if (thresvalue[g_image[y][x - 1]] && !thresvalue[g_image[y][x]])
				if (FAST_DIFFER((int16)g_image[y][x - 1], (int16)g_image[y][x]) > diff_thres)
				{
					g_right_line[y] = x;
					last_x = x;
					find_flag = 0xffff;
					count++;
					times = 0;
					y -= SEARCH_STEP_Y;
					break;
				}
				else
				{
					continue;
				}
			}
			g_right_line[y] = find_flag & g_right_line[y];
			if (!find_flag && times == 2)
			{
				times = 0;
				y -= SEARCH_STEP_Y;
				break;
			}
		}
	}

	return count;
}

// ֱ���쳣�����(ȥ�����)
uint16 image_line_filter(uint16 line_type)
{
	uint16* lines;
	uint16* count;
	if (line_type == LINE_LEFT)
	{
		lines = g_left_line;
		count = &g_left_count;
	}
	else
	{
		lines = g_right_line;
		count = &g_right_count;
	}


	uint16 cnt = 0;
	// ������ĵ��ѵ�����������ĵ��ѵ���������������С
	if (lines[SEARCH_BOT] && lines[SEARCH_BOT - 1] &&
		ABS_DELT(lines[SEARCH_BOT - 1], lines[SEARCH_BOT]) < SEARCH_MARGIN * 2)
	{
		cnt++;
	}
	else
	{
		lines[SEARCH_BOT] = 0;
	}

	for (uint16 y = SEARCH_BOT - 1; y >= SEARCH_TOP + 1; y--)
	{
		// �������Ч�㣬������
		if (!lines[y])
		{
			continue;
		}
		// ����õ��ǹ����㣬��ɾ��
		if (!lines[y + 1] && !lines[y - 1])
		{
			lines[y] = 0;
			continue;
		}
		// ����õ㲻�ǹ����㣬��������һ����ľ���Ͻ�������
		if (lines[y + 1] && ABS_DELT(lines[y + 1], lines[y]) < SEARCH_MARGIN * 2)
		{
			cnt++;
			continue;
		}
		// ����õ㲻�ǹ����㣬��������һ����ľ���Ͻ�������
		else if (lines[y - 1] && ABS_DELT(lines[y - 1], lines[y]) < SEARCH_MARGIN * 2)
		{
			cnt++;
			continue;
		}
		// ����õ㲻�ǹ����㣬������ǰ��������ľ��붼��Զ�����ֵ����������㣩
		else if(lines[y + 1] && lines[y - 1])
		{
			lines[y] = (lines[y + 1] + lines[y - 1]) >> 1;
			continue;
		}
		// ����õ��Ǳ߽�㣬�����������ľ��붼��Զ�������
		else if(lines[y + 1])
		{
			lines[y] = lines[y + 1];
			continue;
		}
		else
		{
			lines[y] = lines[y - 1];
			continue;
		}
	}

	// ���Զ�ĵ��ѵ�����������ĵ��ѵ���������������С
	if (lines[SEARCH_TOP] && lines[SEARCH_TOP + 1] &&
		ABS_DELT(lines[SEARCH_TOP + 1], lines[SEARCH_TOP]) < SEARCH_MARGIN * 2)
	{
		cnt++;
	}
	else
	{
		lines[SEARCH_TOP] = 0;
	}

	*count = cnt;
	return cnt;
}

// ֱ�߷�������
/*���ã�����Բ������������һ�������ر��һ�������ر�С����һ���ǻ���һ����ֱ�ߣ��ж�ΪԲ��*/
uint16 image_line_analyse(uint16 line_type)
{
	uint16* lines;
	uint16* count;
	uint16* iscurve;

	if (line_type == LINE_LEFT)
	{
		lines = g_left_line;
		count = &g_left_count;
		iscurve = &g_left_curve;
		// ���ֱ��̫�̣����˳�
		if (*count < g_min_count)
		{
			return 0;
		}
	}
	else
	{
		lines = g_right_line;
		count = &g_right_count;
		iscurve = &g_right_curve;
		if (*count < g_min_count)
		{
			return 0;
		}
	}

	uint16 number = 0;
	uint16 y = SEARCH_BOT;
	while (y >= SEARCH_TOP)
	{
		uint16 cnt = 0;
		uint16 y_start = 0;
		uint16 y_end = 0;
		// ����ֱ�ߵĿ�ʼ
		for (; y >= SEARCH_TOP; y -= SEARCH_STEP_Y)
		{
			if (lines[y])
			{
				cnt++;
				y_start = y;
				break;
			}
		}
		// ����ֱ�ߵĽ���
		for (; y >= SEARCH_TOP; y -= SEARCH_STEP_Y)
		{
			if (!lines[y])
			{
				break;
			}
			else
			{
				cnt++;
			}
		}
		// ������Чֱ�߶�
		if (cnt < g_min_count)
		{
			continue;
		}
		else
		{
			number++;
			y_end = y_start - (cnt - 1);
		}

		// ����ֱ�߶�����
		uint16 y0 = y_start;
		uint16 x0 = lines[y0];
		uint16 y2 = y_end;
		uint16 x2 = lines[y2];
		uint16 y1 = (y_start + y_end) >> 1;
		uint16 x1 = lines[y1];
		uint16 dysq = (y_end - y_start) * (y_end - y_start);
		uint16 dis1sq = (x0 - x1) * (x0 - x1) + dysq >> 2;
		uint16 dis2sq = (x2 - x1) * (x2 - x1) + dysq >> 2;
		uint16 dis3sq = (x2 - x0) * (x2 - x0) + dysq;
		float dis1 = sqrtf(dis1sq);
		float dis2 = sqrtf(dis2sq);
		float curve = (dis1sq + dis2sq - dis3sq) / (dis1 * dis2);

		// ��������ߣ���
		if (curve > g_min_curve)
		{
			*iscurve = 1;
		}
	}

	return *iscurve;
}
