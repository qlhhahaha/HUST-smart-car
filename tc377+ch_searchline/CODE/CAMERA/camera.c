/*
 * Dream-Seekers-camera.c
 */

/*三岔口思考:算法尽可能简单，准确。
 * 1，当道路两边的黑线突然变宽（是变宽不是丢线），意味着进入三岔口，先让车向左走，等跑完一圈后，再次判断遇到三岔口向右走。
 * 2，可以观察到所有的道路类型：环岛，十字路等，正前方均没有阻挡（黑线）（起跑线需要特殊处理），所以一遇到变宽或者两边丢线且正前方有黑线就判断为环岛。
 **         （可能会与停车程序冲突，不过要记得车库的特征为四周均为黑线）
 * 3,想办法分辨起跑线的前瞻和三岔路的（前瞻）三角尖（是用前瞻还是另外判断）
 * 4，左进左出，右进右出（判断为三岔口后，方向打左进入三岔口就打左从下一个三岔口（一个三岔口有入口出口两个三岔口）出来，这样才能正确从三岔路出来。）
 */

/*
 * 当左边开始丢线，说明车向右边偏，需要左修正，同理右边亦是如此，可以作为方向修正判断依据。
 */


#include "headfile.h"

int Test = 5;

/*********图像提取相关初始化**********/
volatile uint16 Image_Use[RowMax][ColumnMax];     	 	//压缩后图像数据
volatile uint32 Pixle[RowMax][ColumnMax];    	//二值化后用于OLED显示的数据//
volatile uint16 Image_Sobel[RowMax][ColumnMax];     	//Sobel图像数据(即原始灰度图)
volatile uint16 gray_filter[RowMax][ColumnMax]; //滤波之后的灰度图
/*********图像处理相关初始化**********/
int16 Threshold;               
int16 FilterThreshold = 225;
int16 Camera_scan_finish = 0;
const int Width[RowMax+1]=  	//图像每行直道对应行宽 ///镜头限制高度后可能行宽也要做相应的改变，也就是说数据需要手动更新。
{
    /* 0 */	1,		1,		1,		1,		1,		1,		1,		2,		3,		3,
    /* 1 */	10,		10,		10,		10,		16,		17,		17,		19,		29,		31,
    /* 2 */	33,		35,		37,		39,		41,		42,		44,		46,		48,		50,
    /* 3 */	52,		54,		56,		57,		58,		60,		62,		63,		65,		66,
    /* 4 */	67,		69,		70,		71,		72,		73,		75,		75,		77,		77,
    /* 5 */	79,		79,		80,		81,		82,		82,		82,		82,		82,		82,		
    88
};

float Weight[60]=   							//舵机打角权重（最近处不压线,用于弯道）
{
   /* 0 */	0.1,	0.2,	0.3,	0.4,	0.5,	0.5,	0.5,	0.5,	0.5,	0.5,
    /* 1 */	0.6,	0.7,	0.8,	0.9,	1.0,	1.2,	1.2,	1.4,	1.4,	1.5,
    /* 2 */	1.8,	2.0,	2.1,	2.3,	2.5,	2.6,	2.8,	2.9,	3.0,	3.2,
    /* 3 */	3.4,	3.6,	3.8,	4.0,	4.2,	4.3,	4.4,	4.2,	4.0,	3.8,
    /* 4 */	3.6,	3.4,	3.2,	3.0,	2.8,	2.6,	2.4,	2.2,	2.0,	1.8,
    /* 5 */	1.6,	1.6,	1.5,	1.5,  	1.4,  	1.4,  	1.2,  	1.2,  	1.0,   	0.8
};

float 	AverageCenter=0;						//归一中线数值
float 	LastAverageCenter=0;					//上一次中线
int16	LastAverageError[5]= {0};				//中线偏差保存
int16	LeftLose       = 0;						//仅左丢线行数
int16	RightLose      = 0; 					//仅右丢线行数
int16	AllLose        = 0;						//两边全丢白线行数（十字）
int16	WhiteLose      = 0;						//两边全丢黑线行数（丢失赛道）
int16	LeftLoseStart  = 0;						//记录左边丢线的开始行
int16	RightLoseStart = 0;						//记录右边边丢线的开始行
int16	WhiteLoseStart = 0;						//记录全丢白线的开始行
int16	FirstLoseAllLine=0;						//全丢白开始行///这个开始行需要重新定义
int16	LastLoseAllLine=0;						//全丢白结束行
int16	StartLineCount = 0;						//起跑线检测计数
int16	LastLine = 0;							//扫描最后一行,动态前瞻
int16	AvaliableLines = 0;						//扫描有效行数
int16		SearchErrorFlag = 0;

int   Foresight		 = 0;					//前瞻
int   Last_Foresight = 0;					//上一次前瞻
int16 Fictitious_Num = 0;					//虚拟直线拟合求取的偏差
int	  Track_ImgCount = 0;					//赛道判断间隔计数
int	  Track_Type	 = 0;					//赛道种类

uint32 MiddleLine[RowMax+1];					//中线存放数组
uint32 RightEdge[RowMax+1];  					//右边线存放数
uint32 LeftEdge[RowMax+1]; 					//左边线存放数组
uint32 RealWidth[RowMax+1];					//赛道实际宽度
uint32 Left_Add_Line[RowMax+1];				//左补线存放数组
uint32 Right_Add_Line[RowMax+1];				//右补线存放数组
uint32 Left_Add_Flag[RowMax];					//左补线标志位
uint32 Right_Add_Flag[RowMax];					//右补线标志位
uint32 Width_Add[RowMax+1];///uint32 Width_Add[RowMax];//补线宽度数组
uint32 Width_Min;								//赛道最小宽度
int16 Line_Count;							//记录成功识别到的赛道行数
uint32 Left_Add_Start = 0;						//左补线起始
uint32 Left_Add_Stop = 0;						//左补线结束行
uint32 Right_Add_Start = 0;					//右补线起始
uint32 Right_Add_Stop = 0;						//右补线结束行
float Left_Ka = 0, Right_Ka = 0;			//最小二乘法参数
float Left_Kb = 1, Right_Kb = ColumnMax-1;

/*********截止行提取变量初始化**********/
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

/*********十字处理变量初始化**********/
unsigned char  CrossFlag  = 0;					//十字标志
unsigned char  LastCrossFlag  = 0;
unsigned char  LastLastCrossFlag  = 0;
unsigned char  CrossNumber=0;
unsigned char  LoseCrossNumber=0;
unsigned char  CrossEnterFlag=0;
float CrossKL=0,CrossKR=0;

/*********环岛处理变量初始化**********/
float	Cirlce_Angle=0.0;				//环岛角度
float 	CircleDistance = 0.0;
float 	OutCircleDistance=0.0;
int16	RightOvalFlag = 0;
int16	LeftOvalFlag = 0;
int16	LeftCircleFlag = 0;				//左环岛识别标志位
int16	RightCircleFlag = 0;			//右环岛识别标志位
int16 	DrawOvalFlag=0;					//画环岛标志
int16   SalationFlag=0;
int16	LLSalationLine = 0;				//左边线左跳变起始行
int16	LRSalationLine = 0;				//左边线右跳变起始行
int16	RLSalationLine = 0;				//右边线左跳变起始行
int16	RRSalationLine = 0;				//右边线右跳变起始行

int16 Onlyonce = 1;///自加变量

/*********断路变量初始化**********/
float 	StationAngle = 0.0;
int 	StationLineNum = 0;
extern int16 LeftWheelSpeed;
//extern int16 RightWheelSpeed;

/*********三岔路变量**********/
///
unsigned char Road_widening = 0;

unsigned char Road_vanish = 0;

int16 spurroadtriangle_i = 0;

int16 spurroadtriangle_j = 0;

int16  left_right_lost_i = 0;                      ///左右全丢结束行

unsigned char  SpurroadFlag  = 0;                  //三岔口标志

unsigned char  LastSpurroadFlag  = 0;

unsigned char  LastLastSpurroadFlag  = 0;

float SpurroadKL=0,SpurroadKR=0;

/*********其它变量初始化**********/
int StartFlag =WithoutRun;						//起跑线标志位

/**********相关宏定义************/
#define StartLine       57				//中线扫描起始行

#define FinishLine      18				//中线扫描结束行

#define StepLine		1				//边缘追踪扫描间隔行

#define DivideLine		StartLine-10	//全行扫描和边缘追踪分隔行///47

#define MIDVALUE		48

#define ImageMode      		2         	//选择图像模式（0-OSTU/1-ITERATION/2-Sobel）

#define CheckWhitePixle(x,y) Image_Sobel[x][y] >= Threshold	//白点判断
#define CheckBlackPixle(x,y) Image_Sobel[x][y] < Threshold	//黑点判断
#define CheckLeft(x,y)	CheckBlackPixle(x,y) && CheckWhitePixle(x,y-1) //&& CheckWhitePixle(x,y-2)
#define CheckRight(x,y) CheckBlackPixle(x,y) && CheckWhitePixle(x,y+1) //&& CheckWhitePixle(x,y+2)
#define CheckEnd(x,y)	CheckWhitePixle(x,y) //&& CheckWhitePixle(x-1,y)
#define pi 3.14

/*************************ch的变量***********************/
uint8 g_left_edge[60]={0};//左边界
uint8 g_right_edge[60]={0};//右边界
    /*注意：上面两个数组存的是边界的列数
    g_left_edge没有找到边界时赋值为0，
    g_right_edge没有找到边界时赋值为93*/

uint8 g_centerline[60]={0};
uint8 found_num=59;//找到的有效行数
uint8 found_flag=0;
float g_image_err=0.0;
float g_image_last_err=0.0;
float g_image_err_ori=0.0;
float g_lastimg_err[5]={0.0};
int crossroad_flag=0;//十字检测标记
int fork_flag=0;//岔口检测标记
int fork_go_where=FORK_LEFT;
int fork_count=0;
int EXP_CENTERLINE=59;
int gray_diff = 20;//灰度差比和的阈值
uint8 USE_START=40;    
uint8 USE_END =20  ; 
const int row_weight_all[60]=
{
  0, 0, 0, 0, 0, 0, 1, 1, //0-7
  1, 1, 1, 1, 3, 3, 4, 4, //8-15
  4, 4, 4, 4, 4, 4, 4, 4, //16-23
  4, 4, 4, 4, 4, 4, 4, 4, //24-31
  4, 4, 3, 3, 3, 3, 3, 3, //32-39
  3, 3, 3, 3, 3, 3, 2, 2, //40-47
  2, 2, 2, 1, 1, 1, 1, 1, //48-55
  1, 1, 1, 1 //56-63
};
const int row_weight[60]=
{
  1, 1, 1, 1, 1, 1, 1, 1, //0-7
  1, 1, 1, 1, 1, 1, 1, 1, //8-15
  1, 1, 1, 1, 1, 1, 1, 1, //16-23
  1, 1, 1, 1, 1, 1, 1, 1, //24-31
  2, 2, 2, 2, 2, 2, 2, 2, //32-39
  2, 2, 2, 2, 2, 2, 2, 2, //40-47
  4, 4, 4, 4, 4, 4, 4, 4, //48-55
  4, 4, 4, 4//56-63
};
const int row_weight_50_90[15]=
{
  4,4,3,3,3,3,3,//7-13
  2,2,2,2,2,2,2,2//13-21
};

const uint8 row_index[64]={//120*188to64*128
 0, 1, 3, 4, 6, 8, 9,11,
13,14,16,18,19,21,22,24,
26,27,29,31,32,34,36,37,
39,41,42,44,45,47,49,50,
52,54,55,57,59,60,62,63,
65,67,68,70,72,73,75,77,
78,80,82,83,85,86,88,90,
91,93,95,96,98,100,101,103
};
const uint8 col_index[128]={//120*188to64*128
5,6,7,9,10,12,13,14,16,17,19,20,21,23,24,26,
27,28,30,31,33,34,35,37,38,40,41,42,44,45,47,
48,49,51,52,54,55,56,58,59,61,62,63,65,66,68,
69,70,72,73,75,76,77,79,80,82,83,84,86,87,89,
90,91,93,94,96,97,98,100,101,103,104,105,107,
108,110,111,112,114,115,117,118,119,121,122,
124,125,126,128,129,131,132,133,135,136,138,
139,140,142,143,145,146,147,149,150,152,153,
154,156,157,159,160,161,163,164,166,167,168,
170,171,173,174,175,177,178,180,181,183
};
/**********************************************************/

//图像处理总函数
float Camera_scan(void)
{
    Get_Use_Image();                                        //获取图像数据
    Threshold = /*adapt_otsuThreshold(Image_Use,ColumnMax,RowMax);*/ Sobel(1,RowMax-1);  //Sobel算子法，return的是sobel卷积后得到的阈值
    Pixle_Filter(FilterThreshold);		//对sobel图像噪点依据阈值剔除
    GetBlackEndParam();               //抽样提取黑线截止行
    // if(DrawOvalFlag)
    // {
    //     if(RightCircleFlag == OutCircle)
    //         DrawLine(RowMax-1,5,FindOvalPoint(ColumnMax/2,1),75);///从（59，5）到（15~20，75）
    //     else if(LeftCircleFlag == OutCircle)
    //         DrawLine(RowMax-1,90,FindOvalPoint(ColumnMax/2,3),20);///从（59，90）到（15~20，20）
    // }

    Get_01_Value(Threshold,RowMax,ColumnMax);				//二值化图像数据
    get_edge_center_new();

    // CircleConduct();										//对环岛进行处理
    // StartCheck();                                         	//起跑线检测
    WeightedAverageCalc(SpeedParm.AdvanceParm);				//中线加权归一

//    if(DrawOvalFlag)//在CircleConduct()进行环岛检测，有环岛时给DrawOvalFlag赋值为1
//    {
//        if(RightCircleFlag==MustBeCircle || RightCircleFlag==ShouldBeCircle)		//右岛进出岛画椭圆
//        {
//            AverageCenter = 93;//中线给到最右列
//        }
//        else if(LeftCircleFlag==MustBeCircle  || RightCircleFlag==ShouldBeCircle)	//左岛进出岛画椭圆
//        {
//            AverageCenter = 1;//中线给到最左列
//        }
//    }

#if 0
    if(StartLineCount>=10)								//再看到起跑线停车
    {
        int i = 0;
        StationLineNum = 0;
        for(i=StartLine; i>FinishLine; i--) //60///从57到19
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

    if(StartFlag >= FinishRun)///使用了陀螺仪的数据
    {
//		if(ABS(LeftWheelSpeed) + ABS(RightWheelSpeed) <120)
//		  SpeedParm = GearCtl.Zebra;
        StationAngle += (float)GYRO.Z*0.00064;		//对转向角度进行积分 控制进环程度

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
            SpeedParm = GearCtl.Stop;							//停车
            StationAngle = 0.0;
            StationLineNum = 0;
            StartFlag = 0;
//			}
        }
    }
#endif

    Middle_Err_Filter(AverageCenter);						//中线滤波
    TrackJudge();											//赛道类型判断
    LastAverageCenter = AverageCenter;						//中线保存
//	Draw_Road();

    Camera_scan_finish = 1;

    return AverageCenter;
}


/**
 * @file		带提前系数加权平均
 * @note
 
 */
void WeightedAverageCalc(uint32 AdvanceNum)
{
    int16 i=0, PointAdvance=0;
    float sum=0;
    for(sum=0,i=StartLine; i>= 1 /*LastLine*/; i--) ///从57到0
    {
        PointAdvance = i-AdvanceNum;///PointAdvance和i始终差一个AdvanceNum
        if (PointAdvance>59)	PointAdvance=59;///比59大的都按59处理
        AverageCenter += (g_centerline[PointAdvance]*Weight[i]);///*中线存放数组*中的数据与舵机打角权重相乘后累加///提前AdvanceNum个行取中线数据
        sum += Weight[i];///将舵机打角权重累加
    }
    AverageCenter = AverageCenter/sum;              //加权平均
    AverageCenter = RANGEfloat(AverageCenter, 0, ColumnMax-1);///限定范围
}

/**
 * @file		中心偏差滤波
 * @note
 
 */
float Middle_Err_Filter(float middle_err)
{
    float Middle_Err_Fltered;
    Middle_Err_Fltered = middle_err*0.8 + LastAverageCenter*0.2;///这次取0.8和上次的0.2求和
    return Middle_Err_Fltered;
}
/**
 * @file		起始斑马线滤波
 * @note
 
 */
void Zebra_Filter(void)
{
    int16 ZebraFilterLoc[StartLine+2][40];//没有那么多跳变点      ///57*40的数组
    int16 i=0,j=0,k=0,num=1;
    for(i=StartLine+1; i>=FinishLine-1; i--) ///58到18
    {
        for(j=25; j<ColumnMax-25-1; j++) ///25到67
        {
            if(CheckWhitePixle(i,j))///如果是白点
                ZebraFilterLoc[i][num++]=j;	//第二位开始存单行跳变点位置///将j记录到该点对应坐标数组里
        }
        for(k=num; k<40; k++) ZebraFilterLoc[i][k]=0; ///如果白点的数量大于40，不执行这条语句，否则num前面到40全清零
        ZebraFilterLoc[i][0]=num-1;		//第一位存跳变点数量
        num=1;
    }
    for(i=StartLine+1; i>=FinishLine-1; i--) ///58到18
    {
        for(j=1; j<=ZebraFilterLoc[i][0]; j++) ///1到白点总数
        {
            if(ZebraFilterLoc[i][ZebraFilterLoc[i][0]]!=0&&							//如果最后一个点是0一定就是没找到跳变点
                    ABS(ZebraFilterLoc[i][ZebraFilterLoc[i][0]]-ZebraFilterLoc[i][1])<30)///最后一个点不等于0且与第一个的差值小于30则
            {
                for(k=ZebraFilterLoc[i][1]; k<=ZebraFilterLoc[i][ZebraFilterLoc[i][0]]; k++) ///从第一个到最后白点
                {
                    Image_Sobel[i][k] = 0;
                }
            }
        }
    }
}

/**
 * @file		最小二乘法拟合斜率
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
    if((end-begin)*x2sum-xsum*xsum) //判断除数是否为零
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
 * @file		虚拟直线拟合
 * @note     	构建虚拟直线，赛道中线与之比较，返回相似度
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
 * @file		路径判断
 * @note     	计算前瞻，判断赛道类型
 */
void TrackJudge(void)
{
    //使用最远行偏差和加权偏差确定前瞻
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
    //弯道入直道
    if(LastAverageError[0]>7 && LastAverageError[1]>6 && LastAverageError[2]>5 && LastAverageError[4] < 4
            && BlackEndM > 30 && ABS(Fictitious_Num)<30 )
    {
        Track_Type = CurveToStraight;
    }
    //直道
    else if(LastAverageError[0]<4 && LastAverageError[1]<4 && LastAverageError[2]<4 && Foresight<=5)
    {
        Track_Type = InStraight;
    }
    //直道入弯道
    else if(LastAverageError[0]<6 && LastAverageError[1]<6 && LastAverageError[2]<6 && Foresight>=5)
    {
        Track_Type = StraightToCurve;
    }
    //弯道中
    else
    {
        Track_Type = InCurve;
    }
}

/**
 * @file		计算补线坐标
 * @note     	使用两点法计算拟合出的补线坐标
 */
int16 Calculate_Add(int16 i, float Ka, float Kb)	// 计算补线坐标
{
    float res;
    int16 Result;

    res = i * Ka + Kb;
    Result = RANGE16((int32)res, 1, ColumnMax-1);

    return Result;
}

/**
 * @file		两点法拟合直线
 * @note     	用起始点和其上一个点连线的斜率代替后续的点
				拟合直线 y = Ka * x + Kb
				Mode == 1代表左边界，Mode == 2代表右边界
 */
void Curve_Fitting(float *Ka, float *Kb, int16 *Start, int16 *Line, int16 *Add_Flag, int16 Mode)
{
    *Start += StepLine;///start++
    if (Add_Flag[*Start] == 1)///如果start+1需要补线
    {
        if (*Start <= 51)///且start+1小于等于51
            *Start += StepLine;///start比原始值多2
        if (Mode == 2)
        {
            *Ka = 1.0*(Line[*Start+StepLine] - Line[*Start]) / 2;///Line（比原始值多3）-line（比原始值多2）///(△（y+1)-△y)/2  斜率
            if (*Ka < 0)		*Ka = 0;
        }
        if (Mode == 1)
        {
            *Ka = 1.0*(Line[*Start+StepLine] - Line[*Start]) / 2;
            if (*Ka > 0)		*Ka = 0;
        }
    }
    else///否则
    {
        if(Mode == 2)
            *Ka = 1.0*(Line[*Start+StepLine] - Line[*Start]) / 2;///Line（比原始值多2）-line（比原始值多1）
        if(Mode == 1)
            *Ka = 1.0*(Line[*Start+StepLine] - Line[*Start]) / 2;
    }
    *Kb = 1.0*Line[*Start] - (*Ka * (*Start));
}

/**
 * @file		补线修复
 * @note     	有始有终才使用，直接使用两点斜率进行补线
 *              Mode == 1代表左边界，Mode == 2代表右边界
 */
void Line_Repair(int16 Start, int16 Stop,int16 *Line, int16 *Line_Add, int16 *Add_Flag, int16 Mode)
{
    float res;
    int16 i;	// 控制行
    float Ka, Kb;

    if ((Mode == 1) && (Right_Add_Start <= Stop) && Start <= StartLine-5)	// 只有左边界补线
    {
        if (Start <= StartLine-StepLine)///Start如果不大于26，Start自加
        {
            Start +=StepLine;
        }
        for (i = Start+StepLine; i >= Stop+StepLine;)
        {
            i -= StepLine;///一开始进入循环保持初始值，之后就自减到Stop+Stepline
            Line_Add[i] = RANGE16(Right_Add_Line[i] - Width_Add[i+StepLine], 1, ColumnMax-1);///补线的长度限幅
            if(  ABS(Line_Add[i]-Line_Add[i+1])  >5)	Line_Add[i] = Line_Add[i+1];///这行的补线长度和下一行的差值的绝对值大于5，则用下一行的补线数据
            Width_Add[i] = Width_Add[i+StepLine];///补线宽度数据上移
        }
    }
///***************************************自加****************************************
    else
    {
        if (Stop)   // 有始有终
        {
            if ((Right_Add_Stop >= MIDVALUE && Left_Add_Stop >= MIDVALUE) ||
                    (Right_Add_Stop >= MIDVALUE && Left_Add_Start <= Right_Add_Stop) ||
                    (Left_Add_Stop >= MIDVALUE && Right_Add_Start <= Left_Add_Stop))    // 只有较少行需要补线，不计算斜率，直接竖直向下补线
            {
                for (i = Stop-2; i <= 57; i += StepLine)
                    Line_Add[i] = Line_Add[Stop];
            }
            else    // 将起始行和结束行计算斜率补线
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
///***************************************自加****************************************
    if ((Mode == 2) && (Left_Add_Start <= Stop) && Start <= StartLine-5)	// 只有右边界补线
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
        if (Stop)	// 有始有终
        {
            if ((Right_Add_Stop >= MIDVALUE && Left_Add_Stop >= MIDVALUE) ||
                    (Right_Add_Stop >= MIDVALUE && Left_Add_Start <= Right_Add_Stop) ||
                    (Left_Add_Stop >= MIDVALUE && Right_Add_Start <= Left_Add_Stop))	// 只有较少行需要补线，不计算斜率，直接竖直向下补线
            {
                for (i = Stop-2; i <= 57; i += StepLine)
                    Line_Add[i] = Line_Add[Stop];
            }
            else	// 将起始行和结束行计算斜率补线
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


void OnlyOnce(void)///自加
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
 * @file		全行扫描和边缘结合提取赛道的中线
 * @note     	搜线核心算法
 
 */
void SearchCenterBlackline(void)
{
    int16 i = 0, j = 0, k =0;
    int16 LeftFilterStart=0;
    int16 RightFilterStart=0;
    int16 StartColumn  = 0;
    int16 FinishColumn = 0;
    int16 numfind=0;
    //全局变量清零
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
    //复位补线起始行坐标
    Left_Add_Start = 0;
    Right_Add_Start = 0;
    Left_Add_Stop = 0;
    Right_Add_Stop = 0;
    //复位第60行数据
    MiddleLine[RowMax-1]  = ColumnMax/2;///47
    LeftEdge[RowMax-1]    = 0;
    RightEdge[RowMax-1]   = ColumnMax;///94

    OnlyOnce();///自加

    /*************************************************************************************/
    /*                           一次遍历开始（全行+边缘）                                                          */
    /*************************************************************************************/
    for(i=StartLine; i>DivideLine; i--) ///57到47				//首先找前N行，全行扫描
    {

        //左右边界都从中线位置往一侧找
        /***********************************先找左边界********************************************/
        if(i == StartLine)  		j = MiddleLine[RowMax];	//首行就以上一场图像首行作为扫描起点///如果i=StartLine时，MiddleLine[RowMax]无有效数据，可能需要debug
        else if(AllLose > 5)		j = ColumnMax/2;
        else  					j = MiddleLine[i+1]; 	//否则就以上一行中点的位置作为本行扫描起点
        if(j < 3)     			j = 3;     				//j>=3有效范围内搜线
        while(j >= 3)									//j>=3有效范围内进行搜寻
        {
            //从右向左找到白黑黑跳变
            if(CheckLeft(i,j))
            {
                LeftEdge[i] = j;						//找到则赋值 找不到保持原值0
                break;									//跳出本行寻线
            }
            j--;										//列数往左移动
        }
        if(j < 3)
            LeftEdge[i] =1; 								//找不到左边线就令其为1
        /***********************************再找右边界********************************************/
        if(i==StartLine)   		j = MiddleLine[RowMax];	//首行就以上一场图像首行作为扫描起点
        else if(AllLose>5)		j = ColumnMax/2;
        else					j = MiddleLine[i+1]; 	//否则从上一行中心位置开始搜寻
        if(j >ColumnMax-3)		j = ColumnMax-3; 		//j <=ColumnMax-3有效范围内搜线
        while(j <= ColumnMax-3)
        {
            //从左向右找到白黑黑跳变点
            if(CheckRight(i,j))
            {
                RightEdge[i] = j;					//找到则赋值   找不到保持原值
                break;								//跳出本行寻线
            }
            j++;										//列数往右移动
        }
        if(j >= ColumnMax-3)
            RightEdge[i] = ColumnMax-1;					//找不到右边线就令其为ColumnMax-1

        /************************************滤波********************************************/
        if(LeftEdge[i]==1&&LeftEdge[i+1]==1&&LeftEdge[i+2]==1&&LeftEdge[i+3]==1&&LeftEdge[i+4]==1)
        ///一直找不到左边线
        {
            LeftFilterStart=i;///LeftFilterStart记为i左边界滤波开始行
        }
        if(RightEdge[i]==ColumnMax-1&&RightEdge[i+1]==ColumnMax-1&&RightEdge[i+2]==ColumnMax-1&&RightEdge[i+3]==ColumnMax-1&&RightEdge[i+4]==ColumnMax-1)
        ///一直找不到右边线
        {
            RightFilterStart=i;
        }
        /************************************丢线判断********************************************/
        Left_Add_Line[i] = LeftEdge[i];						//记录实际左边界为补线左边界
        Right_Add_Line[i] = RightEdge[i];					//记录实际右边界为补线左边界
        if((RightEdge[i]-LeftEdge[i]) >= (RightEdge[i+1]-LeftEdge[i+1]+2) && i<StartLine)//不满足畸变
            ///((RightEdge[i]-LeftEdge[i]) >= (RightEdge[i+1]-LeftEdge[i+1]+2) && i<StartLine)
        {
            Left_Add_Flag[i]=1;///这一行左补线标志置1
            Right_Add_Flag[i]=1;
            AllLose++;///这行不满足畸变用来判断十字
            if(AllLose>=2||AllLose<=5)
                FirstLoseAllLine = i;
            left_right_lost_i = i;
            MiddleLine[i] = MiddleLine[i+1];//用上一行
        }
        else if(	(LLSalationLine && RightEdge[i]==ColumnMax-1)||
                    (RRSalationLine && LeftEdge[i]==1))  	//全丢
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
        else if(LeftEdge[i]!=1 && RightEdge[i]!=ColumnMax-1)   	//没有丢线
        {
            Left_Add_Flag[i]=0;
            Right_Add_Flag[i]=0;
            MiddleLine[i] = (LeftEdge[i] + RightEdge[i])/2;///中线记为两边线中点
            if((RightEdge[i]-LeftEdge[i]) >= (RightEdge[i+1]-LeftEdge[i+1]+1) && i<StartLine)//不满足畸变
                MiddleLine[i] = MiddleLine[i+1];//用上一行
        }
        else if(LeftEdge[i]==1 && RightEdge[i]!=ColumnMax-1)	//丢了左线
        {
            if(!LeftLoseStart)	LeftLoseStart = i;///记录左边丢线的开始行
            Left_Add_Flag[i]=1; ///这一行左补线标志置1
            Right_Add_Flag[i]=0;///这一行右补线标志置0
            if(i<=RowMax-1 && RightEdge[i] > Width[i]/2 + 1)	//正常的话就用半宽补///右边界大于每行道路边界的一半视为正常。
                ///个人觉得右边界也应该小于width[i]///当左边界丢失时，右边界不可能大于width[i]。
                MiddleLine[i] = RightEdge[i] - Width[i]/2;
            else												//越界则以左界代替
                MiddleLine[i] = 1;///中线定为1意为需要疯狂向左转了
            LeftLose++;///仅左丢线行数
        }
        else if(LeftEdge[i]!=1 && RightEdge[i]==ColumnMax-1)	//丢了右线
        {
            Left_Add_Flag[i]=0;
            Right_Add_Flag[i]=1;
            if(!RightLoseStart)	RightLoseStart = i;
            if(i<=RowMax-1 && LeftEdge[i] + Width[i]/2 <  ColumnMax-1)		//正常的话就用半宽补
                MiddleLine[i] = LeftEdge[i] + Width[i]/2;
            else														//越界则以右界代替
                MiddleLine[i] = ColumnMax-1;
            BlackEndM++;//记录只有右线丢的数量///按对称来说应为下面这行代码
            RightLose++;///仅右丢线行数
        }
        else if((LeftEdge[i]==1 && RightEdge[i]==ColumnMax-1)) //全丢
        {
            Left_Add_Flag[i]=1;
            Right_Add_Flag[i]=1;
            AllLose++;
            if(AllLose>=2||AllLose<=5)
                FirstLoseAllLine = i;
            LastLoseAllLine = i;///全丢结束行会随着i的减少而变化到结束行。
            left_right_lost_i = i;
            MiddleLine[i]=MiddleLine[i+1];
        }
        if(Left_Add_Flag[i])														//左边界需要补线
        {
            if (i >= StartLine-6 && i < StartLine-1)													//前6行
                Left_Add_Line[i] = LeftEdge[StartLine-1];								//使用底行数据
            else if(i < StartLine-6)                    												//剩下的
                Left_Add_Line[i] = Left_Add_Line[i+1];						//使用前1行左边界作为本行左边界
        }
        if(Right_Add_Flag[i])														//右边界需要补线
        {
            if (i >= StartLine-6 && i < StartLine-1)													//前6行
                Right_Add_Line[i] = RightEdge[StartLine-1];							//使用底行数据
            else if(i < StartLine-6)      																//剩下的
                Right_Add_Line[i] = Right_Add_Line[i+1];						//使用前1行右边界作为本行右边界
        }
        if(RightEdge[i] > LeftEdge[i])		RealWidth[i]=RightEdge[i]-LeftEdge[i];	//计算实际赛道宽度
        else								RealWidth[i]=0;
        if(Right_Add_Line[i] > Left_Add_Line[i])									//计算补线赛道宽度
            Width_Add[i] = Right_Add_Line[i] - Left_Add_Line[i];
        else
            Width_Add[i] = 0;

        if(!RRSalationLine && i<StartLine-5 &&///正常情况下RightEdge[i]始终是最小的，如果不是只能说明不满足畸变，遇到了跳变点。目前来看只运行一次。
                RightEdge[i]==MIN(MIN(RightEdge[i+4],RightEdge[i+3]),RightEdge[i+2])&&
                RightEdge[i]==MIN(MIN(RightEdge[i  ],RightEdge[i+1]),RightEdge[i+2]))
        {
            RRSalationLine = i+2;///右边线右跳变起始行
        }
        else if(!LLSalationLine && i<StartLine-5 &&
                LeftEdge[i]==MAX(MAX(LeftEdge[i+4],LeftEdge[i+3]),LeftEdge[i+2])&&
                LeftEdge[i]==MAX(MAX(LeftEdge[i+2],LeftEdge[i+1]),LeftEdge[i]))
        {
            LLSalationLine = i+2;
        }
        /*******************************起始行数据处理**************************************/
        if(i==StartLine)///只有在起始行运行
        {
            LeftEdge      [RowMax]     = LeftEdge      [StartLine];		///数据保存到60行对应的列
            RightEdge     [RowMax]     = RightEdge     [StartLine];
            Left_Add_Line [RowMax]     = Left_Add_Line [StartLine];
            Right_Add_Line[RowMax]     = Right_Add_Line[StartLine];
            Left_Add_Line [StartLine+1]= Left_Add_Line [StartLine];
            Right_Add_Line[StartLine+1]= Right_Add_Line[StartLine];


            if (Left_Add_Flag[StartLine] && Right_Add_Flag[StartLine])///开始行左右需要补线
            {
                MiddleLine[StartLine] = MiddleLine[RowMax];///用60行的中线代替开始行
                FirstLoseAllLine = 0;///全丢白开始行清零
            }
            else
            {
                MiddleLine[RowMax] = MiddleLine[StartLine];	// 更新第60行虚拟中点，便于下一帧图像使用
            }

            MiddleLine[RowMax] = RANGE16(MiddleLine[RowMax],15,80);///中线限幅到15至80之间

            RealWidth[RowMax] = RealWidth[StartLine];

            Width_Add[RowMax] = Width_Add[StartLine];

            Width_Add[StartLine+1] = Width_Add[StartLine];

            Width_Min = Width_Add[StartLine]; //Width_Min其实是第一行赛道宽度也就是
            //没有进行梯形矫正图像的赛道最长宽度
        }
    }
    /*——————————————————分割线————————————————————*/
    for(i=DivideLine; i>=FinishLine; i-=StepLine)//查找剩余行///47到18
    {
        LastLine = i;///扫描最后一行
        /*******************************判断左右边界反常延伸*************************************/
        if(   (!SearchErrorFlag) && (   (LeftEdge[i+1]>=ColumnMax/2+5) || (RightEdge[i+1]<=ColumnMax/2-5)   )   )
        {
            ///ColumnMax/2+5=52,ColumnMax/2-5=42///左边界或右边界异常
            if(LeftEdge[i+1]>=ColumnMax/2+10 || RightEdge[i+1]<=ColumnMax/2-10 )///左边界大于57或者右边界小于37
            {
                SearchErrorFlag = i+2;///记录异常行，之后退出循环。
                break;
            }
            else if(LeftEdge[i+1]>=ColumnMax/2+5)///52
            {
                if(RightEdge[i+1] == ColumnMax-1)///93///未找到右边界
                {
                    FinishColumn = ColumnMax-1;
                    while(FinishColumn >= LeftEdge[i+1])///从右向左找左边界
                    {
                        if(CheckLeft(i,FinishColumn))
                        {
                            LeftEdge[i] = FinishColumn;///找到就记录下来
                            break;
                        }
                        FinishColumn--;
                    }
                    if(FinishColumn < LeftEdge[i+1])///否则就用上一行的左边界
                        LeftEdge[i] = LeftEdge[i+1];
                }
                else
                {
                    LeftEdge[i] = LeftEdge[i+1];///找到右边界，左边界用上一行的
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
        /***************************上一行没找到边界，全部全行扫描*******************************/
        if((LeftEdge[i+1]==1 && RightEdge[i+1]==ColumnMax-1)||
                LeftLose>5||RightLose>5||AllLose>5)
        {
            ////////////////////////先通过全行扫描找左边界///////////////////////////
            if(AllLose>5)
                FinishColumn = MiddleLine[RowMax];///如果全丢用60里的数据
            else
                FinishColumn = MiddleLine[i+1];///否则用上一行的数据
            if(FinishColumn <= 2)
            {
                FinishColumn= 3;
            }
            while(FinishColumn >= 3)
            {
                if(CheckLeft(i,FinishColumn))
                {
                    LeftEdge[i] = FinishColumn;///找得到保存数据
                    break;
                }
                FinishColumn--;
                if(FinishColumn < 3)
                    LeftEdge[i] = 1;///否则用最左边
            }
            ////////////////////////再通过全行扫描找右边界///////////////////////////
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


#if 0//我自己屏蔽的这一部分


        /***************************上一行两边都找到 全部边沿扫描*******************************/
        else if(LeftEdge[i+1]!=1 && RightEdge[i+1]!=ColumnMax-1)
        {
            //////////////////////////////先找左边界///////////////////////////////////
            FinishColumn = ((LeftEdge[i+1]+10) >= ColumnMax-3)? ColumnMax-3:(LeftEdge[i+1]+10);///比ColumnMax-3大用ColumnMax-3
            StartColumn = ((LeftEdge[i+1]-5) <= 3)? 3:(LeftEdge[i+1]-5);///比3小用3
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
            //////////////////////////////再找右边界///////////////////////////////////
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
        /***************************上一行只找到左边界 边沿+全行扫描*******************************/
        else if(LeftEdge[i+1]!=1 && RightEdge[i+1]==ColumnMax-1)
        {
            ////////////////////////先通过边沿扫描找左边界///////////////////////////
            FinishColumn = ((LeftEdge[i+1]+10) >=ColumnMax-2)? ColumnMax-2:(LeftEdge[i+1]+10);///比ColumnMax-2大用ColumnMax-2
            StartColumn = ((LeftEdge[i+1]-5) <= 1)? 1:(LeftEdge[i+1]-5);///比1小用1
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
            ////////////////////////再通过全行扫描找右边界///////////////////////////
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
        /***************************上一行只找到右边界 边沿+全行扫描*******************************/
        else if(LeftEdge[i+1]==1 && RightEdge[i+1]!=ColumnMax-1)
        {
            ////////////////////////先通过边沿扫描找右边界//////////////////////////
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
            ////////////////////////再通过全行扫描找左边界///////////////////////////
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

        /************************************丢线判断********************************************/
        Left_Add_Line[i] = LeftEdge[i];						//记录实际左边界为补线左边界
        Right_Add_Line[i] = RightEdge[i];					//记录实际右边界为补线左边界
        if((RightEdge[i]-LeftEdge[i]) >= (RightEdge[i+1]-LeftEdge[i+1]+3)///(RightEdge[i]-LeftEdge[i]) >= (RightEdge[i+1]-LeftEdge[i+1]+3
                && i<StartLine)//不满足畸变
        {
            if(LeftLose>10||RightLose>10||AllLose>10)///不满足畸变且全丢线大于10
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
            MiddleLine[i] = MiddleLine[i+1];//用上一行
        }
        else if(	(LLSalationLine && RightEdge[i]==ColumnMax-1)||
                    (RRSalationLine && LeftEdge[i]==1))  	//全丢
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
        else if(LeftEdge[i]!=1 && RightEdge[i]!=ColumnMax-1)   	//没有丢线
        {
            Left_Add_Flag[i]=0;
            Right_Add_Flag[i]=0;
            MiddleLine[i] = (LeftEdge[i] + RightEdge[i])/2;
        }
        else if(LeftEdge[i]==1 && RightEdge[i]!=ColumnMax-1)	//丢了左线
        {
            if(!LeftLoseStart)	LeftLoseStart = i;
            Left_Add_Flag[i]=1;
            Right_Add_Flag[i]=0;
            if(i<=RowMax-1 && RightEdge[i] > Width[i]/2 + 1)	//正常的话就用半宽补
                MiddleLine[i] = RightEdge[i] - Width[i]/2;
            else												//越界则以左界代替
                MiddleLine[i] = 1;
            LeftLose++;
        }
        else if(LeftEdge[i]!=1 && RightEdge[i]==ColumnMax-1)	//丢了右线
        {
            if(!RightLoseStart)	RightLoseStart = i;
            Left_Add_Flag[i]=0;
            Right_Add_Flag[i]=1;
            if(i<=RowMax-1 && LeftEdge[i] + Width[i]/2 <  ColumnMax-1)		//正常的话就用半宽补
                MiddleLine[i] = LeftEdge[i] + Width[i]/2;
            else														//越界则以右界代替
                MiddleLine[i] = ColumnMax-1;
            RightLose++;//记录只有右线丢的数量
        }
        else if((LeftEdge[i]==1 && RightEdge[i]==ColumnMax-1) ||
                (LLSalationLine && RightEdge[i]==ColumnMax-1) ||
                (RRSalationLine && LeftEdge[i]==1))  	//全丢
        {
            Left_Add_Flag[i]=1;
            Right_Add_Flag[i]=1;
            AllLose++;
            if(AllLose>=2||AllLose<=5)
                FirstLoseAllLine = i;
            MiddleLine[i]=MiddleLine[i+1];
        }
        if(Left_Add_Flag[i])													//左边界需要补线
        {
            if (i >= StartLine-6 && i < StartLine-1)							//前6行使用底行数据
                Left_Add_Line[i] = LeftEdge[StartLine-1];
            else if(i < StartLine-6)											//后6行使用前1行左边界作为本行左边界
                Left_Add_Line[i] = Left_Add_Line[i+1];
        }
        if(Right_Add_Flag[i])													//右边界需要补线
        {
            if (i >= StartLine-6 && i < StartLine-1)							//前6行使用底行数据
                Right_Add_Line[i] = RightEdge[StartLine-1];
            else if(i < StartLine-6)      										//后6行使用前1行左边界作为本行左边界
                Right_Add_Line[i] = Right_Add_Line[i+1];
        }
        if(RightEdge[i] > LeftEdge[i])			RealWidth[i]=RightEdge[i]-LeftEdge[i];	//计算实际赛道宽度
        else									RealWidth[i]=0;							//若越界则直接置0
        if(Right_Add_Line[i] > Left_Add_Line[i])										//计算补线赛道宽度
            Width_Add[i] = Right_Add_Line[i] - Left_Add_Line[i];
        else
            Width_Add[i] = 0;

        SearchErrorFlag = 0;
    }
    /*************************************************************************************/
    /*                              一次遍历结束                                         */
    /*************************************************************************************/
    /*************************************************************************************/
    /*                      二次遍历开始（进行补线+动态前瞻）                             */
    /*************************************************************************************/
    // for(i=StartLine; i>LastLine; i-=StepLine) ///从57到0
    // {
    //     /********************************* 补线检测开始 *************************************/
    //     if (RealWidth[i] > Width_Min+1)	// 赛道宽度变宽，可能是十字或环路，///或则是三岔路
    //     {
    //         //Width_Min其实是第一行赛道宽度也就是
    //         //没有进行梯形矫正图像的赛道最长宽度

    //         Road_widening = 1;///道路变宽标志，用于判断三岔路

    //         if (Left_Add_Line[i] < Left_Add_Line[i+2] &&
    //                 Left_Add_Line[i+2] < Left_Add_Line[i+4] && i<StartLine) 	//正常的是赛道远处宽度窄即Left_Add_Line[i]比较近处的大
    //         {
    //             if (!Left_Add_Flag[i])
    //                 Left_Add_Flag[i] = 1;	//上面误判为0则强制认定为需要补线
    //         }
    //         if (Right_Add_Line[i] > Right_Add_Line[i+2] &&
    //                 Right_Add_Line[i+2] > Right_Add_Line[i+4] &&i<StartLine)  	//与上面相反
    //         {
    //             if (!Right_Add_Flag[i])
    //                 Right_Add_Flag[i] = 1;	//上面误判为0则强制认定为需要补线
    //         }
    //         if ((Left_Add_Flag[i] || Right_Add_Flag[i] || SearchErrorFlag == i) && (i<FinishLine+10))///(Left_Add_Flag[i] || Right_Add_Flag[i] || SearchErrorFlag == i && i<FinishLine+10)
    //         {
    //             if (Left_Add_Stop || Right_Add_Stop || SearchErrorFlag == i)
    //             {
    //                 break;
    //             }
    //         }
    //     }
    //     /******************************** 第一轮补线开始 ************************************/
    //     if (Left_Add_Flag[i] && i<StartLine)	// 左侧需要补线
    //     {
    //         if (i >= StartLine-3 && i < StartLine)	// 前三行补线不算
    //         {
    //             if (!Left_Add_Start && Left_Add_Flag[i-1] && Left_Add_Flag[i-2])///i在54到56之间且前两行需要补线
    //             {
    //                 Left_Add_Start = i;	// 记录补线开始行
    //                 Left_Ka = 0;
    //                 Left_Kb = Left_Add_Line[i];
    //             }
    //             Left_Add_Line[i] = Calculate_Add(i, Left_Ka, Left_Kb); ///按照程序对称原则自加
    //         }
    //         else if(i < StartLine-3)///i小于54
    //         {
    //             if (!Left_Add_Start && Left_Add_Flag[i-1] && Left_Add_Flag[i-2])	// 之前没有补线
    //             {
    //                 Left_Add_Start = i;	// 记录左侧补线开始行
    //                 Curve_Fitting(&Left_Ka, &Left_Kb, &Left_Add_Start, Left_Add_Line, Left_Add_Flag, 1);	// 使用两点法拟合直线///获得新的Ka和Kb
    //             }
    //         }
    //         Left_Add_Line[i] = Calculate_Add(i, Left_Ka, Left_Kb);	// 使用前一行图像左边界斜率补线///加一个画线程序，画一个点
    //     }
    //     else
    //     {
    //         if (Left_Add_Start)	// 已经开始补线而且开始不丢线
    //         {
    //             if (!Left_Add_Stop &&
    //                     !Left_Add_Flag[i+2*StepLine] &&
    //                     !Left_Add_Flag[i+4*StepLine]
    //                     && i<StartLine-5*StepLine)///第后两行和后四行不丢线且i小于52
    //             {
    //                 if ((Left_Add_Line[i] >= Left_Add_Line[i+2*StepLine] &&
    //                         Left_Add_Line[i+2*StepLine] >= Left_Add_Line[i+4*StepLine]&&
    //                         Left_Add_Line[i]!=1 &&
    //                         Left_Add_Line[i+1*StepLine]!=1 &&
    //                         Left_Add_Line[i+2*StepLine]!=1))///第i，i+1，i+2行左边线不是1且第i，i+2,i+4行的左边线依次变大，说明满足畸变
    //                 {
    //                     Left_Add_Stop = i+4*StepLine;	// 记录左侧补线结束行

    //                     //Line_Repair(Left_Add_Start, Left_Add_Stop, LeftEdge, Left_Add_Line, Left_Add_Flag, 1);
    //                 }
    //             }
    //         }
    //     }
    //     if (Right_Add_Flag[i] && i<StartLine)	// 右侧需要补线
    //     {
    //         if (i >= StartLine-3 && i < StartLine)	// 前三行补线不算
    //         {
    //             if (!Right_Add_Start && Right_Add_Flag[i-1] && Right_Add_Flag[i-2])
    //             {
    //                 Right_Add_Start = i;	// 记录补线开始行
    //                 Right_Ka = 0;
    //                 Right_Kb = Right_Add_Line[i];
    //             }
    //             Right_Add_Line[i] = Calculate_Add(i, Right_Ka, Right_Kb);	// 使用前一帧图像右边界斜率补线
    //         }
    //         else if(i < StartLine-3)

    //         {
    //             if (!Right_Add_Start)	// 之前没有补线
    //             {
    //                 Right_Add_Start = i;	// 记录右侧补线开始行
    //                 Curve_Fitting(&Right_Ka, &Right_Kb, &Right_Add_Start, Right_Add_Line, Right_Add_Flag, 2);	// 使用两点法拟合直线
    //             }
    //             Right_Add_Line[i] = Calculate_Add(i, Right_Ka, Right_Kb);	// 补线完成
    //         }
    //     }
    //     else
    //     {
    //         if (Right_Add_Start)	// 已经开始补线
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
    //                     Right_Add_Stop = i+4*StepLine;	// 记录右侧补线结束行
    //                     //Right_Add_Start = 0;
    //                     //Line_Repair(Right_Add_Start, Right_Add_Stop, RightEdge, Right_Add_Line, Right_Add_Flag, 2);
    //                 }
    //             }
    //         }
    //     }
    //     /********************************* 第一轮补线结束 **********************************/
    //     if(Right_Add_Line[i] >= Left_Add_Line[i])
    //         Width_Add[i] = Right_Add_Line[i] - Left_Add_Line[i];	// 重新计算赛道宽度
    //     else
    //         Width_Add[i] = 0;
    //     if ((Left_Add_Flag[i] && Right_Add_Flag[i]) || (!Left_Add_Flag[i] && !Right_Add_Flag[i]))///左右同时需要补线或者同时不需要
    //         MiddleLine[i] = (Right_Add_Line[i] + Left_Add_Line[i]) / 2;	// 计算中线
    //     else if(i<StartLine)
    //         MiddleLine[i] = MiddleLine[i+StepLine];
    //     if (Width_Add[i] < Width_Min)
    //     {
    //         Width_Min = Width_Add[i];				// 更新最小赛道宽度
    //     }
    //     Line_Count = i;                          	//Line_Count更新
    // }
    // /*************************************************************************************/
    // /*                              二次遍历结束                                         */
    // /*************************************************************************************/

    // /*************************************************************************************/
    // /*                     三次遍历开始（边线修复+中线修复）                              */
    // /*************************************************************************************/
    // /******************************* 补线修复开始 ********************************/

    // if (Left_Add_Start)		// 左边界需要补线
    // {
    //     Line_Repair(Left_Add_Start, Left_Add_Stop, LeftEdge, Left_Add_Line, Left_Add_Flag, 1);
    // }
    // if (Right_Add_Start)	// 右边界需要补线
    // {
    //     Line_Repair(Right_Add_Start, Right_Add_Stop, RightEdge, Right_Add_Line, Right_Add_Flag, 2);
    // }
    // /******************************* 补线修复结束 ********************************/
    // /******************************* 中线修复开始 ********************************/
    // for(i=StartLine; i>=Line_Count; i-=StepLine)
    // {
    //     MiddleLine[i] = (Right_Add_Line[i] + Left_Add_Line[i]) / 2;	// 计算赛道中点
    //     if(Right_Add_Line[i] >= Left_Add_Line[i])
    //         Width_Add[i] = Right_Add_Line[i] - Left_Add_Line[i];	// 重新计算赛道宽度
    //     else
    //         Width_Add[i] = 0;
    //     if(SearchErrorFlag == i || CheckEnd(i-1,MiddleLine[i]) || i == FinishLine)///用这个动态前瞻判断三岔口的三角
    //     {
    //         LastLine = i;//最后一行，动态前瞻
    //         spurroadtriangle_i = i;
    //         spurroadtriangle_j = MiddleLine[i];
    //         AvaliableLines = StartLine - i;//有效行数
    //         break;
    //     }
    // }

    /*************************************************************************************/
    /*                              三次遍历结束                                         */
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
 * @file		提取图像的特征，获取黑线截止行
 * @note     	选取几列，从图像底近处往远扫描
 
 */
void GetBlackEndParam(void)
{
    unsigned char LLLEndFlag = 0;///白点标志位
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

//清零
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
            BlackEndL++;//左黑线截至行
        else if(i > 1 && CheckBlackPixle(i-1,ColumnMax*2/8) && CheckBlackPixle(i-2,ColumnMax*2/8))
            LEndFlag = 1;

        if(CheckWhitePixle(i,ColumnMax*3/8) && !MLEndFlag)//10
            BlackEndML++;
        else if(i > 1 && CheckBlackPixle(i-1,ColumnMax*3/8) && CheckBlackPixle(i-2,ColumnMax*3/8))
            MLEndFlag = 1;

        if(CheckWhitePixle(i,ColumnMax*4/8) && !MEndFlag)//10
            BlackEndM++;//中黑线截至行
        else if(i > 1 && CheckBlackPixle(i-1,ColumnMax*4/8) && CheckBlackPixle(i-2,ColumnMax*4/8))
            MEndFlag = 1;

        if(CheckWhitePixle(i,ColumnMax*5/8) && !MREndFlag)//10
            BlackEndMR++;
        else if(i > 1 && CheckBlackPixle(i-1,ColumnMax*5/8) && CheckBlackPixle(i-2,ColumnMax*5/8))
            MREndFlag = 1;

        if(CheckWhitePixle(i,ColumnMax*6/8) && !REndFlag)//10
            BlackEndR++;//右黑线截至行
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
    for (i = RowMax-1; i >0; i--)///从59到0行///找到白点后再找到黑点,黑点行数计数
    {
        if(CheckBlackPixle(i,ColumnMax/12) && !LLLEndFlag )//94/12=7.83
            BlackEndLLL++;//第i行，第7列是黑点
        else if(i > 1 && CheckEnd(i,ColumnMax/12))
            LLLEndFlag = 1;//第i行，第7列是白点

        if(CheckBlackPixle(i,ColumnMax/8) && !LLEndFlag)//94/8=11.75
            BlackEndLL++;
        else if(i > 1 && CheckEnd(i,ColumnMax/8))
            LLEndFlag = 1;

        if(CheckBlackPixle(i,ColumnMax*2/8) && !LEndFlag)//23.5
            BlackEndL++;//左黑线截至行
        else if(i > 1 && CheckEnd(i,ColumnMax*2/8))
            LEndFlag = 1;

        if(CheckBlackPixle(i,ColumnMax*3/8) && !MLEndFlag)//35.25
            BlackEndML++;
        else if(i > 1 && CheckEnd(i,ColumnMax*3/8))
            MLEndFlag = 1;

        if(CheckBlackPixle(i,ColumnMax*4/8) && !MEndFlag)///47
            BlackEndM++;//中黑线截至行
        else if(i > 1 && CheckEnd(i,ColumnMax*4/8))
            MEndFlag = 1;

        if(CheckBlackPixle(i,ColumnMax*5/8) && !MREndFlag)///58.75
            BlackEndMR++;
        else if(i > 1 && CheckEnd(i,ColumnMax*5/8))
            MREndFlag = 1;

        if(CheckBlackPixle(i,ColumnMax*6/8) && !REndFlag)///70.5
            BlackEndR++;//右黑线截至行
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

    if(!LLLEndFlag)		BlackEndLLL = 0;///清零

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
    BlackEndMin =MIN(BlackEndL,BlackEndM);//取大值
    BlackEndMin =MIN(BlackEndMin,BlackEndR);
}

//对十字进行识别和进行特殊处理
void NormalCrossConduct(void)
{
    ///unsigned char i;
    int j=0;
    LastLastCrossFlag=LastCrossFlag;//记录上上次是否是十字
    LastCrossFlag=CrossFlag;//记录上一次是否是十字
    CrossFlag=0;//清零

    if((AllLose>=5))//左右同时丢线
    {
        CrossFlag=1;//标记十字

        Road_vanish = 1;///判断三岔路，边路消失。
    }
    if((AllLose<=1))//左右不同时丢线
    {
        CrossFlag=0;//清除标记十字

        Road_vanish = 0;///清除三岔路边路消失标志

        if(CrossNumber>3) LoseCrossNumber++;       //如果左右十字计数从有到无则计数
    }
    if(CrossFlag && LastCrossFlag && LastLastCrossFlag)  //连续左右丢线
    {
        CrossNumber++;                                    //十字计数
    }
    if(LoseCrossNumber>3)                                //已经进入圆环了
    {
        CrossEnterFlag = 1;                               //进圆环标志位置起
        CrossNumber = 0;                                  //圆环统计清除

    }
    if(CrossEnterFlag && CrossNumber>3)                  //出圆环
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
 * @file		对三岔口进行处理
 * @note     	判断是三岔口之后在这里处理
 * @date		2021年2月24日
 */
void Spurroad_conduct(void)///1，判断是否道路变宽；2，判断是否两边连续丢线；3，判断前瞻是否大于每个数值；
///4，从前瞻的坐标向另一边补线或者直接向某一方向转向之后就是正常的寻线了。
{
    /// unsigned char i;
    int j=0,k=0;
    int L1,L2,R1,R2;
    LastLastSpurroadFlag=LastSpurroadFlag;//记录上上次是否是十字
    LastSpurroadFlag=SpurroadFlag;//记录上一次是否是十字
    //SpurroadFlag=0;//清零

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
 * @file		对环岛进行处理
 * @note     	判断是环岛之后在这里处理
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
        if(LeftLose > 15)///仅左丢线行数大于15
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
        else if(CircleDistance>50)///圆环距离大于50
        {
            LeftCircleFlag = NoCircle;
            RightCircleFlag = NoCircle;
            CircleDistance = 0;
            SpeedParm = SelectMode;
        }
    }

    /*******************左环********************/
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
    /*****************************确认入环**********************************/
    else if(LeftCircleFlag == InCircle)
    {

        if(Cirlce_Angle>140)
        {
            ImgCount=count=0;
            LeftCircleFlag = OutCircle;					//左右都丢线超过十行则开始出岛
        }
    }
    /*****************************判断出环**********************************/
    else if(LeftCircleFlag == OutCircle)
    {
        if(Cirlce_Angle>240)
        {
            DrawOvalFlag=0;
            LeftCircleFlag = StopCheckCircle;	//出岛完毕
            ImgCount=count=0;    	//限制为100场图像内更新否则清掉
        }
    }
    /********************出环成功，过段时间再扫标志位**************************/
    else if(LeftCircleFlag == StopCheckCircle)
    {
        if(OutCircleDistance*2>=CircleDistance)
        {
            OutCircleDistance = CircleDistance = 0;///这个OutCircleDistance好像没有数据
            ImgCount=count=0;
            LeftCircleFlag = NoCircle;    //一次之后就不判了
        }
    }
    /*******************               右环               ********************/
    /*****************************准备进环***********************************/
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
    /*****************************确认入环**********************************/
    else if(RightCircleFlag == InCircle)
    {
        if(Cirlce_Angle<-140)
        {
            ImgCount=count=0;
            RightCircleFlag = OutCircle;					//左右都丢线超过十行则开始出岛
        }
    }
    /*****************************判断出环**********************************/
    else if(RightCircleFlag == OutCircle)
    {
        if(Cirlce_Angle<-240)
        {
            DrawOvalFlag=0;
            RightCircleFlag = StopCheckCircle;	//出岛完毕
            ImgCount=count=0;    	//限制为100场图像内更新否则清掉
        }
    }
    /********************出环成功，过段时间再扫标志位**************************/
    else if(RightCircleFlag == StopCheckCircle)
    {
        if(OutCircleDistance*2>=CircleDistance)
        {
            OutCircleDistance = CircleDistance = 0;
            ImgCount=count=0;
            RightCircleFlag = NoCircle;   //一次之后就不判了
        }
    }
    ImgCount++;

    if(ImgCount>1500)
    {
        ImgCount=0;
    }
}

/**
 * @file		起跑线检测
 * @note     	判断是不是起跑线
 
 */
void StartCheck(void)///int StartCheck(void)
{
    for(uint32 i =StartLine; i>FinishLine; i--) ///从57到19 
    {
        if(CountRowB2WSalation(i,1,ColumnMax-1)>5 && CountRowW2BSalation(i,1,ColumnMax-1)>5)///从左向右扫描
        //CountRowB2WSalation的return是从黑往右边一列突变到白的点的个数
        //CountRowW2BSalation则相反
        //满足这两个条件时代表白色的道路中间有一黑块斑马线
        {
            StartLineCount++;
        }
    }
}

/**
 * @file		找寻环岛画椭圆起始坐标
 * @note     	以进入环岛处曲线坐标为依据画椭圆
 
 */
int16 FindOvalPoint(int16 col,int16 flag)///从列数的一半开始
{
    int16 i,j,num=0;
    int16 MREndFlag=0,REndFlag=0,RREndFlag=0;
    int16 MLEndFlag=0,LEndFlag=0,LLEndFlag=0;
    int16 CheckMR=0,CheckR=0,CheckRR=0;
    int16 CheckML=0,CheckL=0,CheckLL=0;

    if(flag==1)
    {
        for(i=30; i>15; i--) ///从30到16
        {
            for(j=col; j<=ColumnMax-1; j++) ///从col到93
            {
                if(CheckWhitePixle(i,j) &&
                        Image_Use[i+2,j]>=Image_Use[i,j] &&
                        Image_Use[i+4,j]>=Image_Use[i+2,j])
                {
                    num++;///从（i，j）到（i+4，j）都为白色且越来越白，则num自加
                }
            }
            if(num>15)		return i;
            else			num = 0;
        }
        return 20;
    }

    else if(flag==2)///原本被注释掉了
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
}//我分析这个是从（x0，y0），到（x1，y1）以k为斜率画一条白色边界，再将白线左侧到0的像素点变为黑色。


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
* 函数名称：void Draw_Road(void)
***************************************************************/
void Draw_Road(void)
{
    int16 i = 0, j = 0,temp=0;
    char txt[10];
    ips200_address_set(0,200,ColumnMax-1,200+RowMax-1);//设置显示区域
    for(i=0; i<RowMax; i++) //6*8=48行
    {
        for(j=0; j<ColumnMax; j++) //列数
        {
            temp=0;
            if(Pixle[i][j])
                ips200_wr_data16(0x0000);
            else
                ips200_wr_data16(0xFFFF);
        }
    }
    for(i=0; i<RowMax; i++) //必须独立循环显示
    {
        ips200_drawpoint((int16)AverageCenter, i, RED);//画中线
    }
}

/***************************************************************
* 函数名称：void Get_01_Value(void)
* 功能说明：按照均值的比例进行二值化
* 参数说明： limit-二值化阈值，height/width像素数组大小
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
        //黑点为0（灰度值最低），白点反之
       }
   }
}

/***************************************************************
* 函数名称：void Pixle_Filter(void)
* 功能说明：对sobel图像噪点依据阈值剔除
* 参数说明：
* 函数返回：
* 修改时间：2019
* 备 注：
***************************************************************/
void Pixle_Filter(int16 threshold)
{
    int i,j;
    for(i=StartLine+1; i>=FinishLine-1; i--) ///18到58
    {
        for(j=1; j<ColumnMax-1; j++) ///1到93
        {
            if(CheckWhitePixle(i,j) && Image_Use[i][j]>threshold)
                Image_Sobel[i][j] = 0;///按照我的理解不应该设置为0黑色///阳光算法

            if(	CheckBlackPixle(i-1,j-1)&&
                    CheckBlackPixle(i-1,j  )&&
                    CheckBlackPixle(i-1,j+1)&&
                    CheckBlackPixle(i  ,j-1)&&
                    CheckBlackPixle(i  ,j+1)&&
                    CheckWhitePixle(i  ,j  )&&
                    CheckBlackPixle(i+1,j-1)&&
                    CheckBlackPixle(i+1,j  )&&
                    CheckBlackPixle(i+1,j+1))
                Image_Sobel[i][j] = 0;///找黑点 3*3
            //白点被黑点包围时，将其修正为黑点（这就叫阳光算法吗？）
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  @name    	Sobel
//  @brief   	Sobel算子图像边缘提取
//				sobel算子包含两组3x3的矩阵，分别为横向及纵向，
//				之与图像作平面卷积，即可分别得出横向及纵向的亮度差分近似值。
//  @param		start-起始行；finish-终止行
//  @return     求得的阈值
//  @since      v1.0
//  @note
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
int16 Sobel(int16 start,int16 finish)
{
    int tempx=0,tempy=0,temp=0,i=0,j=0,k=0;
    long long int g_sum=0;
    start=RANGE16(start,1,RowMax-1);///1到59行数
    finish=RANGE16(finish,1,RowMax-1);
    for(i=start; i <finish; i++) ///从开始到结束
    {
        for(j=1; j<ColumnMax-1; j++) ///1到93列数
        {
            //用x方向的算子进行卷积
            tempx=(-  	Image_Use[i-1][j-1])
                  +(-2*Image_Use[i ][j-1])
                  +(-  Image_Use[i+1][j-1])
                  +(   Image_Use[i-1][j+1])
                  +( 2*Image_Use[i  ][j+1])
                  +(   Image_Use[i+1][j+1]);
            if(tempx<0)		tempx=-tempx;

            //用y方向的算子进行卷积
            tempy=(		Image_Use[i+1][j-1])
                  +( 2*Image_Use[i+1][j  ])
                  +(   Image_Use[i+1][j+1])
                  +(-  Image_Use[i-1][j-1])
                  +(-2*Image_Use[i-1][j  ])
                  +(-  Image_Use[i-1][j+1]);
            if(tempy<0)		tempy=-tempy;

            //将xy两方向的结果进行相加，得到该点的灰度值，再与设定的阈值进行比较，判断其是否为边缘点
            temp=tempx+tempy;
            if(temp>255)	temp=255;

            g_sum+=temp;
            k++;
            Image_Sobel[i][j]=temp;
        }
    }
    g_sum=2*g_sum/k;//阈值求取

    return((int16)g_sum);
}

/***************************************************************
* 函数名称：void BinaryImage(uint32 tmImage[IMAGEH][IMAGEW])
* 功能说明：图像数据二值化
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
* 函数名称： void Get_Use_Image(void)
* 功能说明：获取需要的图像数据
***************************************************************/
void Get_Use_Image(void)
{
    int i = 0,j = 0,row = 0,line = 0;///row是高，line是宽

    for(i = 0; i  < IMAGEH; i+=2)  //120行，每2行采集一行，只取原图像的1/2
    {
        for(j = 0; j < IMAGEW; j+=2) //188，每2列采集一次
        {
            Image_Use[row][line] = mt9v03x_image[i][j];
            line++;
        }
        line = 0;
        row++;
    }
}

//**********大津法求阈值*****************//
int16 adapt_otsuThreshold(int16 *image, int16 col, int16 row)   //注意计算阈值的一定要是原图像
{
   #define GrayScale 256
    int16 width = col;
    int16 height = row;
    int pixelCount[GrayScale];
    float pixelPro[GrayScale];
    int i, j, pixelSum = width * height/4;
    int16 threshold = 0;
    int16* data = image;  //指向像素数据的指针
    for (i = 0; i < GrayScale; i++)
    {
        pixelCount[i] = 0;
        pixelPro[i] = 0;
    }
    
    int16 gray_sum=0;
    //统计灰度级中每个像素在整幅图像中的个数  
    for (i = 0; i < height; i+=2)
    {
        for (j = 0; j < width; j+=2)
        {
            pixelCount[(int)data[i * width + j]]++;  //将当前的点的像素值作为计数数组的下标
            gray_sum+=(int)data[i * width + j];       //灰度值总和
        }
    }
                      
    //计算每个像素值的点在整幅图像中的比例  
  
    for (i = 0; i < GrayScale; i++)
    {
        pixelPro[i] = (float)pixelCount[i] / pixelSum;
        
    }

    //遍历灰度级[0,255]  
    float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;
    
    w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;
    for (j = 0; j < GrayScale; j++)         
    {
    
        w0 += pixelPro[j];  //背景部分每个灰度值的像素点所占比例之和   即背景部分的比例
        u0tmp += j * pixelPro[j];  //背景部分 每个灰度值的点的比例 *灰度值 
    
        w1=1-w0;
        u1tmp=gray_sum/pixelSum-u0tmp;

        u0 = u0tmp / w0;              //背景平均灰度
        u1 = u1tmp / w1;              //前景平均灰度
        u = u0tmp + u1tmp;            //全局平均灰度
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

void get_edge_center_new()
{
    int i;
    static int16 j_l;//记录上一次边线位置
    static int16 j_r;
    int16 lose_line_row=0;
    int16 last_centerline=46;
    int temp1=0;
    int temp2=0;
    //从上一次的中点往两边搜,起始行为最近处的一行（第64行）
    // get_longest_whiteline();

    MedianFilter(Image_Sobel[0],gray_filter[0],92,59);
    //中值滤波处理，一个像素点的灰度值用其周围矩阵灰度的平均值来代替
    //作用是消除孤立噪声点，提高平滑性

    found_flag=1;

    for(i = GETEDGE_START;i >= BASE_LINE;i--)  
    {
        //最近一行

        //左边线
        if(i == GETEDGE_START)
        {
            for(j_l = 0; j_l <= 92; j_l++)//第一行从左向右搜线
            {
                temp1=(gray_filter[i][j_l+1]-gray_filter[i][j_l])<<7/(gray_filter[i][j_l+1]+gray_filter[i][j_l]);
                if(temp1>gray_diff)
                {
                    g_left_edge[i] = j_l;
                    break;
                }
            }
            if(j_l == 93)//没搜到//图像内没有黑色点
            {
                g_left_edge[i] = 0;
            }
        }
        else
        {     
            for(j_l = last_centerline; j_l >= 1; j_l--)
            //从中间那一列往左边找边线（即黑点）
            {
                temp1=(gray_filter[i][j_l]-gray_filter[i][j_l-1])<<7/(gray_filter[i][j_l]+gray_filter[i][j_l-1]);
                    if(temp1>gray_diff)
                    {
                        g_left_edge[i] = j_l;
                        break;
                    }
            } 
            if(j_l == 0)//没搜到
            {
                g_left_edge[i] = 0;
            }       
        }

        //右边线
        if(i==GETEDGE_START)
        {
            for(j_r = 93; j_r >=1; j_r--)
            {
                temp2=(gray_filter[i][j_r-1]-gray_filter[i][j_r])<<7/(gray_filter[i][j_r-1]+gray_filter[i][j_r]);
                if(temp2>gray_diff)
                {
                    g_right_edge[i] = j_r;
                    break;
                }
            } 
            
            if(j_r == 0)//没搜到
            {
                g_right_edge[i] = 93;
            }
        }
        else//i<63
        {
            for(j_r = last_centerline; j_r <=92; j_r++)//从上一行的中线开始向两边搜线
            {
                temp2=(gray_filter[i][j_r]-gray_filter[i][j_r+1])<<7/(gray_filter[i][j_r]+gray_filter[i][j_r+1]);
                if(temp2>gray_diff)
                    {
                        g_right_edge[i] = j_r;
                        break;
                    }
            } 
            if(j_r == 93)//没搜到
            {
                g_right_edge[i] = 93;
            }
        }

        last_centerline=(g_right_edge[i]+g_left_edge[i])/2;
        if(Pixle[i][last_centerline]==1&&found_flag==1)
            found_num=i;
        if(Pixle[i][last_centerline]==0)
            found_flag=0;
    }
      fork_check();
       crossroad_pass();
       crossroad_patch();//十字补线
        get_center();//得到中线数组
}

void get_center()
{
   int i;
    for(i = GETEDGE_START;i >= BASE_LINE;i--)  
    {
      g_centerline[i]=(g_right_edge[i]+g_left_edge[i])/2;
    }
}

void MedianFilter(unsigned char *pImg1,unsigned char *pImg,int nWidth,int nHeight)
{		
    unsigned char   *lpSrc;			                // 指向源图像的指针	
	unsigned char   *lpDst;		                 	// 指向要复制区域的指针
	int         aValue[iFilterH*iFilterW];		    // 指向滤波器数组的指针
	int			i,j,k,l;		                    // 循环变量	
	int			lLineBytes;		                    // 图像每行的字节数	
	lLineBytes = WIDTHBYTES(nWidth * 8);
	for ( i=0;i<nWidth;i++,pImg++ )
		(*pImg)=0;
	// 开始中值滤波
	// 行(除去边缘几行)
	for(i = iFilterMY; i < nHeight - iFilterH + iFilterMY + 1; i++)
	{
		// 列(除去边缘几列)
		for(j = iFilterMX; j < nWidth - iFilterW + iFilterMX + 1; j++)
		{
			// 指向新DIB第i行，第j个象素的指针
			lpDst = pImg + lLineBytes * (nHeight - 1 - i) + j;
			
			// 读取滤波器数组
			for (k = 0; k < iFilterH; k++)
			{
				for (l = 0; l < iFilterW; l++)
				{
					// 指向DIB第i - iFilterMY + k行，第j - iFilterMX + l个象素的指针
					lpSrc = pImg1 + lLineBytes * (nHeight - 1 - i + iFilterMY - k) + j - iFilterMX + l;
				
					// 保存象素值
					aValue[k * iFilterW + l] = *lpSrc;
				}
			}
			
			// 获取中值
			* lpDst = GetMedianNum(aValue, iFilterH * iFilterW);
		}
	}
 
}

unsigned char GetMedianNum(int * bArray, int iFilterLen)
{
	int		i,j;			// 循环变量
	unsigned char bTemp;
	
	// 用冒泡法对数组进行排序
	for (j = 0; j < iFilterLen - 1; j ++)
	{
		for (i = 0; i < iFilterLen - j - 1; i ++)
		{
			if (bArray[i] > bArray[i + 1])
			{
				// 互换
				bTemp = bArray[i];
				bArray[i] = bArray[i + 1];
				bArray[i + 1] = bTemp;
			}
		}
	}
	
	// 计算中值
	if ((iFilterLen & 1) > 0)
	{
		// 数组有奇数个元素，返回中间一个元素
		bTemp = bArray[(iFilterLen + 1) / 2];
	}
	else
	{
		// 数组有偶数个元素，返回中间两个元素平均值
		bTemp = (bArray[iFilterLen / 2] + bArray[iFilterLen / 2 + 1]) / 2;
	}
	
	return bTemp;
}


void crossroad_pass()//十字检测
{
    int i=0;
    int counter=0;
    int white_flag=0;
    int center_counter=0;
    int center_flag=0;

    for(i=BASE_LINE+25; i<GETEDGE_START; i++)
    {
        crossroad_flag=0;
        if(g_right_edge[i]==93 && g_left_edge[i]==0)
            counter++;
        if(counter>5)
        {
            white_flag=1;
            break;
    //连续5行发生左右两边界都丢线时，认为出现十字
        }
    }

    for(i=43; i<49; i++)
    {
        if( Pixle[BASE_LINE][i]==1)
        {
            center_counter++;
        }
        if(center_counter>3)
        {
            center_flag=1;
            break;
        }
        /*注意：光是连续五行丢线还不能判断是十字还是三岔口
        所以还要再判断中间几列是不是都是白色（三岔路的中间会有一个突出来的角）*/
    }

    if(center_flag==1 && white_flag==1)
    {
        crossroad_flag=1;//表示确实找到了十字
        fork_flag=0;
    }
}

void fork_check()//检测岔口
{
    int left_flag=0;
    int right_flag=0;
    int temp_left=0;
    int temp_right=93;
    int row_left=0;
    int row_right=0;
    int center_high=0;
    int center_counter=0;
    int temp;
    int i=0;
    int k= 0;
    int center_flag1=0;
    int center_flag2=0;
    int center_temp1=0;
    int center_temp2=0;

    /***************找最靠内的点*******************/
    for(i=GETEDGE_START; i>BASE_LINE; i--)
    {
        if(g_right_edge[i]<temp_right)
        {
            temp_right=g_right_edge[i];
            row_right=i;
        }
    }

    for(i=GETEDGE_START; i>BASE_LINE; i--)
    {
        if(g_left_edge[i]>temp_left)
        {
            temp_left=g_left_edge[i];
            row_left=i;
        }
    }

    /*********************判断条件**************************/
    for(i=GETEDGE_START; i>BASE_LINE+6; i--)
    {
        //若右边出现黑线向内凸
        if((g_right_edge[i]-g_right_edge[i-3])*(g_right_edge[i-3]-g_right_edge[i-6])<0)
        {
            right_flag=1;
            break;
        }
    }

        //若左边出现黑线向内凸
    for(i=GETEDGE_START; i>BASE_LINE+6; i--)
    {
        if((g_left_edge[i]-g_left_edge[i-3])*(g_left_edge[i-3]-g_left_edge[i-6])<0)
        {
            left_flag=1;
            break;
        }
    }

        //用前瞻检测三岔路口中间出现黑色凸起，方向是从中间向左
    for(i= g_centerline[BASE_LINE]; i>0; i--)
    {
        center_temp1=(gray_filter[BASE_LINE][i]-gray_filter[BASE_LINE][i-1])<<7/(gray_filter[BASE_LINE][i]+gray_filter[BASE_LINE][i-1]);
        if(center_temp1<-gray_diff)//为负数时表示从黑跳变为白
        {
            center_flag1=1;
            break;
        }
    }

        //用前瞻检测三岔路口中间出现黑色凸起，方向是从中间向右
    for(i=g_centerline[BASE_LINE]; i<127; i++)
    {
        center_temp2=(gray_filter[BASE_LINE][i]-gray_filter[BASE_LINE][i+1])<<7/(gray_filter[BASE_LINE][i]+gray_filter[BASE_LINE][i+1]);
        if(center_temp2<-gray_diff)
        {
            center_flag2=1;
            break;
        }
    }

    if(left_flag==1&&right_flag==1&&center_flag1==1&&center_flag2==1)
        fork_flag=1;
    else
        fork_flag=0;

    /*****************************************************/

    if(fork_flag==1)
    {
        fork_count++;
        //进入三岔路口之后先后走左路和右路
        crossroad_flag==0;
        temp=(temp_left+temp_right)/2;//此时temp_left，temp_right是三岔路两个突变点（最窄处）的列数

        for(i=GETEDGE_START; i>0; i--)
        {
            //k=(gray_filter[temp][i]-gray_filter[temp][i-1])<<7/(gray_filter[temp][i]+gray_filter[temp][i-1]);
            k=(gray_filter[i][temp]-gray_filter[i-1][temp])<<7/(gray_filter[i][temp]+gray_filter[i-1][temp]);
            if(k>gray_diff)
                center_high=i;
            //看中间黑色凸起凸到哪一行
        }

        switch (fork_count)
        {
            case 1:
                fork_go_where = FORK_LEFT;
                break;

            case 2:
                fork_go_where = FORK_LEFT;
                break;

            case 3:
                fork_go_where = FORK_RIGHT;
                break;

            case 4:
                fork_go_where = FORK_RIGHT;
                break;
            
            default:
                fork_count = 0;
                break;
        }

        if(fork_go_where == FORK_LEFT)
        {
            if(center_high<BASE_LINE)
                center_high=BASE_LINE;
            if(temp>30)
                temp=30;
            
            //通过斜率计算在右侧补线！
            for(i=center_high; i<row_right; i++)
                g_right_edge[i]=temp+(i-center_high)*(g_right_edge[row_right]-temp)/(row_right-center_high);

            fork_go_where = FORK_RIGHT;
            return;//直接跳出三岔路口，下一次再进入时走右路

        }

        if(fork_go_where == FORK_RIGHT)
        {
            if(center_high<BASE_LINE)
                center_high=BASE_LINE;
            if(temp<86)
                temp=86;
            for(i=center_high; i<row_left; i++)
                g_left_edge[i]=temp+(i-center_high)*(g_left_edge[row_left]-temp)/(row_left-center_high);
        }
    }
}

void crossroad_patch()//十字补线
{
    int i=0;
    int j=0;
    int k=0;
    int flag_left=0;
    int flag_right=0;
    int temp_left1=0;
    int temp_left2=59;
    int temp_right1=0;
    int temp_right2=59;
    
    if( crossroad_flag==1)
    {
        for(i=BASE_LINE; i<GETEDGE_START-5; i++)
        {
            if((g_left_edge[i]==0)&&(g_left_edge[i+1]==0)&&(g_left_edge[i+2]==0)&&(g_left_edge[i+3]==0))
            {
                for(k=i; k>BASE_LINE; k--)
                {
                    if(g_left_edge[k]>0)//左边界为白色
                        temp_left1=k;
                    //temp_left1表示十字部分的终点行数
                }
                flag_left=1;
                break;
            }
        }
        temp_left2=GETEDGE_START;


        for(i=BASE_LINE; i<GETEDGE_START-5; i++)
        {
            if((g_right_edge[i]==93)&&(g_right_edge[i+1]==93)&&(g_right_edge[i+2]==93)&&(g_right_edge[i+3]==93))
            {
                for(k=i; k>BASE_LINE; k--)
                {
                    if(g_right_edge[k]<93)
                        temp_right1=k;
                }
                flag_right=1;
                break;
            }

        }
        temp_right2=GETEDGE_START;

        //左右两边各自补线
        for(i=temp_left1; i<temp_left2; i++)
        {
            g_left_edge[i]=g_left_edge[temp_left1]+(i-temp_left1)*(g_left_edge[temp_left2]-g_left_edge[temp_left1])/(temp_left2-temp_left1);
        }
        for(i=temp_right1; i<temp_right2; i++)
        {
            g_right_edge[i]=g_right_edge[temp_right1]+(i-temp_right1)*(g_right_edge[temp_right2]-g_right_edge[temp_right1])/(temp_right2-temp_right1);
        }
    }
}


void get_longest_whiteline()
{
  int i=0,j=0;
  int max=0;
  int counter=0;
  int counter1=0;
  for(i=30;i<100;i++)
  {
    counter=0;
    for(j=60;j>0;j--)
    {
      if(gray_filter[i][j] > Threshold)
        counter++;
      if(gray_filter[i][j] < Threshold)
        break;
    }
    if(counter>=max)
    {
      max=counter;
      EXP_CENTERLINE=i;
    }
  }
}