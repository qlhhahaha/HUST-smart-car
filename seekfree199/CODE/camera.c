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
#include "camera.h"
int Test = 5;///

/*********图像提取相关初始化**********/
volatile u8 Image_Use[RowMax][ColumnMax];     	 	//压缩后图像数据

volatile uint16_t Pixle[RowMax][ColumnMax];    	//二值化后用于OLED显示的数据//

volatile u8 Image_Sobel[RowMax][ColumnMax];     	//Sobel图像数据


/*********图像处理相关初始化**********/
uint8_t Threshold;                      	//OSTU大津法计算的图像阈值
int16 FilterThreshold = 225;

const int Width[RowMax+1]=  					//图像每行直道对应行宽 ///镜头限制高度后可能行宽也要做相应的改变，也就是说数据需要手动更新。
{
    /* 0 */	1,		1,		1,		1,		1,		1,		1,		2,		3,		3,
    /* 1 */	10,		10,		10,		10,		16,		17,		17,		19,		29,		31,
    /* 2 */	33,		35,		37,		39,		41,		42,		44,		46,		48,		50,
    /* 3 */	52,		54,		56,		57,		58,		60,		62,		63,		65,		66,
    /* 4 */	67,		69,		70,		71,		72,		73,		75,		75,		77,		77,
    /* 5 */	79,		79,		80,		81,		82,		82,		82,		82,		82,		82,		88
};
/*		0		1		2		3		4		5		6		7		8		9		*/

float Weight[60]=   							//舵机打角权重（最近处不压线,用于弯道）
{
    /* 0 */	0.1,	0.2,	0.3,	0.4,	0.5,	0.5,	0.5,	0.5,	0.5,	0.5,
    /* 1 */	0.6,	0.7,	0.8,	0.9,	1.0,	1.2,	1.2,	1.4,	1.4,	1.5,
    /* 2 */	1.8,	2.0,	2.1,	2.3,	2.5,	2.6,	2.8,	2.9,	3.0,	3.2,
    /* 3 */	3.4,	3.6,	3.8,	4.0,	4.2,	4.3,	4.4,	4.2,	4.0,	3.8,
    /* 4 */	3.6,	3.4,	3.2,	3.0,	2.8,	2.6,	2.4,	2.2,	2.0,	1.8,
    /* 5 */	1.6,	1.6,	1.5,	1.5,  	1.4,  	1.4,  	1.2,  	1.2,  	1.0,   	0.8
};
/*		0		1		2		3		4		5		6		7		8		9		*/
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
u8		SearchErrorFlag = 0;
int16 	BKnum=0;

int   Foresight		 = 0;					//前瞻
int   Last_Foresight = 0;					//上一次前瞻
int16 Fictitious_Num = 0;					//虚拟直线拟合求取的偏差
int	  Track_ImgCount = 0;					//赛道判断间隔计数
int	  Track_Type	 = 0;					//赛道种类

u16 MiddleLine[RowMax+1];					//中线存放数组
u16 RightEdge[RowMax+1];  					//右边线存放数
u16 LeftEdge[RowMax+1]; 					//左边线存放数组
u16 RealWidth[RowMax+1];					//赛道实际宽度
u16 Left_Add_Line[RowMax+1];				//左补线存放数组
u16 Right_Add_Line[RowMax+1];				//右补线存放数组
u16 Left_Add_Flag[RowMax];					//左补线标志位
u16 Right_Add_Flag[RowMax];					//右补线标志位
u16 Width_Add[RowMax+1];///u16 Width_Add[RowMax];//补线宽度数组
u16 Width_Min;								//赛道最小宽度
int16 Line_Count;							//记录成功识别到的赛道行数
u16 Left_Add_Start = 0;						//左补线起始
u16 Left_Add_Stop = 0;						//左补线结束行
u16 Right_Add_Start = 0;					//右补线起始
u16 Right_Add_Stop = 0;						//右补线结束行
float Left_Ka = 0, Right_Ka = 0;			//最小二乘法参数
float Left_Kb = 1, Right_Kb = ColumnMax-1;
u8 Ramp_Flag = 0;

float LCurvature = 0;
float RCurvature = 0;
int16 LeftEdgeArea=0;
int16 RightEdgeArea=0;
u8	AddFlag[RowMax]= {0};

float GYROControl_P=0.025;
int16 Gyro_Z=0;

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
int16 BlackEndMaxMax  = 0;
int16 BlackEndMin     = 0;
int16 DropRow         = 0;

int16 EndMArray[5];
int16 EndMLArray[5];
int16 EndMRArray[5];

int16 LastBlackEndLL	= 0;
int16 LastBlackEndRR	= 0;
int16 LastBlackEndLLL	= 0;
int16 LastBlackEndRRR	= 0;

float EndLR[4];
float KAdd;
float KRight=0;
float KLeft=0;

/*********十字处理变量初始化**********/
unsigned char  CrossFlag  = 0;					//十字标志
unsigned char  LastCrossFlag  = 0;
unsigned char  LastLastCrossFlag  = 0;
unsigned char  CrossNumber=0;
unsigned char  LoseCrossNumber=0;
unsigned char  CrossEnterFlag=0;
unsigned char  CrossExitFlag=0;
float CrossKL=0,CrossKR=0;

/*********环岛处理变量初始化**********/
float	Cirlce_Angle=0.0;				//环岛角度
float 	CircleDistance = 0.0;
float 	OutCircleDistance=0.0;
int16	RightOvalFlag = 0;
int16	LeftOvalFlag = 0;
int16	LeftAddingLinePointX = 0;		//左补线跳变行
int16	LeftAddingLinePointY = 0;		//左补线跳变列
int16	LastLeftAddingLinePointX = 0;	//上一次左补线跳变行
int16	LastLeftAddingLinePointY = 0;	//上一次左补线跳变列
int16	RightAddingLinePointX = 0;		//右补线跳变行
int16	RightAddingLinePointY = 0;		//右补线跳变列
int16	LastRightAddingLinePointX = 0;	//上一次右补线跳变行
int16	LastRightAddingLinePointY = 0;	//上一次右补线跳变列
uint8	LeftCircleFlag = 0;				//左环岛识别标志位
uint8	RightCircleFlag = 0;			//右环岛识别标志位
uint8	CircleKeepFlag=0;
uint8   StartTurnFlag=1;				//起步转向标志
uint8 	DrawOvalFlag=0;					//画环岛标志
int16   SalationFlag=0;
int16	LLSalationLine = 0;				//左边线左跳变起始行
int16	LRSalationLine = 0;				//左边线右跳变起始行
int16	RLSalationLine = 0;				//右边线左跳变起始行
int16	RRSalationLine = 0;				//右边线右跳变起始行





u8 Onlyonce = 1;///自加变量

/*********断路变量初始化**********/
int16 	LoseGrayAvr=0;
float 	StationAngle = 0.0;
int 	StationLineNum = 0;
extern int16 LeftWheelSpeed;
//extern int16 RightWheelSpeed;

/*********起跑线滤波变量初始化**********/




/*********三岔路变量**********/
///
unsigned char Road_widening = 0;

unsigned char Road_vanish = 0;

int16 spurroadtriangle_i = 0;

int16 spurroadtriangle_j = 0;

int16  left_right_lost_i = 0;                      ///左右全丢结束行

unsigned char Turnleft = 0;

unsigned char Turnright= 0;

unsigned char  spurroadover = 0;

unsigned char  SpurroadFlag  = 0;                  //三岔口标志

unsigned char  LastSpurroadFlag  = 0;

unsigned char  LastLastSpurroadFlag  = 0;

unsigned char  SpurroadNumber=0;

unsigned char  LoseSpurroadNumber=0;

unsigned char  SpurroadEnterFlag=0;

unsigned char  SpurroadExitFlag=0;

float SpurroadKL=0,SpurroadKR=0;


/*********其它变量初始化**********/
unsigned char 	BeepFlag = 0;					//蜂鸣器标志位
int StartFlag =WithoutRun;						//起跑线标志位

/************宏开关**************/
#define ImageMode      		2         	//选择图像模式（0-OSTU/1-ITERATION/2-Sobel）

#define SearchTrackMode		0			//循迹模式（1-间接处理，0-直接处理（直接灰度））

/**********相关宏定义************/
#define StartLine       57				//中线扫描起始行

#define FinishLine      18				//中线扫描结束行

#define StepLine		1				//边缘追踪扫描间隔行

#define DivideLine		StartLine-10	//全行扫描和边缘追踪分隔行///47

#define MIDVALUE		48


#define CheckWhitePixle(x,y) Image_Sobel[x][y] >= Threshold	//白点判断
#define CheckBlackPixle(x,y) Image_Sobel[x][y] < Threshold	//黑点判断
#define CheckLeft(x,y)	CheckBlackPixle(x,y) && CheckWhitePixle(x,y-1) //&& CheckWhitePixle(x,y-2)
#define CheckRight(x,y) CheckBlackPixle(x,y) && CheckWhitePixle(x,y+1) //&& CheckWhitePixle(x,y+2)
#define CheckEnd(x,y)	CheckWhitePixle(x,y) //&& CheckWhitePixle(x-1,y)
#define pi 3.14


//图像处理总函数
float Camera_scan(void)
{
    Get_Use_Image();                                        //获取图像数据
    Threshold = (uint8_t)Sobel(1,RowMax-1);          		//Sobel算子法，return的是sobel卷积后得到的阈值
    Pixle_Filter(FilterThreshold);							//对sobel图像噪点依据阈值剔除
    GetBlackEndParam();                                     //抽样提取黑线截止行
    if(DrawOvalFlag)
    {
        if(RightCircleFlag == OutCircle)
            DrawLine(RowMax-1,5,FindOvalPoint(ColumnMax/2,1),75);///从（59，5）到（15~20，75）
        else if(LeftCircleFlag == OutCircle)
            DrawLine(RowMax-1,90,FindOvalPoint(ColumnMax/2,3),20);///从（59，90）到（15~20，20）
    }

    Get_01_Value(Threshold,RowMax,ColumnMax);				//二值化图像数据
    SearchCenterBlackline();								//寻中线
    CircleConduct();										//对环岛进行处理
    StartCheck();                                         	//起跑线检测
    WeightedAverageCalc(SpeedParm.AdvanceParm);				//中线加权归一

    if(DrawOvalFlag)//在CircleConduct()进行环岛检测，有环岛时给DrawOvalFlag赋值为1
    {
        if(RightCircleFlag==MustBeCircle || RightCircleFlag==ShouldBeCircle)		//右岛进出岛画椭圆
        {
            AverageCenter = 93;//中线给到最右列
        }
        else if(LeftCircleFlag==MustBeCircle  || RightCircleFlag==ShouldBeCircle)	//左岛进出岛画椭圆
        {
            AverageCenter = 1;//中线给到最左列
        }
    }

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
    return AverageCenter;
}

/**
 * @file		带提前系数加权平均
 * @note
 * @author		AHNU逐梦者（CYX）
 * @date		2019
 */
void WeightedAverageCalc(u16 AdvanceNum)
{
    int16 i=0, PointAdvance=0;
    float sum=0;
    for(sum=0,i=StartLine; i>=LastLine; i--) ///从57到0
    {
        PointAdvance = i-AdvanceNum;///PointAdvance和i始终差一个AdvanceNum
        if (PointAdvance>59)	PointAdvance=59;///比59大的都按59处理
        AverageCenter += (MiddleLine[PointAdvance]*Weight[i]);///*中线存放数组*中的数据与舵机打角权重相乘后累加///提前AdvanceNum个行取中线数据
        sum += Weight[i];///将舵机打角权重累加
    }
    AverageCenter = AverageCenter/sum;              //加权平均
    AverageCenter = RANGEfloat(AverageCenter, 0, ColumnMax-1);///限定范围
}

/**
 * @file		中心偏差滤波
 * @note
 * @author		AHNU逐梦者（CYX）
 * @date		2019
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
 * @author		AHNU逐梦者（CYX）
 * @date		2019
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
 * @file		图像测试函数
 * @note
 * @author		AHNU逐梦者（CYX）
 * @date		2019
 */
void CameraTest(int16 num)
{
    static uint16_t wait_flag=0;///count_image=0,,count=0
    Get_Use_Image();                                         //获取图像数据
    GetBlackEndParam();
    //oval(FindOvalPoint(10),10,1);///
    //oval((uint8)(MAX(FindOvalPoint(75),FindOvalPoint(65))),70,0);///
    Threshold = (uint8_t)Sobel(1,RowMax-1);          		//Iteration迭代法///
    Pixle_Filter(FilterThreshold);
    Zebra_Filter();///
    //oval(FindOvalPoint(ColumnMax/2,1),5,1);///
    DrawLine(RowMax-1,5,FindOvalPoint(ColumnMax/2,1),80);///
    Get_01_Value(Threshold,RowMax,ColumnMax);                     //二值化图像数据
    //Pixle_Filter(Pixle);                                     //反数滤波
    Draw_Road();
}

/**
 * @file		最小二乘法拟合斜率
 * @note
 * @author		AHNU逐梦者（CYX）---参考网传代码
 * @date		2019
 */
float Slope_Calculate(uint8 begin,uint8 end,u16 *p)
{
    float xsum=0,ysum=0,xysum=0,x2sum=0;
    uint8 i=0;
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
 * @author		AHNU逐梦者（CYX）---参考网传代码
 * @date		2019
 */
int16 Fictitious_Beeline(void)
{
    float KA,KB;
/// u8 End;
    u8 i;
    int16 Sum=0;
    int16 Num;
    float res;
    u8 Result;

    KA = 1.0*(MiddleLine[StartLine] - MiddleLine[FinishLine]) / (StartLine-FinishLine);
    KB = 1.0*MiddleLine[FinishLine] - (KA * FinishLine);

    for(i=StartLine; i>LastLine; i-=StepLine)
    {
        res = i * KA + KB;
        Result = RANGE16(res,2,ColumnMax-2);
        Num = MiddleLine[i]- Result;
        //Num = ABS(Num);
        Sum+=Num;
    }
    return Sum;
}

/**
 * @file		路径判断
 * @note     	计算前瞻，判断赛道类型
 * @author		AHNU逐梦者（CYX）---参考网传代码
 * @date		2019
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
        LED_Ctrl(LEDCORE,OFF);
        LED_Ctrl(LEDCORE,ON);
        //LED_Ctrl(LED0,ON);
        //LED_Ctrl(LED2,ON);
    }
    //直道
    else if(LastAverageError[0]<4 && LastAverageError[1]<4 && LastAverageError[2]<4 && Foresight<=5)
    {
        Track_Type = InStraight;
        //LED_Ctrl(LEDCORE,OFF);
    }
    //直道入弯道
    else if(LastAverageError[0]<6 && LastAverageError[1]<6 && LastAverageError[2]<6 && Foresight>=5)
    {
        Track_Type = StraightToCurve;
        //LED_Ctrl(LEDCORE,OFF);
        //LED_Ctrl(LED0,ON);
        //LED_Ctrl(LED1,ON);
    }
    //弯道中
    else
    {
        Track_Type = InCurve;
        LED_Ctrl(LEDCORE,OFF);
        //LED_Ctrl(LED3,ON);
        //LED_Ctrl(LED2,ON);
    }
}

/**
 * @file		计算补线坐标
 * @note     	使用两点法计算拟合出的补线坐标
 * @author		AHNU逐梦者（CYX）---参考网传代码
 * @date		2019
 */
int16 Calculate_Add(uint8 i, float Ka, float Kb)	// 计算补线坐标
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
 * @author		AHNU逐梦者（CYX）---参考网传代码
 * @date		2019
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
 * @author		AHNU逐梦者（CYX）---参考网传代码
 * @date		2019
 */
void Line_Repair(uint8 Start, uint8 Stop,int16 *Line, int16 *Line_Add, int16 *Add_Flag, int16 Mode)
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

u8 CheckSunny(u8 LeftGrayScale,u8 RightGrayScale)
{
    float num=0.0;
    num = (LeftGrayScale-RightGrayScale)/(LeftGrayScale+RightGrayScale);
    if(ABS(num)>0.5)
        return 0;
    else
        return 1;
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
 * @author		AHNU逐梦者（CYX）
 * @date		2019
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
        /*if(LeftLose>20 || AllLose>20 && LeftEdge[i]!=1)
        	break;
        if(RightLose>20 || AllLose>20 && RightEdge[i]!=ColumnMax-1)
          	break;*/
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


#if 1


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
        /************************************滤波********************************************/
        //左丢线滤波
        /*if(LeftEdge[i]==1&&LeftEdge[i+1]==1&&LeftEdge[i+2]==1&&LeftEdge[i+3]==1&&LeftEdge[i+4]==1)
        {
        	LeftFilterStart=i;
        }
        if(LeftFilterStart&&LeftFilterStart<=i+2+4
           &&LeftEdge[i]==1&&LeftEdge[i+1]==1&&LeftEdge[i+2]!=1)
        {
        	for(int fix=LeftFilterStart;fix>=i+2;fix-=StepLine)
        	{
        		LeftEdge[fix]=1;
        	}
        }
        //右丢线滤波
        if(RightEdge[i]==ColumnMax-1&&RightEdge[i+1]==ColumnMax-1&&RightEdge[i+2]==ColumnMax-1&&RightEdge[i+3]==ColumnMax-1&&RightEdge[i+4]==ColumnMax-1)
        {
        	RightFilterStart=i;
        }
        if(RightFilterStart&&RightFilterStart<=i+2+4
           &&RightEdge[i]==ColumnMax-1&&RightEdge[i+1]==ColumnMax-1&&RightEdge[i+2]!=ColumnMax-1)
        {
        	for(int fix=RightFilterStart;fix>=i+2;fix-=StepLine)
        	{
        		RightEdge[fix]=ColumnMax-1;
        	}
        }
        //矫正前端不符合规则的线
        if(i<FinishLine+10)
        {
        	if(LeftEdge[i]+2<=LeftEdge[i+1]&&
        	   LeftEdge[i+1]>LeftEdge[i+2]&&LeftEdge[i+2]>LeftEdge[i+3]&&LeftEdge[i+3]>LeftEdge[i+4])
        	{
        		LeftEdge[i] = LeftEdge[i+1];
        	}
        	if(RightEdge[i]>=RightEdge[i+1]+2&&
        	   RightEdge[i+1]<RightEdge[i+2]&&RightEdge[i+2]<RightEdge[i+3]&&RightEdge[i+3]<RightEdge[i+4])
        	{
        		RightEdge[i] = RightEdge[i+1];
        	}
        }*/

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

        /*if(!RRSalationLine && i<StartLine-5 &&
           RightEdge[i]==MIN(MIN(RightEdge[i+4],RightEdge[i+3]),RightEdge[i+2])&&
           RightEdge[i]==MIN(MIN(RightEdge[i],RightEdge[i+1]),RightEdge[i+2]))
        {
        	RRSalationLine = i+2;
        }
        else if(!LLSalationLine && i<StartLine-5 &&
           LeftEdge[i]==MAX(MAX(LeftEdge[i+4],LeftEdge[i+3]),LeftEdge[i+2])&&
           LeftEdge[i]==MAX(MAX(LeftEdge[i+2],LeftEdge[i+1]),LeftEdge[i]))
        {
        	LLSalationLine = i+2;
        }*/
        SearchErrorFlag = 0;
    }
    /*************************************************************************************/
    /*                              一次遍历结束                                         */
    /*************************************************************************************/
    /*************************************************************************************/
    /*                      二次遍历开始（进行补线+动态前瞻）                             */
    /*************************************************************************************/
    for(i=StartLine; i>LastLine; i-=StepLine) ///从57到0
    {
        /********************************* 补线检测开始 *************************************/
        if (RealWidth[i] > Width_Min+1)	// 赛道宽度变宽，可能是十字或环路，///或则是三岔路
        {
            //Width_Min其实是第一行赛道宽度也就是
            //没有进行梯形矫正图像的赛道最长宽度


            Road_widening = 1;///道路变宽标志，用于判断三岔路



            if (Left_Add_Line[i] < Left_Add_Line[i+2] &&
                    Left_Add_Line[i+2] < Left_Add_Line[i+4] && i<StartLine) 	//正常的是赛道远处宽度窄即Left_Add_Line[i]比较近处的大
            {
                if (!Left_Add_Flag[i])
                    Left_Add_Flag[i] = 1;	//上面误判为0则强制认定为需要补线
            }
            if (Right_Add_Line[i] > Right_Add_Line[i+2] &&
                    Right_Add_Line[i+2] > Right_Add_Line[i+4] &&i<StartLine)  	//与上面相反
            {
                if (!Right_Add_Flag[i])
                    Right_Add_Flag[i] = 1;	//上面误判为0则强制认定为需要补线
            }
            if ((Left_Add_Flag[i] || Right_Add_Flag[i] || SearchErrorFlag == i) && (i<FinishLine+10))///(Left_Add_Flag[i] || Right_Add_Flag[i] || SearchErrorFlag == i && i<FinishLine+10)
            {
                if (Left_Add_Stop || Right_Add_Stop || SearchErrorFlag == i)
                {
                    break;
                }
            }
        }
        /******************************** 第一轮补线开始 ************************************/
        if (Left_Add_Flag[i] && i<StartLine)	// 左侧需要补线
        {
            if (i >= StartLine-3 && i < StartLine)	// 前三行补线不算
            {
                if (!Left_Add_Start && Left_Add_Flag[i-1] && Left_Add_Flag[i-2])///i在54到56之间且前两行需要补线
                {
                    Left_Add_Start = i;	// 记录补线开始行
                    Left_Ka = 0;
                    Left_Kb = Left_Add_Line[i];
                }
                Left_Add_Line[i] = Calculate_Add(i, Left_Ka, Left_Kb); ///按照程序对称原则自加
            }
            else if(i < StartLine-3)///i小于54
            {
                if (!Left_Add_Start && Left_Add_Flag[i-1] && Left_Add_Flag[i-2])	// 之前没有补线
                {
                    Left_Add_Start = i;	// 记录左侧补线开始行
                    Curve_Fitting(&Left_Ka, &Left_Kb, &Left_Add_Start, Left_Add_Line, Left_Add_Flag, 1);	// 使用两点法拟合直线///获得新的Ka和Kb
                }
            }
            Left_Add_Line[i] = Calculate_Add(i, Left_Ka, Left_Kb);	// 使用前一行图像左边界斜率补线///加一个画线程序，画一个点
        }
        else
        {
            if (Left_Add_Start)	// 已经开始补线而且开始不丢线
            {
                if (!Left_Add_Stop &&
                        !Left_Add_Flag[i+2*StepLine] &&
                        !Left_Add_Flag[i+4*StepLine]
                        && i<StartLine-5*StepLine)///第后两行和后四行不丢线且i小于52
                {
                    if ((Left_Add_Line[i] >= Left_Add_Line[i+2*StepLine] &&
                            Left_Add_Line[i+2*StepLine] >= Left_Add_Line[i+4*StepLine]&&
                            Left_Add_Line[i]!=1 &&
                            Left_Add_Line[i+1*StepLine]!=1 &&
                            Left_Add_Line[i+2*StepLine]!=1))///第i，i+1，i+2行左边线不是1且第i，i+2,i+4行的左边线依次变大，说明满足畸变
                    {
                        Left_Add_Stop = i+4*StepLine;	// 记录左侧补线结束行

                        //Line_Repair(Left_Add_Start, Left_Add_Stop, LeftEdge, Left_Add_Line, Left_Add_Flag, 1);
                    }
                }
            }
        }
        if (Right_Add_Flag[i] && i<StartLine)	// 右侧需要补线
        {
            if (i >= StartLine-3 && i < StartLine)	// 前三行补线不算
            {
                if (!Right_Add_Start && Right_Add_Flag[i-1] && Right_Add_Flag[i-2])
                {
                    Right_Add_Start = i;	// 记录补线开始行
                    Right_Ka = 0;
                    Right_Kb = Right_Add_Line[i];
                }
                Right_Add_Line[i] = Calculate_Add(i, Right_Ka, Right_Kb);	// 使用前一帧图像右边界斜率补线
            }
            else if(i < StartLine-3)

            {
                if (!Right_Add_Start)	// 之前没有补线
                {
                    Right_Add_Start = i;	// 记录右侧补线开始行
                    Curve_Fitting(&Right_Ka, &Right_Kb, &Right_Add_Start, Right_Add_Line, Right_Add_Flag, 2);	// 使用两点法拟合直线
                }
                Right_Add_Line[i] = Calculate_Add(i, Right_Ka, Right_Kb);	// 补线完成
            }
        }
        else
        {
            if (Right_Add_Start)	// 已经开始补线
            {
                if (!Right_Add_Stop &&
                        !Right_Add_Flag[i+StepLine] &&
                        !Right_Add_Flag[i+2*StepLine] &&
                        i<StartLine-5*StepLine)
                {
                    if ((Right_Add_Line[i] <= Right_Add_Line[i+1*StepLine] &&
                            Right_Add_Line[i+1*StepLine] <= Right_Add_Line[i+2*StepLine]
                            && Right_Add_Line[i]!=ColumnMax-1
                            && Right_Add_Line[i+1*StepLine]!=ColumnMax-1
                            && Right_Add_Line[i+2*StepLine]!=ColumnMax-1))
                    {
                        Right_Add_Stop = i+4*StepLine;	// 记录右侧补线结束行
                        //Right_Add_Start = 0;
                        //Line_Repair(Right_Add_Start, Right_Add_Stop, RightEdge, Right_Add_Line, Right_Add_Flag, 2);
                    }
                }
            }
        }
        //if (!Left_Add_Stop && Left_Add_Start && i==FinishLine)				Left_Add_Stop = FinishLine;
        //if (!Right_Add_Stop && Right_Add_Start && i==FinishLine)			Right_Add_Stop = FinishLine;
        /********************************* 第一轮补线结束 **********************************/
        if(Right_Add_Line[i] >= Left_Add_Line[i])
            Width_Add[i] = Right_Add_Line[i] - Left_Add_Line[i];	// 重新计算赛道宽度
        else
            Width_Add[i] = 0;
        if ((Left_Add_Flag[i] && Right_Add_Flag[i]) || (!Left_Add_Flag[i] && !Right_Add_Flag[i]))///左右同时需要补线或者同时不需要
            MiddleLine[i] = (Right_Add_Line[i] + Left_Add_Line[i]) / 2;	// 计算中线
        else if(i<StartLine)
            MiddleLine[i] = MiddleLine[i+StepLine];
        if (Width_Add[i] < Width_Min)
        {
            Width_Min = Width_Add[i];				// 更新最小赛道宽度
        }
        Line_Count = i;                          	//Line_Count更新
    }
    /*************************************************************************************/
    /*                              二次遍历结束                                         */
    /*************************************************************************************/

    /*************************************************************************************/
    /*                     三次遍历开始（边线修复+中线修复）                              */
    /*************************************************************************************/
    /******************************* 补线修复开始 ********************************/
    //if (Left_Add_Start && !Left_Add_Stop)		Left_Add_Stop = Line_Count;
    //if (Right_Add_Start && !Right_Add_Stop)		Right_Add_Stop = Line_Count;



    if (Left_Add_Start)		// 左边界需要补线
    {
        Line_Repair(Left_Add_Start, Left_Add_Stop, LeftEdge, Left_Add_Line, Left_Add_Flag, 1);
    }
    if (Right_Add_Start)	// 右边界需要补线
    {
        Line_Repair(Right_Add_Start, Right_Add_Stop, RightEdge, Right_Add_Line, Right_Add_Flag, 2);
    }
    /******************************* 补线修复结束 ********************************/
    /******************************* 中线修复开始 ********************************/
    for(i=StartLine; i>=Line_Count; i-=StepLine)
    {
        MiddleLine[i] = (Right_Add_Line[i] + Left_Add_Line[i]) / 2;	// 计算赛道中点
        if(Right_Add_Line[i] >= Left_Add_Line[i])
            Width_Add[i] = Right_Add_Line[i] - Left_Add_Line[i];	// 重新计算赛道宽度
        else
            Width_Add[i] = 0;
        if(SearchErrorFlag == i || CheckEnd(i-1,MiddleLine[i]) || i == FinishLine)///用这个动态前瞻判断三岔口的三角
        {
            LastLine = i;//最后一行，动态前瞻
            spurroadtriangle_i = i;
            spurroadtriangle_j = MiddleLine[i];
            AvaliableLines = StartLine - i;//有效行数
            break;
        }
    }
    /*for(i=StartLine;i>=FinishLine;i-=StepLine)
    {
    	//扫描完就最大有效行
    	if(i == FinishLine)
    	{
    		AvaliableLines = StartLine - FinishLine;
    		LastLine  = FinishLine;
    		break;
    	}
    	uint16 m = MiddleLine[i];
    	if(m < 5)					m = 5;
    	if(m > ColumnMax - 5)		m = ColumnMax - 5;
    	//动态前瞻
    	if(((LeftEdge[i]!=0 && LeftEdge[i]>=ColumnMax - 15) || 	//左边线越右界
    		(RightEdge[i]!=ColumnMax && RightEdge[i]<15) || 	//右边线越左界
    		(i>=1) && CheckEnd(i-1,MiddleLine[i]))						//中线所在列下一行丢失
    	   	&&i<StartLine-10)
    	{
    		LastLine = i;//最后一行，动态前瞻
    		AvaliableLines = StartLine - i;//有效行数
    		break;
    	}
    }*/

    /*************************************************************************************/
    /*                              三次遍历结束                                         */
    /*************************************************************************************/
    //oled画线
    for(k = FinishLine; k<=StartLine; k++)
    {
        for(j = 0; j<ColumnMax; j++)
        {
            if(//j==LeftEdge[k] || j==RightEdge[k] ||
                j==MiddleLine[k] || k==LastLine)

                Pixle[k][j] = 1;///屏幕上的中线///黑线
//		  else
//			Pixle[k][j] = 0;
        }

    }
}

/**
 * @file		提取图像的特征，获取黑线截止行
 * @note     	选取几列，从图像底近处往远扫描
 * @author		AHNU逐梦者（CYX）
 * @date		2019
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
    int j=0,k=0;
    int L1,L2,R1,R2;
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
 * @author		“芯”中有梦——汪旭
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
 * @author		AHNU逐梦者（CYX）
 * @date		2019
 */
void CircleConduct(void)
{
    static int ImgCount=0,count=0,LastRight=0;///,LastLastj=0
    int16 mincol=0;///,maxcol=0
    ///int j=0,Lastj=0,minimize=94,minLine=0;///i=0,
    int num1[ColumnMax];///,num2[ColumnMax]
    ///double XLose=0,X=0,Y=0,KK=0;///YLose=0,

    //LeftCircleFlag = MustBeCircle;
    //BeepFlag = 0;
    LED_Ctrl(LEDCORE,OFF);
    if((LeftCircleFlag >= ShouldBeCircle || RightCircleFlag >= ShouldBeCircle) &&
            ABS(LeftWheelSpeed)< 75)///ABS(LeftWheelSpeed) + ABS(RightWheelSpeed) < 150)
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
    /*************************************************************************/
    /*******************               左环               ********************/
    /*************************************************************************/
    if(LeftCircleFlag == MustBeCircle)
    {
        BeepFlag = 1;
        //if(ImgCount>300)		{LeftCircleFlag = NoCircle;ImgCount=count = 0;}
        /*if(Cirlce_Angle<20)
          SpeedParm = GearCtl.Circle;
        else
          SpeedParm = SelectMode;*/
        LED_Ctrl(LEDCORE,ON);
        //if(MIN(MIN(BlackEndL,BlackEndML),BlackEndLL)>30)	DrawOvalFlag=1;
        //if(MIN(BlackEndL,BlackEndML)>30)	DrawOvalFlag=1;

//		if(!DrawOvalFlag && CircleDistance>30 )//&& LeftEdge[StartLine]==1)
//		{
//		  //Cirlce_Angle = 0;
//		  DrawOvalFlag=1;
//		}
        if(Cirlce_Angle>45)
        {
            SpeedParm = SelectMode;
//		  	LED_Ctrl(LEDCORE,OFF);                                            //判断进岛
            LeftOvalFlag = 0;
            LeftCircleFlag = InCircle;
            ImgCount=count = 0;
        }
    }
    /*****************************确认入环**********************************/
    else if(LeftCircleFlag == InCircle)
    {
        //BeepFlag = 1;
        if(Cirlce_Angle>140)
        {
            //SpeedParm = GearCtl.Circle;
//			LED_Ctrl(LEDALL,ON);
            ImgCount=count=0;
            LeftCircleFlag = OutCircle;					//左右都丢线超过十行则开始出岛
        }
    }
    /*****************************判断出环**********************************/
    else if(LeftCircleFlag == OutCircle)
    {
//		BeepFlag = 0;
        //if(ImgCount>200)		{LeftCircleFlag = NoCircle;ImgCount=count = 0;}

        if(Cirlce_Angle>240)
        {
            DrawOvalFlag=0;
//		  SpeedParm = SelectMode;
            BeepFlag=0;
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
            BeepFlag = 0;
            ImgCount=count=0;
            LeftCircleFlag = NoCircle;    //一次之后就不判了
        }
    }
    /*************************************************************************/
    /*******************               右环               ********************/
    /*************************************************************************/
    /*****************************准备进环***********************************/
    if(RightCircleFlag == MustBeCircle)
    {
        BeepFlag = 1;
        //if(ImgCount>300)		{RightCircleFlag = NoCircle;ImgCount=count = 0;}
        /*if(Cirlce_Angle>-20)
          SpeedParm = GearCtl.Circle;
        else
          SpeedParm = SelectMode;*/
        //LED_Ctrl(LEDALL,ON);
        //if(MIN(MIN(BlackEndR,BlackEndMR),BlackEndRR)>30)	DrawOvalFlag=1;
        //if(MIN(BlackEndR,BlackEndMR)>30||RightLoseStart<40)	DrawOvalFlag=1;

//		if(!DrawOvalFlag && CircleDistance>30 )//&& RightEdge[StartLine]==ColumnMax-1)
//		{
//		  //Cirlce_Angle = 0;
//		  DrawOvalFlag=1;
//		}
        if(Cirlce_Angle<-45)
        {
            SpeedParm = SelectMode;
//            LED_Ctrl(LEDALL,OFF);                                               //判断进岛
            RightCircleFlag = InCircle;
            RightOvalFlag = 0;
            ImgCount=count = 0;
        }
    }
    /*****************************确认入环**********************************/
    else if(RightCircleFlag == InCircle)
    {
        //BeepFlag = 1;
        //if(ImgCount>100 && Cirlce_Angle>-120)	{RightCircleFlag = NoCircle;ImgCount=count = 0;}
        if(Cirlce_Angle<-140)
        {
            //SpeedParm = GearCtl.Circle;
//			LED_Ctrl(LEDALL,ON);
            ImgCount=count=0;
            RightCircleFlag = OutCircle;					//左右都丢线超过十行则开始出岛
        }
    }
    /*****************************判断出环**********************************/
    else if(RightCircleFlag == OutCircle)
    {
//		BeepFlag = 0;
        //if(ImgCount>200)		{RightCircleFlag = NoCircle;ImgCount=count = 0;}
        if(Cirlce_Angle<-240)
        {
            DrawOvalFlag=0;
//		  SpeedParm = SelectMode;
            BeepFlag=0;
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
            BeepFlag = 0;
            ImgCount=count=0;
            RightCircleFlag = NoCircle;   //一次之后就不判了
        }
    }
    ImgCount++;

    if(ImgCount>1500)
    {
        ImgCount=0;
        BeepFlag=0;
    }
}

/**
 * @file		起跑线检测
 * @note     	判断是不是起跑线
 * @author		AHNU逐梦者（CYX）
 * @date		2019
 */
void StartCheck(void)///int StartCheck(void)
{
///  static int ImgCount = 0;
    for(u16 i =StartLine; i>FinishLine; i--) ///从57到19 
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
 * @author		AHNU逐梦者（CYX）
 * @date		2019
 */
int16 FindOvalPoint(int16 col,int16 flag)///从列数的一半开始
{
    int16 i,j,k,num=0;
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

/**
 * @file		画椭圆
 * @note     	进环岛画椭圆
 * @author		AHNU逐梦者（CYX）---参考网传代码
 * @date		2019
 */
void oval(uint8 oval_x,uint8 oval_y,int8 zf)
{
    float x0,y0,x1,y1;
    y0=oval_y;
    x0=RowMax;
    x1=RowMax-oval_x;

    uint8 x,y;
    int16 xx;
#if ImageMode==2
    if(!zf)
    {
        y1=oval_y;
        for(float i=pi/2; i<=pi; i+=0.03)
        {
            y=(uint8)(y0+y1*cosf(i));
            x=(uint8)(x0-x1*sinf(i));
            y=RANGE16(y,10+oval_x/5,ColumnMax-2); //5+oval_x/5
            x=RANGE16(x,2,RowMax-1);
            Image_Sobel[x][y]=255;
            Image_Sobel[x-1][y]=255;
            Image_Sobel[x][y-1]=255;
            Image_Sobel[x-2][y]=255;
            Image_Sobel[x][y-2]=255;
            Image_Sobel[x-1][y-1]=255;
            Image_Sobel[x-2][y-2]=255;
            Image_Sobel[x-1][y-2]=255;
            Image_Sobel[x-2][y-1]=255;
            /*for(xx=x-3;xx>=0;xx--)
            {
            	Image_Sobel[xx][y]=0;
            	Image_Sobel[xx][y-1]=0;
            	Image_Sobel[xx][y-2]=0;
            }*/
        }
    }
    else if(zf>0)
    {
        y1=ColumnMax-oval_y;
        for(float i=pi/2; i<=pi; i+=0.03)
        {
            y=(uint8)(y0-y1*cosf(i));
            x=(uint8)(x0-x1*sinf(i));
            y=RANGE16(y,2,ColumnMax-10-oval_x/5);//123-oval_x/5
            x=RANGE16(x,2,RowMax-1);
            Image_Sobel[x  ][y  ]=255;
            Image_Sobel[x-1][y  ]=255;
            Image_Sobel[x  ][y-1]=255;
            Image_Sobel[x-2][y  ]=255;
            Image_Sobel[x  ][y-2]=255;
            Image_Sobel[x-1][y-1]=255;
            Image_Sobel[x-2][y-2]=255;
            Image_Sobel[x-1][y-2]=255;
            Image_Sobel[x-2][y-1]=255;
            /*for(xx=x-3;xx>=0;xx--)
            {
            	Image_Sobel[xx][y]=0;
            	Image_Sobel[xx][y-1]=0;
            	Image_Sobel[xx][y-2]=0;
            }*/
        }
    }
#elif ImageMode==1
    if(!zf)
    {
        y1=oval_y;
        for(float i=pi/2; i<=pi; i+=0.03)
        {
            y=(uint8)(y0+y1*cosf(i));
            x=(uint8)(x0-x1*sinf(i));
            y=RANGE16(y,2+oval_x/5,ColumnMax-2); //5+oval_x/5
            x=RANGE16(x,2,RowMax);
            Image_Use[x][y]=0;
            Image_Use[x-1][y]=0;
            Image_Use[x][y-1]=0;
            Image_Use[x-2][y]=0;
            Image_Use[x][y-2]=0;
            Image_Use[x-1][y-1]=0;
            Image_Use[x-2][y-2]=0;
            Image_Use[x-1][y-2]=0;
            Image_Use[x-2][y-1]=0;
        }
    }
    else if(zf>0)
    {
        y1=ColumnMax-oval_y;
        for(float i=pi/2; i<=pi; i+=0.03)
        {
            y=(uint8)(y0-y1*cosf(i));
            x=(uint8)(x0-x1*sinf(i));
            y=RANGE16(y,2,ColumnMax-2-oval_x/5);//123-oval_x/5
            x=RANGE16(x,2,RowMax);
            Image_Use[x][y]=0;
            Image_Use[x-1][y]=0;
            Image_Use[x][y+1]=0;
            Image_Use[x-2][y]=0;
            Image_Use[x][y+2]=0;
            Image_Use[x-1][y+1]=0;
            Image_Use[x-2][y+2]=0;
            Image_Use[x-1][y+2]=0;
            Image_Use[x-2][y+1]=0;
        }
    }
#endif
}

void DrawLine(int16 x0,int16 y0,int16 x1,int16 y1)
{
    int16 x,y,xx;
///  	int16 yy;///
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
//		for(yy=y-6;yy>=0;yy--)
//		{
//            Image_Sobel[yy  ][x]=0;
//            Image_Sobel[yy][x-1]=0;
//            Image_Sobel[yy][x-2]=0;
//		}
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

u16 CountRowW2BSalation(u16 line,u16 start,u16 finish)
{
    u16 num=0;
    start=RANGE16(start,0,ColumnMax-1);
    finish=RANGE16(finish,0,ColumnMax-1);
    for(int j=start; j<finish; j++)
    {
        if(CheckWhitePixle(line,j) && CheckBlackPixle(line,j+1) )//&& CheckBlackPixle(line,j+2))
            num++;
    }
    return num;
}

u16 CountRowB2WSalation(u16 line,u16 start,u16 finish)
{
    u16 num=0;
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
* 功能说明：显示图像到OLED模块
***************************************************************/
void Draw_Road(void)
{
    u8 i = 0, j = 0,temp=0;
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
    for(i=0; i<RowMax; i++) ///必须独立循环显示
    {
        ips200_drawpoint((uint16)AverageCenter, i, RED);
    }
///  for(i=0;i<ColumnMax;i++)///必须独立循环显示///
///    {
///        ips200_drawpoint( i,(uint16)RowMax/2, RED);
///    }
    sprintf((char*)txt,"%4.2f",AverageCenter);
    ips200_showstr(96,10,txt);
    sprintf((char*)txt,"%4d",Foresight);
    ips200_showstr(96,11,txt);
    sprintf((char*)txt,"%4d, %4d",StartLineCount, StationLineNum);
    ips200_showstr(96,12,txt);
    sprintf((char*)txt,"%4d, %4d",LeftLose, RightLose);
    ips200_showstr(96,13,txt);
}

/***************************************************************
* 函数名称：void Get_01_Value(void)
* 功能说明：按照均值的比例进行二值化
* 参数说明： limit-二值化阈值，height/width像素数组大小，s二值化模式选择,Image_Trans为发送上位机查看数组
* 函数返回：
* 修改时间：2018年12月16日
* 备 注：（D. By陈玉熙）参照论文《不同光照条件下二值化技术研究及应用》
          s为0时正常二值化，
          s为1时只将低于阈值像素二值化，高于阈值像素保留
          s为2时只将高于阈值像素二值化，低于阈值像素保留
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
    int i,j;///int i,j,k,ii,kk,num,num1,num2,startcol,lastcol,firstcol;
    ///int salacolL[5],salacolR[5],numsala=0,max=0;///int salacolL[5],salacolR[5],numsala=0,max=0;
    ///int32 Graynum=0;
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
uint16_t Sobel(int16 start,int16 finish)
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

    return((uint16_t)g_sum);
}

/***************************************************************
* 函数名称：void BinaryImage(uint8_t tmImage[IMAGEH][IMAGEW])
* 功能说明：图像数据二值化
* 参数说明：
* 函数返回：void
* 修改时间：2018年3月27日
* 备 注：
***************************************************************/
void BinaryImage(uint8_t tmImage[IMAGEH][IMAGEW],uint8_t ThresholdV)
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
* 参数说明：
* 函数返回：无
* 修改时间：2018年3月27日
* 备 注：
***************************************************************/
void Get_Use_Image(void)
{
    int i = 0,j = 0,row = 0,line = 0;///row是高，line是宽

    for(i = 0; i  < IMAGEH; i+=2)  //120行，每2行采集一行，///只取原图像的1/2
    {
        for(j = 0; j < IMAGEW; j+=2) //188，
        {
            Image_Use[row][line] = mt9v03x_image[i][j];
            line++;
        }
        line = 0;
        row++;
    }
}



//-------------------------------------------------------------------------------------------------------------------
//  @name    	TestCamera
//  @brief   	摄像头测试函数
//  @param
//  @return
//  @since      v1.0
//  @note
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void TestCamera(void)
{
    CameraInit();
    while(TRUE)
    {
//	    if(mt9v03x_finish_flag)
//	    {
//	        ips200_showstr(140,5,"xiaoxuxu");
////		ShowZoomImage(&mt9v03x_image[0], 188, 120, 94, 60);
////		CameraTest(0);///
//
////		Get_Use_Image();                                        //获取图像数据
////		Threshold = (uint8_t)Sobel(1,RowMax-1);          		//Sobel算子法
////		Pixle_Filter(Threshold);                          //Pixle_Filter(FilterThreshold);
////		GetBlackEndParam();                                     //抽样提取黑线截止行
////		Get_01_Value(Threshold,RowMax,ColumnMax);				//二值化图像数据
////
////		SearchCenterBlackline();								//寻中线
////		CircleConduct();										//对环岛进行处理
////		WeightedAverageCalc(2);									//中线加权归一
////		Middle_Err_Filter(AverageCenter);						//中线滤波
////		TrackJudge();											//赛道类型判断
////		LastAverageCenter = AverageCenter;						//中线保存
//		Draw_Road();
////		ips200_displayimage032_zoom1(&Image_Sobel[0],188, 120,50,50,94,60);
//		mt9v03x_finish_flag = 0;
//	    }
        if(GET_KEYCODE()==MY_KEY_CANCLE)
            break;
    }
}
