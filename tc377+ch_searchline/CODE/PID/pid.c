
#include "headfile.h"
#include "common.h"

//-------------------------------------------------------------------------------------------------------------------
//  @name    PID_Init
//  @brief   PID 参数初始化
//  @param  		    
//  @return     				
//  @since      v1.0
//  @note      
//  Sample usage:               
//-------------------------------------------------------------------------------------------------------------------
void PID_Init(PID *SPID)
{
    SPID->SumError = 0;
    SPID->LastError = 0; //Error[-1]
    SPID->PrevError = 0; //Error[-2]
    SPID->Proportion = 0; //比例常数 Proportional Const
    SPID->Integral = 0; //积分常数 Integral Const
    SPID->Derivative = 0; //微分常数 Derivative Const
}

//-------------------------------------------------------------------------------------------------------------------
//  @name    PID_Increase
//  @brief   增量式 PID 控制设计
//  @param
//  @return
//  @since      v1.0
//  @note
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
int16 PID_Increase(PID *SPID,int16 NextPoint,int16 NowData)
{
    register int iError,iIncpid;                        									//当前误差
    iError = NextPoint - NowData;                  										//增量计算
    iIncpid = (int16)((SPID->Proportion + SPID->Integral + SPID->Derivative) * iError		//E[k]项
            -(SPID->Proportion + SPID->Derivative * 2             ) * SPID->LastError	//E[k－1]项
            +(SPID->Derivative								 	 ) * SPID->PrevError);	//E[k－2]项
    SPID->PrevError = SPID->LastError;                    								//存储误差，用于下次计算
    SPID->LastError = iError;
    return(iIncpid);   																	//返回增量值
}

//-------------------------------------------------------------------------------------------------------------------
//  @name    PID_Realize
//  @brief   位置式 PID 控制设计
//  @param
//  @return
//  @since      v1.0
//  @note
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
int16 PID_Realize(PID *SPID,int16 NextPoint,int16 NowData)
{
    register int iError,dError,place,a;
    iError = NextPoint - NowData;				//偏差
    SPID->SumError += iError; 					//积分

    SPID->SumError = RANGE32(SPID->SumError,-ColumnMax*10*2/2,ColumnMax*10*2/2);
    //积分限幅，防止积分过饱和，反应迟钝

    dError = iError - SPID->LastError; 			//微分
    SPID->LastError = iError;

    //在系统误差较大时，取消积分环节；当误差较小时，引入积分环节
    //这样既不影响控制器的动态性能，又可以提高控制器的稳态精度
    //每一次的当前误差大于460/2，即大于四分之一个赛道时，不进行积分
    if (iError > ColumnMax*10/4 || iError < -ColumnMax*10/4)
        a = 0;
    else 
        a = 1;

    place = (int16)(SPID->Proportion * iError 	//比例项
        + SPID->Integral * SPID->SumError *a 		//积分项
        + SPID->Derivative * dError); 			//微分项
    return(place);
}

