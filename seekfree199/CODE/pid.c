/*
 * Dream-Seekers-pid.c
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

#include "headfile.h"
#include "pid.h"
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
  register int iError,dError,place;
  iError = NextPoint - NowData;					//偏差
  SPID->SumError += iError; 					//积分
  dError = iError - SPID->LastError; 			//微分
  SPID->LastError = iError;
  place = (int16)(SPID->Proportion * iError 	//比例项
       + SPID->Integral * SPID->SumError 		//积分项
       + SPID->Derivative * dError); 			//微分项
  return(place);
}

