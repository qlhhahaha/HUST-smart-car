/*
 * Dream-Seekers-pid.h
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

#ifndef CODE_PID_H_
#define CODE_PID_H_


typedef struct PID
{
  long SumError; //误差累计
  int LastError; //Error[-1]
  int PrevError; //Error[-2]
  double Proportion; //比例常数 Proportional Const
  double Integral; //积分常数 Integral Const
  double Derivative; //微分常数 Derivative Const
} PID;

extern void PID_Init(PID *SPID);
extern int16 PID_Increase(PID *SPID,int16 NextPoint,int16 NowData);
extern int16 PID_Realize(PID *SPID,int16 NextPoint,int16 NowData);

#endif /* CODE_PID_H_ */
