
#ifndef PID_H_
#define PID_H_

#include "common.h"
#include "key.h"
#include "motor.h"

typedef struct PID
{
    long SumError; //误差累计
    int LastError; //Error[-1]
    int PrevError; //Error[-2]
    double Proportion; //比例常数 Proportional Const
    double Integral; //积分常数 Integral Const
    double Derivative; //微分常数 Derivative Const
} PID;

void PID_Init(PID *SPID);
int16 PID_Increase(PID *SPID,int16 NextPoint,int16 NowData);
int16 PID_Realize(PID *SPID,int16 NextPoint,int16 NowData);

#endif
