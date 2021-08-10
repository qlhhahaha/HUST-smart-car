
#ifndef PID_H_
#define PID_H_

#include "common.h"
#include "key.h"
#include "motor.h"

typedef struct PID
{
    long SumError; //����ۼ�
    int LastError; //Error[-1]
    int PrevError; //Error[-2]
    double Proportion; //�������� Proportional Const
    double Integral; //���ֳ��� Integral Const
    double Derivative; //΢�ֳ��� Derivative Const
} PID;

void PID_Init(PID *SPID);
int16 PID_Increase(PID *SPID,int16 NextPoint,int16 NowData);
int16 PID_Realize(PID *SPID,int16 NextPoint,int16 NowData);

#endif
