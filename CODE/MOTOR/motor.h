/*
 * motor.h
 *
 *  Created on: 2021?¨º5??9??
 *       
 */

#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "common.h"
#include "zf_gtm_pwm.h"

#define MOTOR_PINA ATOM1_CH0_P20_12
#define MOTOR_PINB ATOM1_CH1_P20_13
#define MOTOR_FREQ 15000

typedef struct
{
    float32 m_p;
    float32 m_i;
    float32 m_d;
    int16 spd_tar;
    int16 spd_pre[5];
    int16 sum;
    int16 differ;
}M_PID;

void Motor_Init(void);
void Mpid_Init(void);
void Motor_Duty(void);
void Motor_Pid(int32* pduty,int16 spd_now,M_PID* pm);

#endif
