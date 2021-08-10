/*
 * servo.h
 *
 *  Created on: 2021?¨º5??9??
 *       
 */

#ifndef _SERVO_H_
#define _SERVO_H_

#include "zf_gtm_pwm.h"

#define SERVO_PIN ATOM0_CH0_P22_1
#define SERVO_FREQ 200

typedef struct
{
    float32 s_p;
    float32 s_i;
    float32 s_d;
    float32 sum;
    float32 differ;
}S_PID;

void Servo_Init(void);
void Servo_Duty(void);
void Spid_Init(void);
void Servo_Pid(uint32* pduty,S_PID* ps);

#endif
