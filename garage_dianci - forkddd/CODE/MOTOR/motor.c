/*
 * motor.c
 *
 *  Created on: 2021年5月9日
 *      Author: 朱江禹
 */

#include "motor.h"
#include "enc.h"

int32 motor_duty;
float32 mduty_last[3];
M_PID mpid;

extern int16 pwm_stop_flag;

extern int16 turn_speed_flag;

void Motor_Init(void)
{
    gtm_pwm_init(MOTOR_PINA, MOTOR_FREQ, 0);
    gtm_pwm_init(MOTOR_PINB, MOTOR_FREQ, 0);
    mduty_last[0] =
        mduty_last[1] =
            mduty_last[2] = 0.0;
    motor_duty = 0;
}

void Mpid_Init(void)
{
    mpid.m_p = 20.5;
    mpid.m_i = 0.0;
    mpid.m_d = 5;
    mpid.spd_tar = 90;
    mpid.spd_pre[0] = mpid.spd_pre[1] =
        mpid.spd_pre[2] = mpid.spd_pre[3] =
            mpid.spd_pre[4] = 0;
    mpid.sum = mpid.differ = 0;
}

void Motor_Duty(void)
{
    if (pwm_stop_flag == 0 && turn_speed_flag ==0)
    {
        if (motor_duty < -3800)
            motor_duty = -3800;
        if (motor_duty > 3800)
            motor_duty = 3800;
        if (motor_duty < 0)
        {
            pwm_duty(MOTOR_PINA, 0);
            pwm_duty(MOTOR_PINB, -motor_duty);
        }
        else
        {
            pwm_duty(MOTOR_PINA, motor_duty);
            pwm_duty(MOTOR_PINB, 0);
        }
    }

    if (pwm_stop_flag == 0 &&turn_speed_flag == 1)
    {
        if (motor_duty < -3750)
            motor_duty = -3750;
        if (motor_duty > 3750)
            motor_duty = 3750;
        if (motor_duty < 0)
        {
            pwm_duty(MOTOR_PINA, 0);
            pwm_duty(MOTOR_PINB, -motor_duty);
        }
        else
        {
            pwm_duty(MOTOR_PINA, motor_duty);
            pwm_duty(MOTOR_PINB, 0);
        }
    }
}

void Motor_Pid(int32 *pduty, int16 spd_now, M_PID *pm)
{
    if (pwm_stop_flag == 0 && turn_speed_flag ==0)
       {
            mpid.spd_tar =60;
            //mpid.spd_tar =0;
       }

       if (pwm_stop_flag == 0 &&turn_speed_flag == 1)
       {
           mpid.spd_tar = 59;
          // mpid.spd_tar =0;
       }

    float32 newduty;
    pm->spd_pre[0] = pm->spd_pre[1];
    pm->spd_pre[1] = pm->spd_pre[2];
    pm->spd_pre[2] = pm->spd_pre[3];
    pm->spd_pre[3] = pm->spd_pre[4];
    pm->spd_pre[4] = spd_now;                                                                                       //数据更新
    pm->sum = 5 * pm->spd_tar - pm->spd_pre[0] - pm->spd_pre[1] - pm->spd_pre[2] - pm->spd_pre[3] - pm->spd_pre[4]; //积分
    pm->differ = pm->spd_pre[3] - pm->spd_pre[4];                                                                   //微分
    newduty = (float32)(*pduty) + (pm->spd_tar - pm->spd_pre[4]) * pm->m_p + pm->sum * pm->m_i + pm->differ * pm->m_d;
    newduty = 0.1 * mduty_last[0] + 0.2 * mduty_last[1] + 0.3 * mduty_last[2] + 0.4 * newduty;
    mduty_last[0] = mduty_last[1];
    mduty_last[1] = mduty_last[2];
    mduty_last[2] = newduty;
    *pduty = (int32)newduty;
}
