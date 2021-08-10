/*
 * servo.c
 *
 *  Created on: 2021年5月9日
 *      Author: 朱江禹
 */

#include "servo.h"

uint32 servo_duty;
uint32 sduty_last[3];
uint32 servo_ref[3] =
    {5100, 6140, 7180};
S_PID spid;
extern float32 error[5];

void Servo_Init(void)
{
    gtm_pwm_init(SERVO_PIN, SERVO_FREQ, servo_ref[1]);
    sduty_last[0] =
        sduty_last[1] =
            sduty_last[2] = servo_ref[1];
    servo_duty = servo_ref[1];
}

void Servo_Duty(void)
{
    if (servo_duty > servo_ref[2])
        servo_duty = servo_ref[2];
    else if (servo_duty < servo_ref[0])
        servo_duty = servo_ref[0];
    pwm_duty(SERVO_PIN, servo_duty);
}

void Spid_Init(void)
{
    spid.s_p = 50;
    spid.s_i = 0.0;
    spid.s_d = 0;
    spid.sum = spid.differ = 0;
}

void Servo_Pid(uint32 *pduty, S_PID *ps)
{
    float32 newduty;
    ps->sum = error[0] + error[1] + error[2] + error[3] + error[4]; //积分
    ps->differ = error[4] - error[3];                               //微分
    newduty = (float32)(servo_ref[1]) + error[4] * ps->s_p + ps->sum * ps->s_i + ps->differ * ps->s_d;
    newduty = 0.1 * sduty_last[0] + 0.2 * sduty_last[1] + 0.3 * sduty_last[2] + 0.4 * newduty;
    sduty_last[0] = sduty_last[1];
    sduty_last[1] = sduty_last[2];
    sduty_last[2] = newduty;
    *pduty = (int32)newduty;
}
