/*
 * motor.c
 *
 *  Created on: 2021年5月9日
 *      Author: 朱江禹
 */

#include "motor.h"

int32 motor_duty;

void Motor_Init(void)
{
    gtm_pwm_init(MOTOR_PINA,MOTOR_FREQ,0);
    gtm_pwm_init(MOTOR_PINB,MOTOR_FREQ,0);
    motor_duty=0;
}

void Motor_Duty(void)
{
    if(motor_duty<0)
    {
        pwm_duty(MOTOR_PINA,0);
        pwm_duty(MOTOR_PINB,-motor_duty);
    }
    else
    {
        pwm_duty(MOTOR_PINA,motor_duty);
        pwm_duty(MOTOR_PINB,0);
    }
}
