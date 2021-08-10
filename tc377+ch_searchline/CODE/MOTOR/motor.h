/*
 * motor.h
 *
 *  Created on: 2021年5月9日
 *      Author: 朱江禹
 */

#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "zf_gtm_pwm.h"

#define MOTOR_PINA ATOM0_CH7_P20_8
#define MOTOR_PINB ATOM0_CH2_P14_3
#define MOTOR_FREQ 15000

void Motor_Init(void);
void Motor_Duty(void);


/*qlh魔改部分*/
typedef struct SpeedStruct
{
  int16	MaxSpeed;
  int16	MinSpeed;
  int16 AdvanceParm;
  float Differential;
  uint8	ReverseFlag;
}
ControlStruct;

typedef struct GearSet
{
  ControlStruct Defaults;
  ControlStruct Driver1;
  ControlStruct Driver2;
  ControlStruct Driver3;
  ControlStruct Driver4;
  ControlStruct Driver5;
  ControlStruct Driver6;
  ControlStruct Lose;
  ControlStruct Block1;
  ControlStruct Block2;
  ControlStruct Magnetic;
  ControlStruct	Circle;
  ControlStruct	Ramp;
  ControlStruct	Zebra;
  ControlStruct Stop;
  ControlStruct Reverse;
}GearSet;

#endif
