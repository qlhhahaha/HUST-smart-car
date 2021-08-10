
#ifndef CODE_MOTOR_H_
#define CODE_MOTOR_H_

//#define Motor_Module1 ATOM1_CH5_P15_2///pwm4
//#define Motor_Module2 ATOM1_CH6_P15_3


//#define Motor_Module3 ATOM1_CH6_P15_3///ATOM1_CH6_P15_3///pwm1
//
//#define Motor_Module4 ATOM2_CH7_P15_4///ATOM2_CH7_P15_4///pwm2

#define Motor_Module3 ATOM0_CH2_P14_3
#define Motor_Module4 ATOM0_CH7_P20_8


//电机参数枚举
typedef enum
{
    MOT_Frequency=0,          //电机频率设置
    MOT_MinDuty,            //电机最大允许占空比
    MOT_MaxDuty,            //电机最小允许占空比
}Mot_CMD;

typedef struct SpeedStruct
{
  int16	MaxSpeed;
  int16	MinSpeed;
  int16 AdvanceParm;
  float Differential;
  u8	ReverseFlag;
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


void Motor_Init();
void Motor_Control(int32 dutyR);//, int32 dutyR);
void TestMotor(void);

void MotorRun(int Error);

#endif /* CODE_MOTOR_H_ */
