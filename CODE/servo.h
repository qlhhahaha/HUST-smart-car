

#ifndef CODE_SERVO_H_
#define CODE_SERVO_H_

#define Servo_Module ATOM1_CH1_P21_3

//电机参数枚举
typedef enum
{
	SERVO_Frequency=0,          //舵机频率设置
	SERVO_MidDuty,            	//舵机中值占空比
	SERVO_MinDuty,         	 	//舵机最小允许占空比
	SERVO_MaxDuty,            	//舵机最大允许占空比
}Servo_CMD;

void Servo_Init();
void Servo_Control(uint32 duty);
void TestServo(void);


void ServoOut(void);
void ADC_ServoControl(void);
void ServoControl(float CalcCenter);

#endif /* CODE_SERVO_H_ */
