

#ifndef CODE_SERVO_H_
#define CODE_SERVO_H_

#define Servo_Module ATOM1_CH1_P21_3

//�������ö��
typedef enum
{
	SERVO_Frequency=0,          //���Ƶ������
	SERVO_MidDuty,            	//�����ֵռ�ձ�
	SERVO_MinDuty,         	 	//�����С����ռ�ձ�
	SERVO_MaxDuty,            	//����������ռ�ձ�
}Servo_CMD;

void Servo_Init();
void Servo_Control(uint32 duty);
void TestServo(void);


void ServoOut(void);
void ADC_ServoControl(void);
void ServoControl(float CalcCenter);

#endif /* CODE_SERVO_H_ */
