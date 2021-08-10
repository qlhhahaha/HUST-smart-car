#include "headfile.h"
#include "Motor.h"
//���õ���Ĳ���
int32 MOT_CFG[][2]=
{
    {MOT_Frequency, 		15000},	///		//���Ƶ��
    {MOT_MinDuty,          	/*-9990*/ 500},				//�����Сռ�ձ�(-10000~10000),������ʾ��ת
    {MOT_MaxDuty,          	/*9990*/  1000},			//������ռ�ձ�(-10000~10000)
};

//-------------------------------------------------------------------------------------------------------------------
//  @name    Motor_Init
//  @brief   �����ʼ��
//  @param
//  @return  void
//  @since      v1.0
//  @note
//  Sample usage:  Motor_Init()
//-------------------------------------------------------------------------------------------------------------------
void Motor_Init()
{
///	gtm_pwm_init(Motor_Module1, MOT_CFG[MOT_Frequency][1], 0, 0);
///	gtm_pwm_init(Motor_Module2, MOT_CFG[MOT_Frequency][1], 0, 0);
    gtm_pwm_init(Motor_Module3, MOT_CFG[MOT_Frequency][1], 1000,0);
    gtm_pwm_init(Motor_Module4, MOT_CFG[MOT_Frequency][1], 1000,0);
}

//-------------------------------------------------------------------------------------------------------------------
//  @name    Motor_Control
//  @brief   ���ռ�ձȿ���
//  @param   duty-ռ�ձ� (0-10000)
//  @return  void
//  @since      v1.0
//  @note
//  Sample usage:  Motor_Control(5000)
//-------------------------------------------------------------------------------------------------------------------
void Motor_Control(int32 dutyR)
{
    if(!(SpeedParm.ReverseFlag))
    {
        dutyR = RANGE32(dutyR, 0, MOT_CFG[MOT_MaxDuty][1]);
    }
    else
    {
        dutyR = RANGE32(dutyR, MOT_CFG[MOT_MinDuty][1], MOT_CFG[MOT_MaxDuty][1]);
    }

    if(dutyR>=0)
    {
        pwm_duty(Motor_Module3, dutyR, 0);
        pwm_duty(Motor_Module4, 0, 0);
    }
    else
    {
        pwm_duty(Motor_Module3, 0, 0);
        pwm_duty(Motor_Module4, ABS(dutyR), 0);
    }

}

//-------------------------------------------------------------------------------------------------------------------
//  @name    TestMotor
//  @brief   �����ʼ��
//  @param   x-��Ч����
//  @return  void
//  @since      v1.0
//  @note
//  Sample usage:  TestMotor(0)
//-------------------------------------------------------------------------------------------------------------------
void TestMotor(void)
{
    int16 num=0;
    char show[20];
    Motor_Init();
    ips200_showstr(10,2,"Motor-Duty : 0");
    while(1)
    {
        if(GET_KEYCODE()==MY_KEY_UP)
        {
            num+=50;
            num = RANGE16(num, MOT_CFG[MOT_MinDuty][1], MOT_CFG[MOT_MaxDuty][1]);
            sprintf(show,"Motor-Duty : %5d",num);
            ips200_showstr(10,2,show);
            Motor_Control(num);
        }
        else if(GET_KEYCODE()==MY_KEY_DOWN)
        {
            num-=50;
            num = RANGE16(num, MOT_CFG[MOT_MinDuty][1], MOT_CFG[MOT_MaxDuty][1]);
            sprintf(show,"Motor-Duty : %5d",num);
            ips200_showstr(10,2,show);
            Motor_Control(num);
        }
        else if(GET_KEYCODE()==MY_KEY_CANCLE)
        {
            Motor_Control(0);
            break;
        }
    }
}


/****************�������*****************/
///int16 LeftMotorPWM=0;
int16 RightMotorPWM=0;

/****************�ٶ�����*****************/
float  Differential_P=0.0;
float LSpeedSet=0.0;
float RSpeedSet=0.0;
float SpeedSet=0.0;
int16 LastError=0;
int16 PreL_Speed[4];
int16 PreR_Speed[4];
extern int16 LeftWheelSpeed;
extern int16 RightWheelSpeed;

extern float ADCDirectionError;
extern float ServoDiff;
/**
 * @file		�ٶ���������
 * @note     	����������������ת���پ��������~
 */
void GetSpeedSet(int Error)
{
    if(0)
    {
        SpeedSet = (SpeedParm.MaxSpeed+SpeedParm.MinSpeed)/2;
        if(ABS(ADCDirectionError)>25)
        {
            if(ADCDirectionError>0)
            {
                ///LSpeedSet = SpeedSet + ((SpeedSet * ADCDirectionError * SpeedParm.Differential)/45.0);
                RSpeedSet = SpeedSet;
            }
            else
            {
                ///LSpeedSet = SpeedSet;
                RSpeedSet = SpeedSet - ((SpeedSet * ADCDirectionError * SpeedParm.Differential)/45.0);
            }
        }
        else
        {
            ///LSpeedSet = SpeedSet;
            RSpeedSet = SpeedSet;
        }
        ///LSpeedSet=RANGEfloat(LSpeedSet,SpeedParm.MinSpeed,SpeedParm.MaxSpeed);
        RSpeedSet=RANGEfloat(RSpeedSet,SpeedParm.MinSpeed,SpeedParm.MaxSpeed);
//		LSpeedSet=(int16)(SpeedSet-(Differential_P*ADCDirectionError*SpeedSet));	//���ֲ���
//		RSpeedSet=(int16)(SpeedSet+(Differential_P*ADCDirectionError*SpeedSet));	//���ֲ���
    }
    else
    {
        if(Track_Type==CurveToStraight)
        {
            SpeedSet += 50.0;
        }
        else if (Foresight >= 18)
        {
            SpeedSet = (float)SpeedParm.MinSpeed;
        }
        else if (Foresight >= 16)
        {
            SpeedSet -= 14.0;
        }
        else if (Foresight >= 14)
        {
            SpeedSet -= 12.0;
        }
        else if (Foresight >= 12)
        {
            SpeedSet -= 10.0;
        }
        else if (Foresight >= 10)
        {
            SpeedSet -= 8.0;
        }
        /*else if (Foresight >= 8)
        {
        	SpeedSet -= 6.4;
        	if(SpeedSet>((SpeedParm.MaxSpeed+SpeedParm.MinSpeed)/2))
        	{
        		SpeedSet=(float)SpeedParm.MinSpeed+10.0;
        	}
        }*/
        else
        {
            SpeedSet = (float)SpeedSet + 20.0;
        }
        SpeedSet=RANGEfloat(SpeedSet,SpeedParm.MinSpeed,SpeedParm.MaxSpeed);
//	if(StartFlag >= FinishRun)
//	{
//		SpeedSet = 0;

//		LSpeedSet=RANGEfloat(LSpeedSet,SpeedParm.MinSpeed,SpeedParm.MaxSpeed);
//		RSpeedSet=RANGEfloat(RSpeedSet,SpeedParm.MinSpeed,SpeedParm.MaxSpeed);
//	}
//	else
//	{
//		if(ServoDiff>= 0)///ת�����
//		{
//			RSpeedSet = SpeedSet - (SpeedSet * ServoDiff * SpeedParm.Differential)/45.0;
/////			LSpeedSet = SpeedSet;
/////		    LSpeedSet = SpeedSet - (SpeedSet * ServoDiff * SpeedParm.Differential)/45.0;///Differential����Ҫ����һ�����ȡSeepedSet�ͼ����ƽ����
//		}
//		else
//		{
//			RSpeedSet = SpeedSet - (SpeedSet * ServoDiff * SpeedParm.Differential)/45.0;
/////			LSpeedSet = SpeedSet + (SpeedSet * ServoDiff * SpeedParm.Differential)/45.0;
/////		    LSpeedSet = SpeedSet - (SpeedSet * ServoDiff * SpeedParm.Differential)/45.0;
//		}
//		if(StartFlag == FinishRun)
//		{
//			LSpeedSet = -100;
//		}
//		else if(StartFlag == FinishRun+1)
//		{
//			RSpeedSet = -100;
//		}
//	}
    }

    //�����˲�
//  PreL_Speed[3]=PreL_Speed[2];
//  PreL_Speed[2]=PreL_Speed[1];
//  PreL_Speed[1]=PreL_Speed[0];
//  PreL_Speed[0]=LSpeedSet;
//  LSpeedSet=PreL_Speed[0]*0.4+PreL_Speed[1]*0.3+PreL_Speed[2]*0.2+PreL_Speed[3]*0.1;

    PreR_Speed[3]=PreR_Speed[2];
    PreR_Speed[2]=PreR_Speed[1];
    PreR_Speed[1]=PreR_Speed[0];
    PreR_Speed[0]=RSpeedSet;
    RSpeedSet=PreR_Speed[0]*0.4+PreR_Speed[1]*0.3+PreR_Speed[2]*0.2+PreR_Speed[3]*0.1;

//  LSpeedSet = 60;
    RSpeedSet = 60;
    if(StartFlag==ReadyRun)
    {
        RSpeedSet = 0;
    }//{LSpeedSet=0;	RSpeedSet=0;}
    LastError = Error;                        ///δʹ���������
}


/**
 * @file		�������
 * @note     	������10ms��ʱ���ã���������ת��ת��ת---
 * @author
 * @date
 */
void MotorRun(int Error)
{
    GetSpeedSet(Error);														///����������ٶ�����//�������������ٶ�����ֵ
    Encoder_Count();														//��õ�ǰ���ת�ٵ�����ֵ
///	LeftMotorPWM += PID_Increase(&LeftMotorPID,LSpeedSet,WheelSpeed);	//��������ʽPID����
    RightMotorPWM += PID_Increase(&RightMotorPID,RSpeedSet,LeftWheelSpeed);//�ҵ������ʽPID����
    RightMotorPWM = RANGE32(RightMotorPWM, MOT_CFG[MOT_MinDuty][1], MOT_CFG[MOT_MaxDuty][1]);

    if(StartFlag == WithoutRun)
    {
        if (LeftWheelSpeed> 15)///(LeftWheelSpeed + RightWheelSpeed > 15)
            //	Motor_Control(2000,2000);
            StartFlag = StartRun;//������
    }
    else
        Motor_Control(RightMotorPWM);								//���PWM����
    if(StartFlag >= FinishRun && ABS(LeftWheelSpeed)< 40)//(StartFlag >= FinishRun && ABS(RightWheelSpeed) + ABS(LeftWheelSpeed) < 80)
    {
        SpeedParm = SelectMode;
    }
}


void ClearIncResult(void)
{
///  LeftMotorPWM = 0;
    RightMotorPWM = 0;
}
