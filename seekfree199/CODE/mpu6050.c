/*
 * Dream-Seekers-mpu6050.c
 * 
 *  Created on:
 *      Author:
 *     Version: V1.0
 *        Core: TC264D
 *
 *   	  Name:
 *		 Brief: 
 *	      Note:
 */

#include "headfile.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "mpu6050.h"
//-------------------------------------------------------------------------------------------------------------------
//  @name    
//  @brief   
//  @param  		    
//  @return     				
//  @since      v1.0
//  @note      
//  Sample usage:               
//-------------------------------------------------------------------------------------------------------------------

S_FLOAT_XYZ
	GYRO_Real,		// ������ת���������
	ACC_Real,		// ���ٶȼ�ת���������
	Attitude_Angle,	// ��ǰ�Ƕ�
	Last_Angle,		// �ϴνǶ�
	Target_Angle;	// Ŀ��Ƕ�


S_INT16_XYZ
	GYRO,			// ������ԭʼ����
	GYRO_Offset,	// ��������Ʈ
	GYRO_Last,		// �������ϴ�����
	ACC, 			// ���ٶȼ�����
	ACC_Offset,		// ���ٶȼ���Ʈ
	ACC_Last;		// ���ٶȼ��ϴ�����
S_INT32_XYZ
	Tar_Ang_Vel,	// Ŀ����ٶ�
	Tar_Ang_Vel_Last;	// �ϴ�Ŀ����ٶ�

int32
	Speed_Now = 0,	// ��ǰʵ���ٶ�
	Speed_Min = 0,	// ������С�ٶ�
	Speed_Set = 0, 	// Ŀ���趨�ٶ�
	Theory_Duty = 0,// ����ֱ��ռ�ձ�
	Vel_Set = 0,	// Ŀ��ת����ٶ�
	Direct_Parameter = 0,// ת��ϵ��
	Direct_Last = 0,
	Radius = 0;		// Ŀ��ת��뾶����

char Offset_OK = 0;

#define DELAY_MS(x)		delay(x)

/*
 * ��������GetData
 * ����  �����16λ����
 * ����  ��REG_Address �Ĵ�����ַ
 * ���  �����ؼĴ�������
 * ����  ���ⲿ����
 */
int16 GetData(u8 REG_Address)
{
	u8 H, L;

	H = MPU6050_RD(REG_Address);
	L = MPU6050_RD(REG_Address+1);

	return ((H<<8)|L);   //�ϳ�����
}

/*
 * ��������MPU6050_GetData
 * ����  ����ô�������������
 * ����  ��*GYRO ������		*ACC ���ٶȼ�
 * ���  ����
 * ����  ���ⲿ����
 */
void MPU6050_GetData(S_INT16_XYZ *GYRO, S_INT16_XYZ *ACC)
{
	if (Offset_OK)
	{
		ACC->X = GetData(MPU_ACCEL_XOUTH_REG);	// ��ȡ���ٶȼ�ԭʼ����
		ACC->Y = GetData(MPU_ACCEL_YOUTH_REG);
		ACC->Z = GetData(MPU_ACCEL_ZOUTH_REG);

		GYRO->X = GetData(MPU_GYRO_XOUTH_REG) - GYRO_Offset.X;	// ��ȡ������ԭʼ����
		GYRO->Y = GetData(MPU_GYRO_YOUTH_REG) - GYRO_Offset.Y;
		GYRO->Z = GetData(MPU_GYRO_ZOUTH_REG) - GYRO_Offset.Z;
	}
	else
	{
		ACC->X = GetData(MPU_ACCEL_XOUTH_REG);	// ��ȡ���ٶȼ�ԭʼ���ݲ���һ��
		ACC->Y = GetData(MPU_ACCEL_YOUTH_REG);
		ACC->Z = GetData(MPU_ACCEL_ZOUTH_REG);

		GYRO->X = GetData(MPU_GYRO_XOUTH_REG);	// ��ȡ������ԭʼ���ݲ���һ��
		GYRO->Y = GetData(MPU_GYRO_YOUTH_REG);
		GYRO->Z = GetData(MPU_GYRO_ZOUTH_REG);
	}
}


/*
 * ��������MPU6050_Offset
 * ����  ���������ɼ���ƫ
 * ����  ����
 * ���  ����
 * ����  ���ڲ�����
 */
void MPU6050_Offset(void)
{
	uint8 i, Count = 100;
	int64 temp[6] = {0};

	GYRO_Offset.X = 0;
	GYRO_Offset.Y = 0;
	GYRO_Offset.Z = 0;

	for (i = 0; i < Count; i++)
	{
		MPU6050_GetData(&GYRO, &ACC);	// ��ȡ����������
		DELAY_MS(2);

		temp[0] += ACC.X;
		temp[1] += ACC.Y;
		temp[2] += ACC.Z;

		temp[3] += GYRO.X;
		temp[4] += GYRO.Y;
		temp[5] += GYRO.Z;
	}
	ACC_Offset.X = temp[0] / Count;
	ACC_Offset.Y = temp[1] / Count;
	ACC_Offset.Z = temp[2] / Count;

	GYRO_Offset.X = temp[3] / Count;
	GYRO_Offset.Y = temp[4] / Count;
	GYRO_Offset.Z = temp[5] / Count;

	Offset_OK = 1;
}


/************************ ������ ��ʼ�� **********************************/
void MPU6050_Init(void)
{
  	int i=0,j=0;
	MPU6050_OPEN();
	while (mpu_dmp_init());		// ��ʼ��DMP
	MPU6050_Offset();

	/*for (i = 0; i < 400; i++)
	{
		for (j = 0; j < 5; j++)
		{
			MPU6050_GetData(&GYRO, &ACC);	// ��ȡ����������
			Data_Filter();					// ��ԭʼ���ݻ����˲�
			mpu_dmp_get_data(&Attitude_Angle.Y, &Attitude_Angle.X, &Attitude_Angle.Z);	// ʹ��DMPֱ�Ӷ�ȡŷ����
		}
//		Get_Attitude();	// ��̬����
//		Tar_Ang_Vel.Y = PID_Realize(&Angle_PID, Angle, (int32)(Attitude_Angle.Y*100), Zero_Angle*100);
	}*/
}


float Acc_Z_Value[3];
float Gyro_Z_Value[3];
float Gyro_Y_Value[3];
void GetAngleSpeedValue()//��ȡGyroԭʼ����
{
  MPU6050_GetData(&GYRO, &ACC);			//��ȡ����������

  Acc_Z_Value[0] = Acc_Z_Value[1];
  Acc_Z_Value[1] = Acc_Z_Value[2];
  Acc_Z_Value[2] = ACC.Z;//�����ǻ�ȡ��ǰ���ٶ�  �����䶼�� �ϴ�ֵ�����ϴ�ֵ

  Gyro_Y_Value[0] = Gyro_Y_Value[1];
  Gyro_Y_Value[1] = Gyro_Y_Value[2];
  Gyro_Y_Value[2] = GYRO.Y ;             //�����ǻ�ȡ��ǰGyro�����䶼�� �ϴ�ֵ�����ϴ�ֵ

  Gyro_Z_Value[0] = Gyro_Z_Value[1]; //ǰǰ��
  Gyro_Z_Value[1] = Gyro_Z_Value[2]; //ǰ��
  Gyro_Z_Value[2] = GYRO.Z ;          //����
};

void AngleSoftwareFilter()				//�������������������� ͨ����������ں��˲�����Ƕ�
{
	//���ٶ�  ���ٶȲɼ�
  ACC.Z = 0.9*Acc_Z_Value[2] + 0.05*Acc_Z_Value[1] + 0.05*Acc_Z_Value[0];     	//Ȩֵ��ͨ�˲�
  GYRO.Z = 0.9*Gyro_Z_Value[2] + 0.05*Gyro_Z_Value[1] + 0.05*Gyro_Z_Value[0];	//Ȩֵ��ͨ�˲�
  GYRO.Y = 0.92*Gyro_Y_Value[2] + 0.05*Gyro_Y_Value[1] + 0.03*Gyro_Y_Value[0];	//Ȩֵ��ͨ�˲�
};


void Test_Gyroscope(void)
{

	///int16_t aacx,aacy,aacz;					//���ٶȴ�����ԭʼ����
	int16_t gyroy,gyroz;				//������ԭʼ����///gyrox,
	int16_t temp;                          //�¶�
	///uint16_t a[8];
	uint8_t  txt[30]="X:";

#if 0
	MPU6050_Init();                          //��ʼ��MPU6050
	while(1)
	{
  //    temp=MPU_Get_Temperature();	//�õ��¶�ֵ
	  MPU_Get_Raw_data(&aacx,&aacy,&aacz,&gyrox,&gyroy,&gyroz);	//�õ����ٶȴ���������
	  sprintf((char*)txt,"ax:%06d",aacx);
	  LCD_P6x8Str(0,0,txt);
	  sprintf((char*)txt,"ay:%06d",aacy);
	  LCD_P6x8Str(0,1,txt);
	  sprintf((char*)txt,"az:%06d",aacz);
	  LCD_P6x8Str(0,2,txt);
	  sprintf((char*)txt,"gx:%06d",gyrox);
	  LCD_P6x8Str(0,3,txt);
	  sprintf((char*)txt,"gy:%06d",gyroy);
	  LCD_P6x8Str(0,4,txt);
	  sprintf((char*)txt,"gz:%06d",gyroz);
	  LCD_P6x8Str(0,5,txt);
	  a[0]=aacx;
	  a[1]=aacy;
	  a[2]=aacz;
	  a[3]=gyrox;
	  a[4]=gyroy;
	  a[5]=gyroz;
	  time_delay_ms(100);
	  if(!KEY_Read(KEY2))
	  {
		  break;
	  }
  	}
#else
	MPU6050_Init();
	while(1)
	{
	  	//MPU6050_GetData(&GYRO, &ACC);	// ��ȡ����������   93142,101753
		//Data_Filter();					// ��ԭʼ���ݻ����˲�
		GetAngleSpeedValue();					//��ȡ����������
		AngleSoftwareFilter();					//���ֵ�ͨ�˲�����������
		//Get_Attitude();
  		mpu_dmp_get_data(&Attitude_Angle.Y, &Attitude_Angle.X, &Attitude_Angle.Z);	// ʹ��DMPֱ�Ӷ�ȡŷ���� 12213411
		sprintf((char*)txt,"ax:%.3f",Attitude_Angle.Y);
		ips200_showstr(10,0,txt);
		sprintf((char*)txt,"ay:%.3f",Attitude_Angle.X);
		ips200_showstr(10,1,txt);
		sprintf((char*)txt,"az:%.3f",Attitude_Angle.Z);
		ips200_showstr(10,2,txt);
		sprintf((char*)txt,"Y:%3d , %6d",GYRO.Y,ACC.Y);
		ips200_showstr(10,4,txt);
		sprintf((char*)txt,"X:%3d , %6d",GYRO.X,ACC.X);
		ips200_showstr(10,5,txt);
		sprintf((char*)txt,"Z:%3d , %6d",GYRO.Z,ACC.Z);
		ips200_showstr(10,6,txt);
//		delay_ms(100);
  	}
#endif
}

