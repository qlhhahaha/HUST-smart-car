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
	GYRO_Real,		// 陀螺仪转化后的数据
	ACC_Real,		// 加速度计转化后的数据
	Attitude_Angle,	// 当前角度
	Last_Angle,		// 上次角度
	Target_Angle;	// 目标角度


S_INT16_XYZ
	GYRO,			// 陀螺仪原始数据
	GYRO_Offset,	// 陀螺仪温飘
	GYRO_Last,		// 陀螺仪上次数据
	ACC, 			// 加速度计数据
	ACC_Offset,		// 加速度计温飘
	ACC_Last;		// 加速度计上次数据
S_INT32_XYZ
	Tar_Ang_Vel,	// 目标角速度
	Tar_Ang_Vel_Last;	// 上次目标角速度

int32
	Speed_Now = 0,	// 当前实际速度
	Speed_Min = 0,	// 左右最小速度
	Speed_Set = 0, 	// 目标设定速度
	Theory_Duty = 0,// 理论直立占空比
	Vel_Set = 0,	// 目标转向角速度
	Direct_Parameter = 0,// 转向系数
	Direct_Last = 0,
	Radius = 0;		// 目标转向半径倒数

char Offset_OK = 0;

#define DELAY_MS(x)		delay(x)

/*
 * 函数名：GetData
 * 描述  ：获得16位数据
 * 输入  ：REG_Address 寄存器地址
 * 输出  ：返回寄存器数据
 * 调用  ：外部调用
 */
int16 GetData(u8 REG_Address)
{
	u8 H, L;

	H = MPU6050_RD(REG_Address);
	L = MPU6050_RD(REG_Address+1);

	return ((H<<8)|L);   //合成数据
}

/*
 * 函数名：MPU6050_GetData
 * 描述  ：获得传感器所有数据
 * 输入  ：*GYRO 陀螺仪		*ACC 加速度计
 * 输出  ：无
 * 调用  ：外部调用
 */
void MPU6050_GetData(S_INT16_XYZ *GYRO, S_INT16_XYZ *ACC)
{
	if (Offset_OK)
	{
		ACC->X = GetData(MPU_ACCEL_XOUTH_REG);	// 获取加速度计原始数据
		ACC->Y = GetData(MPU_ACCEL_YOUTH_REG);
		ACC->Z = GetData(MPU_ACCEL_ZOUTH_REG);

		GYRO->X = GetData(MPU_GYRO_XOUTH_REG) - GYRO_Offset.X;	// 获取陀螺仪原始数据
		GYRO->Y = GetData(MPU_GYRO_YOUTH_REG) - GYRO_Offset.Y;
		GYRO->Z = GetData(MPU_GYRO_ZOUTH_REG) - GYRO_Offset.Z;
	}
	else
	{
		ACC->X = GetData(MPU_ACCEL_XOUTH_REG);	// 获取加速度计原始数据并归一化
		ACC->Y = GetData(MPU_ACCEL_YOUTH_REG);
		ACC->Z = GetData(MPU_ACCEL_ZOUTH_REG);

		GYRO->X = GetData(MPU_GYRO_XOUTH_REG);	// 获取陀螺仪原始数据并归一化
		GYRO->Y = GetData(MPU_GYRO_YOUTH_REG);
		GYRO->Z = GetData(MPU_GYRO_ZOUTH_REG);
	}
}


/*
 * 函数名：MPU6050_Offset
 * 描述  ：传感器采集零偏
 * 输入  ：无
 * 输出  ：无
 * 调用  ：内部调用
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
		MPU6050_GetData(&GYRO, &ACC);	// 读取陀螺仪数据
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


/************************ 陀螺仪 初始化 **********************************/
void MPU6050_Init(void)
{
  	int i=0,j=0;
	MPU6050_OPEN();
	while (mpu_dmp_init());		// 初始化DMP
	MPU6050_Offset();

	/*for (i = 0; i < 400; i++)
	{
		for (j = 0; j < 5; j++)
		{
			MPU6050_GetData(&GYRO, &ACC);	// 读取陀螺仪数据
			Data_Filter();					// 对原始数据滑动滤波
			mpu_dmp_get_data(&Attitude_Angle.Y, &Attitude_Angle.X, &Attitude_Angle.Z);	// 使用DMP直接读取欧拉角
		}
//		Get_Attitude();	// 姿态解算
//		Tar_Ang_Vel.Y = PID_Realize(&Angle_PID, Angle, (int32)(Attitude_Angle.Y*100), Zero_Angle*100);
	}*/
}


float Acc_Z_Value[3];
float Gyro_Z_Value[3];
float Gyro_Y_Value[3];
void GetAngleSpeedValue()//获取Gyro原始数据
{
  MPU6050_GetData(&GYRO, &ACC);			//读取陀螺仪数据

  Acc_Z_Value[0] = Acc_Z_Value[1];
  Acc_Z_Value[1] = Acc_Z_Value[2];
  Acc_Z_Value[2] = ACC.Z;//本句是获取当前加速度  上两句都是 上次值和上上次值

  Gyro_Y_Value[0] = Gyro_Y_Value[1];
  Gyro_Y_Value[1] = Gyro_Y_Value[2];
  Gyro_Y_Value[2] = GYRO.Y ;             //本句是获取当前Gyro上两句都是 上次值和上上次值

  Gyro_Z_Value[0] = Gyro_Z_Value[1]; //前前次
  Gyro_Z_Value[1] = Gyro_Z_Value[2]; //前次
  Gyro_Z_Value[2] = GYRO.Z ;          //本次
};

void AngleSoftwareFilter()				//本函数用于数字陀螺仪 通过软件互补融合滤波解算角度
{
	//加速度  角速度采集
  ACC.Z = 0.9*Acc_Z_Value[2] + 0.05*Acc_Z_Value[1] + 0.05*Acc_Z_Value[0];     	//权值低通滤波
  GYRO.Z = 0.9*Gyro_Z_Value[2] + 0.05*Gyro_Z_Value[1] + 0.05*Gyro_Z_Value[0];	//权值低通滤波
  GYRO.Y = 0.92*Gyro_Y_Value[2] + 0.05*Gyro_Y_Value[1] + 0.03*Gyro_Y_Value[0];	//权值低通滤波
};


void Test_Gyroscope(void)
{

	///int16_t aacx,aacy,aacz;					//加速度传感器原始数据
	int16_t gyroy,gyroz;				//陀螺仪原始数据///gyrox,
	int16_t temp;                          //温度
	///uint16_t a[8];
	uint8_t  txt[30]="X:";

#if 0
	MPU6050_Init();                          //初始化MPU6050
	while(1)
	{
  //    temp=MPU_Get_Temperature();	//得到温度值
	  MPU_Get_Raw_data(&aacx,&aacy,&aacz,&gyrox,&gyroy,&gyroz);	//得到加速度传感器数据
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
	  	//MPU6050_GetData(&GYRO, &ACC);	// 读取陀螺仪数据   93142,101753
		//Data_Filter();					// 对原始数据滑动滤波
		GetAngleSpeedValue();					//读取陀螺仪数据
		AngleSoftwareFilter();					//数字低通滤波陀螺仪数据
		//Get_Attitude();
  		mpu_dmp_get_data(&Attitude_Angle.Y, &Attitude_Angle.X, &Attitude_Angle.Z);	// 使用DMP直接读取欧拉角 12213411
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

