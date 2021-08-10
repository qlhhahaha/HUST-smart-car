/*
 * Dream-Seekers-ultrasonic.c
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
#include "ultrasonic.h"
uint8 ultrasonic_buff[10];
uint16 distance;
//-------------------------------------------------------------------------------------------------------------------
//  @name   	Ultrasonic_ReadData
//  @brief   	超声波读取串口数据
//  @param  		    
//  @return     				
//  @since      v1.0
//  @note
//  Sample usage:               
//-------------------------------------------------------------------------------------------------------------------
uint8 Ultrasonic_ReadData(void)
{
	uint8 res=0,*ptr;
	if(ULTRASONIC_GetChar(&res))
	{
		ptr = ultrasonic_buff;
		*(ptr++) = res;						//开始接受
		delay_ms(1);

		if(ultrasonic_buff[0] != 0xa5)
			return 0;						//检查头帧是否正确，不正确就重新接收
		else
		{
			ULTRASONIC_GetChar(&res);			//直到接受完成
			*(ptr++) = res;
			ULTRASONIC_GetChar(&res);			//直到接受完成
			*(ptr++) = res;
			distance = ultrasonic_buff[1]<<8 | ultrasonic_buff[2];
			return 1;
		}
		return 0;
	}
	else
		return 0;
}


//-------------------------------------------------------------------------------------------------------------------
//  @name   	Test_Ultrasonic
//  @brief   	测试超声波
//  @param
//  @return
//  @since      v1.0
//  @note
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void Test_Ultrasonic(void)
{
	ULTRASONIC_Com_Init(115200);
	while(1)
	{
		if(Ultrasonic_ReadData())
		{
			printf((char*)ultrasonic_buff);
			char dat[50];sprintf(dat,"REC:%4d",distance);
			ULTRASONIC_ShowStr(10,14,dat);
			memset(ultrasonic_buff,0,strlen(ultrasonic_buff)*sizeof(uint8));
		}
		if(GET_KEYCODE()==MY_KEY_CANCLE)
			break;
	}

}
