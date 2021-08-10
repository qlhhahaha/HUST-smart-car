
#include "headfile.h"
#include "common.h"

//-------------------------------------------------------------------------------------------------------------------
//  @name    PID_Init
//  @brief   PID ������ʼ��
//  @param  		    
//  @return     				
//  @since      v1.0
//  @note      
//  Sample usage:               
//-------------------------------------------------------------------------------------------------------------------
void PID_Init(PID *SPID)
{
    SPID->SumError = 0;
    SPID->LastError = 0; //Error[-1]
    SPID->PrevError = 0; //Error[-2]
    SPID->Proportion = 0; //�������� Proportional Const
    SPID->Integral = 0; //���ֳ��� Integral Const
    SPID->Derivative = 0; //΢�ֳ��� Derivative Const
}

//-------------------------------------------------------------------------------------------------------------------
//  @name    PID_Increase
//  @brief   ����ʽ PID �������
//  @param
//  @return
//  @since      v1.0
//  @note
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
int16 PID_Increase(PID *SPID,int16 NextPoint,int16 NowData)
{
    register int iError,iIncpid;                        									//��ǰ���
    iError = NextPoint - NowData;                  										//��������
    iIncpid = (int16)((SPID->Proportion + SPID->Integral + SPID->Derivative) * iError		//E[k]��
            -(SPID->Proportion + SPID->Derivative * 2             ) * SPID->LastError	//E[k��1]��
            +(SPID->Derivative								 	 ) * SPID->PrevError);	//E[k��2]��
    SPID->PrevError = SPID->LastError;                    								//�洢�������´μ���
    SPID->LastError = iError;
    return(iIncpid);   																	//��������ֵ
}

//-------------------------------------------------------------------------------------------------------------------
//  @name    PID_Realize
//  @brief   λ��ʽ PID �������
//  @param
//  @return
//  @since      v1.0
//  @note
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
int16 PID_Realize(PID *SPID,int16 NextPoint,int16 NowData)
{
    register int iError,dError,place,a;
    iError = NextPoint - NowData;				//ƫ��
    SPID->SumError += iError; 					//����

    SPID->SumError = RANGE32(SPID->SumError,-ColumnMax*10*2/2,ColumnMax*10*2/2);
    //�����޷�����ֹ���ֹ����ͣ���Ӧ�ٶ�

    dError = iError - SPID->LastError; 			//΢��
    SPID->LastError = iError;

    //��ϵͳ���ϴ�ʱ��ȡ�����ֻ��ڣ�������Сʱ��������ֻ���
    //�����Ȳ�Ӱ��������Ķ�̬���ܣ��ֿ�����߿���������̬����
    //ÿһ�εĵ�ǰ������460/2���������ķ�֮һ������ʱ�������л���
    if (iError > ColumnMax*10/4 || iError < -ColumnMax*10/4)
        a = 0;
    else 
        a = 1;

    place = (int16)(SPID->Proportion * iError 	//������
        + SPID->Integral * SPID->SumError *a 		//������
        + SPID->Derivative * dError); 			//΢����
    return(place);
}

