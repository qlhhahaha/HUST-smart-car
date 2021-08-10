
#ifndef CONTROL_H_
#define CONTROL_H_

#include "pid.h"
#include "motor.h"

extern PID ServoPID;
///extern PID LeftMotorPID;
extern PID RightMotorPID;
///extern PID MagneticPID;

extern const GearSet GearCtl;
extern ControlStruct SpeedParm,SelectMode;
extern int32 Distance,TimeCount;

extern u8 PIDWayFlag;
extern u8 ExitFlag;
///extern u8 MagneticFlag;
extern u8 RoadBlockFlag;
extern u8 RoadPassDirFlag;
extern u8 LoseRoadFlag;
extern u8 RampFlag;					//ÆÂµÀ±êÖ¾
extern u8 CircleDirFlag;
extern float Ramp_Distance;
extern float Block_Angle;
extern float Block_Distance;

void ADCRUN(void);
void Running(int mode);
void OledModeSelect(void);
void RunInitalize(void);
void ExitRunning(void);
void RunningHandle(void);
void Upload_Signal(int Channel_A,int Channel_B,int Channel_C,int Channel_D);

void PITOnTimeHandle(void);
void DMAOnTimeHandle(void);
void LED_TEST(void);

#endif 
