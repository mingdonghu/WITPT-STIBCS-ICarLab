#ifndef _SPEED_H_
#define _SPEED_H_

#include "include.h"
#include "common.h"

/************速度PID参数用户设定*****************/  
#define Spe_Kp             0.1
#define Spe_Ki             0 
#define Spe_Kd             0

//速度PI控制
void Speed_control();
//编码器测速初始化
void coderPWM_Pulseinit();
//速度获取
void Car_speed_get();
//速度偏差计算
void  Car_Speed_error();


extern float Speed_Set ;  
extern float Speed_control_out ;
extern uint32 getRight_motor_speed,getLeft_motor_speed;
 


#endif