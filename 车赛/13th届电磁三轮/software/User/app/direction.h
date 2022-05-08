#ifndef  _DIRECTION_H_
#define  _DIRECTION_H_

#include "include.h"
#include "common.h"

//开方差比和Function
#define  ExtractDifferentialRatioAnd( A , B )      ( sqrt((A)) - sqrt((B)) ) / ( (A) + (B) )

#define  DifferentialRatioAnd( A , B , C )      ( (A) - (B) ) / ( (A) + (B) + (C) )

/******************方向PID参数************************/
#define Dir_Kp1         850          //直道，弯道    Speed_Set:560   
#define Dir_Kd1         795

#define Dir_Kp2          830           //直道，弯道    Speed_Set:470
#define Dir_Kd2          795      

#define Dir_Kp3          630           //直道，弯道    Speed_Set:530
#define Dir_Kd3          570      

#define Dir_Kp4          860         //直道，弯道    Speed_Set:500
#define Dir_Kd4          795  

#define Dir_Kp5         1000           //右弯道稳定处理 
#define Dir_Kd5          0

#define Dir_Kp6         1000           //左弯道稳定处理 
#define Dir_Kd6          0

//方向PD控制
void Direction_control();
//电磁传感器初始化     
void get_AD();
// (中值滤波)获取每个电感的最大值， 
void get_ADMAXvalue_init();
// (中值滤波)电感信号采集    
void Read_ADC();
//电磁传感器初始化
void EMC_init();

/*******声明全局变量*******/
extern float Turn_control_out ;
extern uint8 d ;
extern int16 g_ValueOfAD[3];		//获取的电感值      //[0] 左   [1] 右    [2] 中 
extern float Turn_error; 


#endif