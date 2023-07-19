#ifndef _Date_analyse_h
#define _Date_analyse_h



#include "headfile.h"
#include "misc.h"
#include "common.h"
#include "MK60_port.h"
#include "MK60_gpio.h"
#include "MK60_port_cfg.h"


/*****全局变量的声明***************/
//以下变量大多在main函数中定义，为了正在其他.c中应用在此进行全局声明
//Date_analyse.h中方向环相关变量的声明
extern float Turn_error;
extern float Turn_error_later;
extern uint16 Dir_Kp;
extern uint16 Dir_Kd;
extern float	RightAngle_Dir_Kp;
extern int Spe_gears;
extern uint16 SpeedPID_Set_max;
extern uint16 SpeedPID_Set_min;
extern float Spe_Kp; //速度环比例系数
extern float Spe_Ki; //速度环积分系数
extern float Spe_Kd; //速度环微分系数
extern float SpeedPID_Set;  //速度设定值

			
/***********************************/


void EMCS_init(void);//电磁传感器初始化
void SC_black_Init(void);// 最大值采样
void Read_ADC(void);//读取电感值
void Date_analyse(void);
void testt(void);//测试用函数
void ultrasonic(void) ;  
void Speed_control(float *Speed_Set,float *Spe_Kp,float *Spe_Ki,float *Spe_Kd); //速度环


void direction_control(float *Turn_error,uint16 *Dir_Kp,uint16 *Dir_Kd);//


#endif
