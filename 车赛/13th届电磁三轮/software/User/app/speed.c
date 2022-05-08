/*!
 * @file       speed.c
 * @brief      小车速度传感器及控制平台程序
 * @author     胡明栋
 * @application   恩智浦杯三轮电磁组
 * @date       2018.7
 */

#include "speed.h"
#include "control.h"
#include "direction.h"

 float Speed_Set = 0 ;  
 float Speed_control_out = 0;
uint32 getRight_motor_speed=0,getLeft_motor_speed=0;
 float Speed_error = 0;
 float Feedback_Speed = 0;

//速度PI控制
void Speed_control()
{
//增量式PID
  static float Speed_error_last = 0 ;
  static float Speed_error_last_last = 0 ;
  static float  Speed_control_out_Last = 0 ;	   
  float Spe_nP = 0;
  float Spe_nI = 0;
  float Spe_nD = 0;
  
  Car_Speed_error();  //计算速度偏差
  
  Spe_nP = Spe_Kp * (Speed_error- Speed_error_last);
  
  printf("Speed_error_last=%f\n",Speed_error_last);
  
  Spe_nI = Spe_Ki * Speed_error;
 
  Spe_nD = Spe_Kd * (  Speed_error - 2 * Speed_error_last + Speed_error_last_last );
  
  Speed_error_last_last = Speed_error_last ;
  Speed_error_last = Speed_error ;
  
  /*****速度环积分限幅***********************/
  Spe_nI = ( Spe_nI > 10000 ? 10000 : Spe_nI ) ;
  Spe_nI = ( Spe_nI < -10000 ? -10000 : Spe_nI ) ;
  /**********速度环输出************************/
  Speed_control_out = Spe_nP + Spe_nI + Spe_nD ;
  /**************************************************/
  Speed_control_out += Speed_control_out_Last ;
  Speed_control_out_Last = Speed_control_out ;
  
  /***************速度环输出限幅*************/
  Speed_control_out = ( Speed_control_out > 70000 ? 70000 : Speed_control_out ); // PWM duty 扩大1000倍
  Speed_control_out = ( Speed_control_out < -70000 ? -70000 : Speed_control_out );
  
 
  printf("Speed_control_out=%f\n",Speed_control_out);
  
}

//编码器测速初始化
void coderPWM_Pulseinit()        
{
     ftm_pulse_init (FTM0,FTM_PS_1,TCLK1);     //左编码器脉冲计数输入  //E0 
     
     gpio_init (PTA0, GPI,0);     //左编码器方向输入口    //A0
     port_pull (PTA0, ENABLE);    //IO上拉 
     
     ftm_pulse_init (FTM1,FTM_PS_1,TCLK2);    //右编码器脉冲计数输入   //E7 
     
     gpio_init (PTA1, GPI,0);     //右编码器方向输入口    //A1
     port_pull (PTA1, ENABLE);    //IO上拉
}


//速度获取
void Car_speed_get()
{
  //获取FTM 的脉冲数
    getLeft_motor_speed  = ftm_pulse_get(FTM0);
    getRight_motor_speed = ftm_pulse_get(FTM1);
   //判断脉冲正负 
    getLeft_motor_speed = (  gpio_get(PTA0) == 1 ? getLeft_motor_speed : - getLeft_motor_speed );
    getRight_motor_speed = (  gpio_get(PTA1) == 0 ? getRight_motor_speed : - getRight_motor_speed );
     //清除FTM 的脉冲数
    ftm_pulse_clean(FTM0);
    ftm_pulse_clean(FTM1);
    
    printf("左电机:%d", getLeft_motor_speed );
    printf("      右电机:%d\n", getRight_motor_speed );
    
}


//速度偏差计算
void  Car_Speed_error()
{
  
  Feedback_Speed = ( getLeft_motor_speed + getRight_motor_speed ) >> 2 ; 
  
  Speed_error = Speed_Set - Feedback_Speed ; 
  
  printf("Speed_error:%f\n",Speed_error) ;
  
}



