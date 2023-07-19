#ifndef _CONTROL_H_
#define _CONTROL_H_

#include "include.h"
#include "common.h"

/**********control.c中全局变量的声明************/
extern uint8 K;

/************control.c中自定义函数的声明*************/
void pwm_init();
void PWM_OUT();
void Car_Run();
//KBI外部中断初始化,设置速度
void KBI_init();
//IRQ外部中断初始化,启动与停车检测
void IRQ_init();
//定时器初始化
void PIT0_init();


#endif