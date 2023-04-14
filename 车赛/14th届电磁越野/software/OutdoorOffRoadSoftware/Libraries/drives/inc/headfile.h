/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2017,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：179029047
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		headfile
 * @company	   		成都逐飞科技有限公司
 * @author     		Go For It(1325536866)
 * @version    		v2.0
 * @Software 		IAR 7.7 or MDK 5.17
 * @Target core		MK60DN and MK60FX
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2017-09-19
 ********************************************************************************************************************/



#ifndef _headfile_h
#define _headfile_h
//按键宏定义
	
	#define key24    gpio_get(A24)	   //读取A24  IO口状态 	
	#define key25    gpio_get(A25)	   //读取A24  IO口状态 	
	#define key27    gpio_get(A27)	   //读取A24  IO口状态 	
	#define key29    gpio_get(A29)	   //读取A24  IO口状态 

#define key1    gpio_get(B23)	   //读取A24  IO口状态 
#define	key2    gpio_get(C1)	  //读取   IO口状态 
#define	key3    gpio_get(C4)	  //读取IO口状态 
#define	key4    gpio_get(C3)   //读取IO口状态 
#define spe_integral_limit 950 //速度环积分限幅\

#define spe_output_limit 999 //速度环积分限幅
#define Barricades_distance_limit 20 //距离判定限定！当前方有障碍触发横断路障专用中断
#define ultrasonic_distance_limit 99 //距离判定限定！当前方有障碍时强制减速，优先级最高	
#define ultrasonic_distance_spe 280 //速度限定 ！当前方有障碍时强制减速，优先级最高	
#define SpeedPID_Set_14_define 18000
#define SpeedPID_Set_13_define 17000
#define SpeedPID_Set_12_define 16000
#define SpeedPID_Set_11_define 15000
#define SpeedPID_Set_10_define 14000
#define SpeedPID_Set_9_define 13000
#define SpeedPID_Set_8_define 12000
#define SpeedPID_Set_7_define 11000
#define SpeedPID_Set_6_define 10000
#define SpeedPID_Set_5_define 9000
#define SpeedPID_Set_4_define 8000
#define SpeedPID_Set_3_define 7000
#define SpeedPID_Set_2_define 6000
#define SpeedPID_Set_1_define 5000
#define SpeedPID_Set_0_define 4000


#define Direction_Median  1400 //舵机中值
#define EMCS_MAX  750  //固定每个电感的最大值
#define AD_Area_limit_1   19 //分区判断时两侧电感最小值 在 Read_ADC();使用
#define AD_Area_limit_2   21 //分区判断时两侧电感最小值 在 Read_ADC();使用
#define Turn_error_max 420//方向环积分限幅


#define AD_crossroad_limit 70//十字路口识别AD幅值限定

#define AD_roundabout_limit  70 //环岛识别AD幅值限定、

#define AD_RightAngle_limit_min 40   //直角识别AD幅值限定、
#define AD_RightAngle_limit_max 60 //直角识别AD幅值限定、
#define AD_1_multiple 0.9   //2号电感放大倍数

#define AD_judge_limit_min 	30 //是否进行区域判定限定值
#define AD_judge_limit_max  99 //是否进行区域判定限定值
#define roundabout_Count_delay_1  0   //环岛第一次判定延时计数置零，在650次 后将所改动系数还原为初始值
 #define roundabout_Count_delay_2   550 //环岛第二次判定延时计数置零，在100次 后将所改动系数还原为初始值
#define Dir_Kp_Turn_error_1_57	1.1  //5和7号电感误差倍数
#define Dir_Kp_Turn_error_1_04	1  //0和4号电感误差倍数
#define Dir_Kp_Turn_error_1_13	1.4   //1和3号电感误差倍数
//#define RightAngle_Dir_Kp  2.5  //K值放大



#define pit_time_get_limit 900000 //超声波PIT中断计时约束条件
#include "MK60_port_cfg.h"
#include "common.h"
#include "misc.h"
#include "stdio.h"

//--------函数库--------
#include "MK60_rtc.h"
#include "MK60_gpio.h"
#include "MK60_systick.h"
#include "MK60_port.h"
#include "MK60_uart.h"
#include "MK60_pit.h"
#include "MK60_lptmr.h"
#include "MK60_ftm.h"
#include "MK60_adc.h"
#include "MK60_dac.h"
#include "MK60_flash.h"
#include "MK60_spi.h"
#include "MK60_i2c.h"
#include "MK60_wdog.h"
#include "MK60_dma.h"
#include "MK60_cmt.h"
#include "MK60_sdhc.h"
//自行添加库2019年4月27日
#include "Date_analyse.h"

#include "isr.h"

//fatfs文件系统
#include "ff.h"

//--------逐飞科技产品例程库--------
#include "SEEKFREE_MT9V032.h"
#include "SEEKFREE_18TFT.h"
#include "SEEKFREE_FUN.h"
#include "SEEKFREE_TSL1401.h"
#include "SEEKFREE_7725.h"
#include "SEEKFREE_IIC.h"
#include "SEEKFREE_NRF24L01.h"
#include "SEEKFREE_OLED.h"
#include "SEEKFREE_L3G4200D.h"
#include "SEEKFREE_MMA8451.h"
#include "SEEKFREE_MPU6050.h"

void read_ad_test(void);//读取未经处理过的值调试采用
 void  keycan(void);
#endif
