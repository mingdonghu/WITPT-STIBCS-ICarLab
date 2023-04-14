/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2016,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：179029047
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		isr.c
 * @brief      		中断函数库
 * @company	   		成都逐飞科技有限公司
 * @author     		Go For It(1325536866)
 * @version    		v1.0
 * @Software 		IAR 7.7 or MDK 5.17
 * @Target core		MK60DN512VLL10
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2016-02-25
 ********************************************************************************************************************/



#include "isr.h"

extern uint8 TIME1flag_1ms   ;     //定时器时间中断标志位
//extern uint8 TIME1flag_10ms  ;
extern uint8 TIME1flag_20ms  ;
//extern uint8 TIME1flag_50ms  ;
//extern uint8 TIME1flag_80ms  ;
//extern uint8 TIME1flag_100ms ;



//-------------------------------------------------------------------------------------------------------------------
//  @brief      PROTA中断执行函数
//  @return     void   
//  @since      v1.0
//  Sample usage:               当A口启用中断功能且发生中断的时候会自动执行该函数
//-------------------------------------------------------------------------------------------------------------------
void PORTA_IRQHandler(void)
{
    //清除中断标志第一种方法直接操作寄存器，每一位对应一个引脚
	PORTA->ISFR = 0xffffffff;
	//使用我们编写的宏定义清除发生中断的引脚
	//PORTA_FLAG_CLR(A1);

}


void PORTC_IRQHandler(void)
{
    //清除中断标志第一种方法直接操作寄存器，每一位对应一个引脚
	PORTC->ISFR = 0xffffffff;
	//使用我们编写的宏定义清除发生中断的引脚
//		ftm_pwm_duty(ftm1,ftm_ch1,Direction_Median);  
//		ftm_quad_clean(ftm2);//编码器计数清零
//		ftm_pwm_duty(ftm0,ftm_ch7,600); 
//		ftm_pwm_duty(ftm0,ftm_ch5,0); 	
//				while(ftm_quad_get(ftm2) < 16000);
//	
//			ftm_pwm_duty(ftm0,ftm_ch7,0); 
//		ftm_pwm_duty(ftm0,ftm_ch5,180); 	
//		systick_delay_ms(1000);
//		ftm_pwm_duty(ftm0,ftm_ch7,0); 
//		ftm_pwm_duty(ftm0,ftm_ch5,0);
//		
//			ftm_pwm_duty(ftm1,ftm_ch1,Direction_Median);  
	keycan();
	PORTC_FLAG_CLR(C3);
		PORTC_FLAG_CLR(C4);
//    VSYNC();
}


void DMA0_IRQHandler(void)
{
	DMA_IRQ_CLEAN(DMA_CH0);
    row_finished();
	
}

void PIT0_IRQHandler(void)
{
	PIT_FlAG_CLR(pit0);

}


void PIT2_IRQHandler(void)
{
	PIT_FlAG_CLR(pit2);

	
	
	
		printf("pit2\r\n");

if(0)//不停车过路障
{

//				ftm_quad_clean(ftm2);//编码器计数清零


////		printf(" ftm_quad_get(ftm2)=%d\r\n",(uint16)ftm_quad_get(ftm2));
//			ftm_pwm_duty(ftm1,ftm_ch1,Direction_Median); 
//		ftm_pwm_duty(ftm0,ftm_ch7,450); 
//		ftm_pwm_duty(ftm0,ftm_ch5,0); 	
//				while(ftm_quad_get(ftm2) < 15000);
			ftm_quad_clean(ftm2);//编码器计数清零
			
		//右打 ，前进
		ftm_pwm_duty(ftm1,ftm_ch1,Direction_Median+Turn_error_max);  
		ftm_pwm_duty(ftm0,ftm_ch7,450); 
		ftm_pwm_duty(ftm0,ftm_ch5,0); 
		while(ftm_quad_get(ftm2) < 9000);
		printf(" youzhuan=%d\r\n",ftm_quad_get(ftm2));
		ftm_quad_clean(ftm2);//编码器计数清零
	
		//方向摆正直走
		ftm_pwm_duty(ftm1,ftm_ch1,Direction_Median);  
		ftm_pwm_duty(ftm0,ftm_ch7,450); 
		ftm_pwm_duty(ftm0,ftm_ch5,0); 
		while(ftm_quad_get(ftm2) < 8000);
				printf(" zhizou=%d\r\n",ftm_quad_get(ftm2));
		ftm_quad_clean(ftm2);//编码器计数清零

		
		//直走后左转
		ftm_pwm_duty(ftm1,ftm_ch1,Direction_Median-Turn_error_max);  
		while(ftm_quad_get(ftm2) < 22000);
			printf(" zuozhuan=%d\r\n",ftm_quad_get(ftm2));
		ftm_quad_clean(ftm2);//编码器计数清零
	
		
		
		//前进时 ，刹车
			ftm_pwm_duty(ftm0,ftm_ch7,0); 
		ftm_pwm_duty(ftm0,ftm_ch5,160); 	
		systick_delay_ms(200);
		ftm_pwm_duty(ftm0,ftm_ch7,0); 
		ftm_pwm_duty(ftm0,ftm_ch5,0);
		
			ftm_pwm_duty(ftm1,ftm_ch1,Direction_Median);  


}


if(1)//停车过路障
{

				ftm_quad_clean(ftm2);//编码器计数清零

////		printf(" ftm_quad_get(ftm2)=%d\r\n",(uint16)ftm_quad_get(ftm2));
//			ftm_pwm_duty(ftm1,ftm_ch1,Direction_Median); 
//		ftm_pwm_duty(ftm0,ftm_ch7,650); 
//		ftm_pwm_duty(ftm0,ftm_ch5,0); 	
//				while(ftm_quad_get(ftm2) < 20000);
				
		
		//前进时刹车刹车
		ftm_pwm_duty(ftm0,ftm_ch7,0); 
		ftm_pwm_duty(ftm0,ftm_ch5,600); 	
			systick_delay_ms(50);
		ftm_pwm_duty(ftm0,ftm_ch7,0); 
		ftm_pwm_duty(ftm0,ftm_ch5,0); 	
			systick_delay_ms(50);
			ftm_quad_clean(ftm2);//编码器计数清零
			
			
		//左打 ，后退
		ftm_pwm_duty(ftm1,ftm_ch1,Direction_Median-Turn_error_max);  
		ftm_pwm_duty(ftm0,ftm_ch7,0); 
		ftm_pwm_duty(ftm0,ftm_ch5,999); 
		while(ftm_quad_get(ftm2) > -6600);
				ftm_quad_clean(ftm2);//编码器计数清零
		printf(" ftm2_-7000=%d\r\n",ftm_quad_get(ftm2));
		
		
		//后退时，刹车
		ftm_pwm_duty(ftm0,ftm_ch7,200); 
		ftm_pwm_duty(ftm0,ftm_ch5,0); 	
		systick_delay_ms(200);
		ftm_pwm_duty(ftm0,ftm_ch7,0); 
		ftm_pwm_duty(ftm0,ftm_ch5,0);
		
		
				//方向摆正右转
		ftm_pwm_duty(ftm1,ftm_ch1,Direction_Median+Turn_error_max);  
		ftm_pwm_duty(ftm0,ftm_ch7,999); 
		ftm_pwm_duty(ftm0,ftm_ch5,0); 
		while(ftm_quad_get(ftm2) < 100);
				ftm_quad_clean(ftm2);//编码器计数清零
		printf(" ftm2_6000=%d\r\n",ftm_quad_get(ftm2));
		
		
		//方向摆正直走
		ftm_pwm_duty(ftm1,ftm_ch1,Direction_Median);  
		ftm_pwm_duty(ftm0,ftm_ch7,999); 
		ftm_pwm_duty(ftm0,ftm_ch5,0); 
		while(ftm_quad_get(ftm2) < 900);
				ftm_quad_clean(ftm2);//编码器计数清零
		printf(" ftm2_6000=%d\r\n",ftm_quad_get(ftm2));
		
		//直走后左转
		ftm_pwm_duty(ftm1,ftm_ch1,Direction_Median-Turn_error_max);  
		while(ftm_quad_get(ftm2) < 26000);
				ftm_quad_clean(ftm2);//编码器计数清零
		printf(" ftm2_25000=%d\r\n",ftm_quad_get(ftm2));
		
		
		//前进时 ，刹车
//			ftm_pwm_duty(ftm0,ftm_ch7,0); 
//		ftm_pwm_duty(ftm0,ftm_ch5,160); 	
//		systick_delay_ms(200);
//		ftm_pwm_duty(ftm0,ftm_ch7,0); 
//		ftm_pwm_duty(ftm0,ftm_ch5,0);
//		
			ftm_pwm_duty(ftm1,ftm_ch1,Direction_Median);  


}
//	ftm_pwm_duty(ftm1,ftm_ch1,Direction_Median-Turn_error_max);  
	
	
	
//	  //舵机占空比设置	
//		ftm_pwm_duty(ftm1,ftm_ch1,Direction_Median+Turn_error_max);  
//	 systick_delay_ms(400);   //延时1000毫秒
//	//D7   //电机正
//	
//			ftm_pwm_duty(ftm1,ftm_ch1,Direction_Median);  
//	 systick_delay_ms(600);   //延时1000毫秒



//			ftm_pwm_duty(ftm1,ftm_ch1,Direction_Median-Turn_error_max);  
//		 systick_delay_ms(1220);   //延时1000毫秒
//	
//	
//	
	
	
	


disable_irq(PIT2_IRQn);
}


void PIT1_IRQHandler(void)    //PIT1中断用超声波测距模块 产生 触发脉冲信号     1ms进入一次中断
{
     static uint8  TimeCountTurn = 0 ;     //方向环调节周期
	 static uint8  TimeCountSpeed = 0 ;   //速度环调节周期
	 static uint8  TimeCountWave = 0 ;    //超声波测距信号周期
	
	 PIT_FlAG_CLR(pit1);

   TimeCountTurn++;
	 TimeCountSpeed++;
	 TimeCountWave++;	
	
	
	
	
	
	
	
    //电感和舵机处理
			Read_ADC();//读取各个电感值
			direction_control(&Turn_error,&Dir_Kp,&Dir_Kd);
	//速度环
 if(TimeCountSpeed >= 20 )
   {
		 TimeCountSpeed = 0 ;
			 Speed_control(&SpeedPID_Set,&Spe_Kp,&Spe_Ki,&Spe_Kd);
			/*
      Feedback_Speed = ftm_quad_get(ftm2);              //获取电机速度
      ftm_quad_clean(ftm2);
			printf("ftm_quad_get(ftm2)=%d\r\n",Feedback_Speed);
			*/
   }
if(key27)
	 
 if(TimeCountWave >= 5)
	 {

		 TimeCountWave = 0 ;
		 
		 ultrasonic();


	 }
 }


//2019-07-16 00:13:39备份
//void PIT1_IRQHandler(void)    //PIT1中断用超声波测距模块 产生 触发脉冲信号     1ms进入一次中断
//{
//     static uint8  TimeCountTurn = 0 ;     //方向环调节周期
//	 static uint8  TimeCountSpeed = 0 ;   //速度环调节周期
//	 static uint8  TimeCountWave = 0 ;    //超声波测距信号周期
//	
//	 PIT_FlAG_CLR(pit1);

//   TimeCountTurn++;
//	 TimeCountSpeed++;
//	 TimeCountWave++;	
//	
//	
//	
//	
//	
//	
//	
//    //电感和舵机处理
//			Read_ADC();//读取各个电感值
//			direction_control(&Turn_error,&Dir_Kp,&Dir_Kd);
//	//速度环
// if(TimeCountSpeed >= 20 )
//   {
//		 TimeCountSpeed = 0 ;
//			 Speed_control(&SpeedPID_Set,&Spe_Kp,&Spe_Ki,&Spe_Kd);
//			/*
//      Feedback_Speed = ftm_quad_get(ftm2);              //获取电机速度
//      ftm_quad_clean(ftm2);
//			printf("ftm_quad_get(ftm2)=%d\r\n",Feedback_Speed);
//			*/
//   }

//	 
// if(TimeCountWave >= 5)
//	 {

//		 TimeCountWave = 0 ;
//		 
//		 ultrasonic();


//	 }

//}



//-------------------------------------------------------------------------------------------------------------------
//  @brief      UART3中断执行函数
//  @return     void   
//  @since      v1.0
//  Sample usage:               当UART3启用中断功能且发生中断的时候会自动执行该函数
//-------------------------------------------------------------------------------------------------------------------
void UART3_RX_TX_IRQHandler(void)
{
    if(UART3->S1 & UART_S1_RDRF_MASK)                                     //接收数据寄存器满
    {
        //用户需要处理接收数据
        mt9v032_cof_uart_interrupt();
    }
    if(UART3->S1 & UART_S1_TDRE_MASK )                                    //发送数据寄存器空
    {
        //用户需要处理发送数据

    }
}


/*
中断函数名称，用于设置对应功能的中断函数
Sample usage:当前启用了DMA0中断，然后就到下面去找哪个是DMA0的中断函数名称，找到后写一个该名称的函数即可
void DMA0_IRQHandler(void)
{
    ;
}
记得进入中断后清除标志位


DMA0_IRQHandler  
DMA1_IRQHandler  
DMA2_IRQHandler  
DMA3_IRQHandler  
DMA4_IRQHandler  
DMA5_IRQHandler  
DMA6_IRQHandler  
DMA7_IRQHandler  
DMA8_IRQHandler  
DMA9_IRQHandler  
DMA10_IRQHandler 
DMA11_IRQHandler 
DMA12_IRQHandler 
DMA13_IRQHandler 
DMA14_IRQHandler 
DMA15_IRQHandler 
DMA_Error_IRQHandler      
MCM_IRQHandler            
FTFL_IRQHandler           
Read_Collision_IRQHandler 
LVD_LVW_IRQHandler        
LLW_IRQHandler            
Watchdog_IRQHandler       
RNG_IRQHandler            
I2C0_IRQHandler           
I2C1_IRQHandler           
SPI0_IRQHandler           
SPI1_IRQHandler           
SPI2_IRQHandler           
CAN0_ORed_Message_buffer_IRQHandler    
CAN0_Bus_Off_IRQHandler                
CAN0_Error_IRQHandler                  
CAN0_Tx_Warning_IRQHandler             
CAN0_Rx_Warning_IRQHandler             
CAN0_Wake_Up_IRQHandler                
I2S0_Tx_IRQHandler                     
I2S0_Rx_IRQHandler                     
CAN1_ORed_Message_buffer_IRQHandler    
CAN1_Bus_Off_IRQHandler                
CAN1_Error_IRQHandler                  
CAN1_Tx_Warning_IRQHandler             
CAN1_Rx_Warning_IRQHandler             
CAN1_Wake_Up_IRQHandler                
Reserved59_IRQHandler                  
UART0_LON_IRQHandler                   
UART0_RX_TX_IRQHandler                 
UART0_ERR_IRQHandler                   
UART1_RX_TX_IRQHandler                 
UART1_ERR_IRQHandler  
UART2_RX_TX_IRQHandler
UART2_ERR_IRQHandler  
UART3_RX_TX_IRQHandler
UART3_ERR_IRQHandler  
UART4_RX_TX_IRQHandler
UART4_ERR_IRQHandler  
UART5_RX_TX_IRQHandler
UART5_ERR_IRQHandler  
ADC0_IRQHandler
ADC1_IRQHandler
CMP0_IRQHandler
CMP1_IRQHandler
CMP2_IRQHandler
FTM0_IRQHandler
FTM1_IRQHandler
FTM2_IRQHandler
CMT_IRQHandler 
RTC_IRQHandler 
RTC_Seconds_IRQHandler  
PIT0_IRQHandler  
PIT1_IRQHandler  
PIT2_IRQHandler  
PIT3_IRQHandler  
PDB0_IRQHandler  
USB0_IRQHandler  
USBDCD_IRQHandler
ENET_1588_Timer_IRQHandler
ENET_Transmit_IRQHandler  
ENET_Receive_IRQHandler
ENET_Error_IRQHandler  
Reserved95_IRQHandler  
SDHC_IRQHandler
DAC0_IRQHandler
DAC1_IRQHandler
TSI0_IRQHandler
MCG_IRQHandler 
LPTimer_IRQHandler 
Reserved102_IRQHandler 
PORTA_IRQHandler 
PORTB_IRQHandler 
PORTC_IRQHandler 
PORTD_IRQHandler 
PORTE_IRQHandler 
Reserved108_IRQHandler
Reserved109_IRQHandler
SWI_IRQHandler 
*/
                


