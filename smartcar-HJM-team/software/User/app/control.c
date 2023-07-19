/*!
 * @file       control.c
 * @brief      小车控制平台总程序
 * @author     胡明栋
 * @application   恩智浦杯三轮电磁组
 * @date       2018.7
 */

#include "control.h"
#include "direction.h"
#include "speed.h"


uint8 K = 0 ;    //启动与停车标志

//uint8 TimeCount = 0;
int32 right_pwm_out = 0 ;
int32 left_pwm_out = 0 ;
int32 LeftDead = 0;
int32 RighDead = 0;

 
//PWM初始化
void pwm_init()      
{
  ftm_pwm_init (FTM2, FTM_CH2, 14 * 1000, 0);             //C2  
  ftm_pwm_init (FTM2, FTM_CH3, 14 * 1000, 0 );              //C3   // 左电机   
  ftm_pwm_init (FTM2, FTM_CH4, 14 * 1000, 0 );         //G6                
  ftm_pwm_init (FTM2, FTM_CH5, 14 * 1000, 0 );           //G7   //   右电机 
}

//PWM输出
void PWM_OUT()
{
  int32 nLeftPWM = 0, nRighPWM = 0;
  
  right_pwm_out = (int32)(Speed_control_out + Turn_control_out);
  left_pwm_out = (int32)(Speed_control_out - Turn_control_out);
  
  printf(" right_pwm_out =%d\n ",right_pwm_out);
  printf(" left_pwm_out =%d\n ",left_pwm_out);
  
  if(left_pwm_out <= 0)
  {
		nLeftPWM = LeftDead - left_pwm_out;
		nLeftPWM = (nLeftPWM > 75? 75: nLeftPWM);
		ftm_pwm_duty(FTM2, FTM_CH2, nLeftPWM);    //C2   zuo
    ftm_pwm_duty(FTM2, FTM_CH3, 0);  
  }
  else
  {
		nLeftPWM = LeftDead + left_pwm_out;
		nLeftPWM = (nLeftPWM > 75? 75: nLeftPWM);
		ftm_pwm_duty(FTM2, FTM_CH2, 0);           //C2   zuo
    ftm_pwm_duty(FTM2, FTM_CH3, nLeftPWM);         
  }

  if(right_pwm_out <= 0)
  {
		nRighPWM = RighDead - right_pwm_out;
		nRighPWM = (nRighPWM > 75? 75: nRighPWM);
		ftm_pwm_duty(FTM2, FTM_CH4, 0 );         //G6  you 
    ftm_pwm_duty(FTM2, FTM_CH5, nRighPWM);
  }
  else
  {
		nRighPWM = RighDead + right_pwm_out;
		nRighPWM = (nRighPWM > 75? 75: nRighPWM);
		ftm_pwm_duty(FTM2, FTM_CH4, nRighPWM );     //G6  you 
    ftm_pwm_duty(FTM2, FTM_CH5, 0);
  }	
}

//小车运行
void Car_Run()
{
  static uint8 TurnControlCount = 0;
  static uint8 SpeedControlCount = 0;
  static uint8 MotorControlCount = 0;
   
  TurnControlCount++;
  SpeedControlCount++;
  MotorControlCount++;
  
  if(MotorControlCount == 1)  //大约1ms 进入一次
  {
    PWM_OUT();
    MotorControlCount = 0;
  }
  else if(TurnControlCount == 2) //大约2ms 进入一次
  {
    Read_ADC();  //获取电磁信号
    Direction_control();
     
    if( g_ValueOfAD[0]<=0 && g_ValueOfAD[1]<=0 && g_ValueOfAD[2]<=0 ) //冲出赛道处理
    {
      K = 1;
      ftm_pwm_duty(FTM2, FTM_CH2, 0);   
      ftm_pwm_duty(FTM2, FTM_CH3, 0  );  
      ftm_pwm_duty(FTM2, FTM_CH4, 0 );   
      ftm_pwm_duty(FTM2, FTM_CH5, 0 );
      DELAY_MS(10);
    }
    TurnControlCount = 0;
  }
  else if(SpeedControlCount == 20) //大约20ms 进入一次
  {
    Car_speed_get();
    Speed_control();
    SpeedControlCount = 0;
  }
  
   
#if 0
   static uint8 speedCONTROL_count = 0 ;
   static uint8 TimeCount = 0;
   
    if( TimeCount > 4 )
    {
      TimeCount=0;
    }
    
    else if(TimeCount ==1)
    {
         PWM_OUT();
    }
    else if(TimeCount ==2)
    {
      
       Read_ADC();  //获取电磁信号
       Direction_control();
      
     
       if( g_ValueOfAD[0]<=0 && g_ValueOfAD[1]<=0 && g_ValueOfAD[2]<=0 ) //冲出赛道处理
       {
           K = 1;
          ftm_pwm_duty(FTM2, FTM_CH2, 0);   
          ftm_pwm_duty(FTM2, FTM_CH3, 0  );  
          ftm_pwm_duty(FTM2, FTM_CH4, 0 );   
          ftm_pwm_duty(FTM2, FTM_CH5, 0 );
          DELAY_MS(10);
       }
      
      
    }      
    else if(TimeCount ==4)
    {
      
      if( speedCONTROL_count >=5 )    //20ms一次
      {
         Car_speed_get();
         Speed_control();
         speedCONTROL_count = 0;
         
      }
       speedCONTROL_count++;
      
    }
    TimeCount++ ;
#endif
    
}


//KBI外部中断初始化,设置速度
void KBI_init()
{
   kbi_init (PTH1, KBI_PULLUP_EN | KBI_FALLING) ;     //PTH1下降沿触发
    
   DELAY_MS(1);        //以便上拉使得PTE4 保持高电平
    
   KBI1_CLEAN_FLAG();
   
   set_irq_priority(KBI1_IRQn,1) ;     //设置中断优先级为1
   
   kbi_enable_irq(PTH1);
   
}


//IRQ外部中断初始化,启动与停车检测
void IRQ_init()
{
  irq_init (IRQ_PIN, IRQ_PULLUP_EN | IRQ_FALLING ) ;      //配置PTI0为下降沿中断

  enable_irq(IRQ_IRQn);

}

//定时器初始化
void PIT0_init()
{
   pit_init_ms(PIT0, 1);                               // 初始化PIT0, 定时 1ms
   enable_irq(PIT_CH0_IRQn);                             // 使能PIT_CH0中断
  
}










