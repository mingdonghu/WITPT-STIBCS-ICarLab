/*!
 * @file       main.c
 * @brief      山外KEA 平台主程序
 * @author     胡明栋
 * @application   恩智浦杯三轮电磁组
 * @date       2018.7
 */

#include "common.h"
#include "include.h"
#include "control.h"
#include "direction.h"
#include "speed.h"

uint8 flag = 0;

//主函数
void main(void)
{
    DisableInterrupts;            //禁止全部中断
    
  /*********************************************************
  *     初始化程序
  **********************************************************/
   PIT0_init();         //定时器初始化
  
   KBI_init();    //KBI外部中断初始化,设置速度
   
   IRQ_init();    //IRQ外部中断初始化,启动与停车检测
   
  // uart_init (VCAN_PORT, VCAN_BAUD);   //初始化串口波特率为115200 ,蓝牙串口
  // uart_rx_irq_en(VCAN_PORT);    //使能串口中断
     
   pwm_init();      //PWM初始化
  
   coderPWM_Pulseinit();        //编码器测速初始化
  
   EMC_init();                //电磁传感器初始化
   
   OLED_Init();         //OLED显示初始化化
   OLED_Fill(0,0,127,63,0);//填充
   //显示队名
   OLED_ShowString(0,0,"HJM SmartCarTeam" ,16);//显示字符串    X:列  Y：行
   OLED_ShowString(0,25,"SetSpeedValue:" ,12);//显示字符串
   
   Speed_Set = 530;     //初始化设置车子速度
   OLED_ShowNum(100,25,(uint8)(Speed_Set/10),2,12);
   OLED_Refresh_Gram();//更新显存到OLED 
     
   gpio_init(PTG2, GPO,0);      
   DELAY_MS(500);
   gpio_set (PTG2, 1);      //LED2闪烁一下，已完成初始化
   
   EnableInterrupts;        //打开全部中断
   
  /*********************************************************
  *     执行程序
  **********************************************************/     
  while(1)
  {
     
  }
    
}

/*********一些中断服务函数的定义 ***************/
//PT0中断服务函数
void pit_ch0_irq()
{    
     
#if 1     
     if( !K )        //K=0,启动小车
     {
         Car_Run();
     }
     PIT_Flag_Clear(PIT0);       //清中断标志位
     
#endif    
     
    
}

//KBI1中断服务函数     //调速
void  kbi1_irq(void)
{
  static uint8 count = 0;
    
    if(IS_KBI1_IRQ())       //判断是否有效的KBI中断
    {
        if(IS_KBI_CH_IRQ(PTH1)) //判断是否PTH1触发的中断
        {
            count++;
        }
    }
    
    if(count == 1)     //按第一次，设定速度1挡
    {
       Speed_Set = 470;
       OLED_ShowNum(100,25,(uint8)(Speed_Set/10),2,12);
       OLED_Refresh_Gram();//更新显存到OLED
         
    }else if(count == 2)    //按第二次，设定速度2挡
      {
          Speed_Set = 560;
          OLED_ShowNum(100,25,(uint8)(Speed_Set/10),2,12);
          OLED_Refresh_Gram();//更新显存到OLED
       
      }else if(count == 3)  //按第三次，设定速度空挡，档位归零
        {
           count = 0; 
        }
    KBI1_CLEAN_FLAG() ;
    
}

//IRQ中断服务函数      //启动与停车检测
void irq_irq()
{
  
   if(IS_IRQ_FLAG())
    {
       flag++;
    }
   if(flag==2)      //第一次检测到起跑线，启动
   {
       K = 0;
       DELAY_MS(20);
     
   }
   else if(flag==3)   //第二次检测到起跑线，停车
   {
      flag = 0 ;
      K = 1;
      ftm_pwm_duty(FTM2, FTM_CH2, 0);   //C2   zuo   
      ftm_pwm_duty(FTM2, FTM_CH3, 0  );  
      ftm_pwm_duty(FTM2, FTM_CH4, 0 );   //G6  you 
      ftm_pwm_duty(FTM2, FTM_CH5, 0 );
      DELAY_MS(5);
      
   }
    IRQ_CLEAN_FLAG();
}

void uart0_irq()
{
    char ch;
    uart_getchar   (VCAN_PORT, &ch);   
    if(uart_querychar (VCAN_PORT, &ch) == 1)                        //查询接收1个字符
    {
       printf("成功接收到一个字节");
    }
}









