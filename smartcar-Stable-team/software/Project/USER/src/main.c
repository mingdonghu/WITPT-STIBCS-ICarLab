/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2017,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：179029047
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @company	   		成都逐飞科技有限公司
 * @version    		v2.0
 * @Software 		IAR 7.7 or MDK 5.17
 * @Target core		MK60DN and MK60FX
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2017-09-19
 ********************************************************************************************************************/

#include "headfile.h"
#include "oled.h"
#include "mpu6050.h"
#include "inv_mpu.h"

			uint8  beishu=10;//比例和微分调整一次改变数值
//自定义函数的声明




void keycan(void);//案件测试

void ultrasonic(void);    //超声波测距
void  mpu6050(void);    //陀螺仪主函数
void EMCS_init(void);  //电磁传感器初始化
void read_adc(void);   //获取电感值
void read_ad_test(void);//读取未经处理过的值调试采用
int l=0 ;
	
void coderPWM_Pulseinit(void);  //编码器测速初始化
/***************************************/

/**全局变量声明**************************/

uint16 Speed_Set=0;//OLED_
float pitch,roll,yaw; 		//陀螺仪_欧拉角
short aacx,aacy,aacz;		//陀螺仪_加速度传感器原始数据
short gyrox,gyroy,gyroz;	//陀螺仪_陀螺仪原始数据
float temp;					//陀螺仪_温度
float acce_val_x,acce_val_y,acce_val_z;	 //陀螺仪_accelerometer test value

uint8 TIME1flag_1ms   ;   ////定时器时间中断标志位
//uint8 TIME1flag_10ms  ;
uint8 TIME1flag_20ms  ;
//uint8 TIME1flag_50ms  ;
//uint8 TIME1flag_80ms  ;
//uint8 TIME1flag_100ms ;

/*速度环全局变量*******************/
float Spe_Kp = 0.112; //速度环比例系数
float Spe_Ki = 0.0000; //速度环积分系数
float Spe_Kd = 0.003; //速度环微分系数
//float SpeedPID_Set = SpeedPID_Set_1;  //速度设定值被放到 void Read_ADC(void);//读取电感值
//float SpeedPID_Set = SpeedPID_Set_Media ;  //速度设定值
int Spe_gears=0;
float SpeedPID_Set = SpeedPID_Set_0_define;
uint16 SpeedPID_Set_max =SpeedPID_Set_3_define;
uint16 SpeedPID_Set_min = SpeedPID_Set_0_define;
int SpeedPID_Set_max_num = 3;
int SpeedPID_Set_min_num = 0;
/*********************************/

/*方向环全局变量*******************/
float Turn_error = 0;//总偏差
float Turn_error_later;
uint16 duoji=Direction_Median;
uint16 duty_test=Direction_Median;
uint16 Dir_Kp = 600;//方向环比例系数
uint16 Dir_Kd = 240;//方向环微分系数
float RightAngle_Dir_Kp = 0.7;//进环岛5和7电感放大比例系数(在原基础上加大)
/*********************************/

//编码器测试
//int16 Feedback_Speed = 0 ;  //反馈速度



/***********************************************************************************/	
//主函数	
int main(void)
{
  DisableInterrupts;     //关闭总的中断开关
	get_clk();//上电后必须运行一次这个函数，获取各个频率信息，便于后面各个模块的参数设置
	
	/************灯程序初始化IO口！提供测试用****************/
	gpio_init(A14,GPO,0);//测试程序是否正常循环
	gpio_init(A15,GPO,0);//测试程序是否正常循环
	gpio_init(A16,GPO,0);//测试程序是否正常循环
	//gpio_init(A17,GPO,0);//测试程序是否正常循环
	
	uart_init(uart1,115200);//初始化uart1/RX E1/TX EO/
	
/************按键初始化****************/

  gpio_init(B23,GPI,1)	;	//初始化按键 IO口1
	gpio_init(C1,GPI,1)		;	//初始化按键IO口2
	gpio_init(C3,GPI,1)		;	//初始化按键IO口4
	gpio_init(C4,GPI,1)		;	//初始化按键IO口3
//拨码开关从前到后 
		gpio_init(A24,GPI,1)		;	//初始化按键IO口3
		gpio_init(A25,GPI,1)		;	//初始化按键IO口3
		gpio_init(A27,GPI,1)		;	//初始化按键IO口3
		gpio_init(A29,GPI,1)		;	//初始化按键IO口3

	
/*******************OLED TEST******************/
	 OLED_Init();         //OLED显示初始化化
   OLED_Fill_s(0,0,127,63,0);//填充
   //显示队名
   OLED_ShowString(0,0,"HJM SmartCarTeam" ,16);//显示字符串    X:列  Y：行
   OLED_ShowString(0,27,"SetSpeedValue:" ,12);//显示字符串
   Speed_Set = 530;     //初始化设置车子速度
   OLED_ShowNum(100,25,(uint8)(Speed_Set/10),2,12);
   OLED_Refresh_Gram();//更新显存到OLED 

/*******超声波测距程序 测试**************************/
	 pit_init_ms(pit1,1);								       //定时1ms后中断
	 set_irq_priority(PIT1_IRQn,2);						 //设置优先级,根据自己的需求设置，用于方向环和速度环
	 enable_irq(PIT1_IRQn);								     //打开pit1的中断开关
	 
	 
	 
	 	 pit_init_ns(pit2,10);
	 set_irq_priority(PIT2_IRQn,0);						//设置PIT0优先级为0，用于横断路障处理
//	 
	 port_init (C3, IRQ_FALLING | PF | ALT1 | PULLUP );	//初始化 C3 管脚，下降沿触发中断，带无源滤波器，复用功能为GPIO并设置为输入 ，上拉电阻
		port_init (C4, IRQ_FALLING | PF | ALT1 | PULLUP );	//初始化 C4 管脚，下降沿触发中断，带无源滤波器，复用功能为GPIO并设置为输入 ，上拉电阻
		set_irq_priority(PORTC_IRQn,1);						//设置优先级
		enable_irq(PORTC_IRQn);								//打开PORTC中断开关

	 gpio_init(E26,GPO,0);    //初始化发送端
	 gpio_init(A9,GPI,0);    //初始化接收端



/**********舵机/电机初始化程序 测试***********************/
		ftm_pwm_init(ftm1,ftm_ch1,100,1500); //舵机初始化,找中间 A13,ftm1模块，1通道   ftm1pwm精度设为10000 ！50hz 750 舵机中值
		//若更改舵机频率则必须更改中值headfile.h中 50HZ对应 750
		
		ftm_pwm_init(ftm0,ftm_ch7,10000,0);  //电机PWM初始化D7
		ftm_pwm_init(ftm0,ftm_ch5,10000,0); //电机PWM初始化D5


/******************* 正交解码初始化 ************************/
  
	    ftm_quad_init(ftm2);

		//ADC通道初始化    

	EMCS_init();//电磁传感器初始化包含获取各个电感的最大值
	while(key24)//等待key24按下
	{
		read_ad_test();
		
				gpio_set(A16,1);//关闭小灯
				systick_delay_ms(200);//延时100ms
				gpio_set(A16,0);//点亮小灯
			systick_delay_ms(200);//延时100ms
	}
	//等待按下按键

/*******陀螺仪初始化以及检测函数**************************/
//接线：模块TRIG接 E9  ECH0 接E7	 VCC接VCC（5V），GND接GND
//数码管：共阴极
//数码管显示所测距离（毫米）
//注意请勿带电插拔
/*********************************************************/
#if 0
	MPU_Init();	
	while(mpu_dmp_init())
	{
		printf("MPU6050 Error \r\n");
	}
	printf("MPU6050 OK\r\n");
#endif	
	/****************************************************/
	
		printf("init  OK\r\n");
		EnableInterrupts;									//打开总的中断开关
for(;;)
	{

		//testt();//测试用

	//	read_ad_test();
		
	   /***************LED闪烁检测程序是否正常循环**************/
	systick_delay_ms(200);//延时100ms
	   gpio_turn(A14);//反转D2状态测试程序是否正常循环
	  /****************************************************/
//   ultrasonic();//超声波测距·模块
	}
}


//按键扫描函数
void keycan()
{
			
	if(key1==0||key2==0||key3==0||key4==0)	
	{
		
		printf("KEY ok\r\n");
								if(key3==0)
									{  
										systick_delay_ms(4);
										if(key3==0)
											{  
													
												switch( SpeedPID_Set_max_num )
												{
													case 0: SpeedPID_Set_max = SpeedPID_Set_0_define  ; break;//4000
													case 1: SpeedPID_Set_max = SpeedPID_Set_1_define  ; break;
													case 2: SpeedPID_Set_max = SpeedPID_Set_2_define  ; break;
													case 3: SpeedPID_Set_max = SpeedPID_Set_3_define  ; break;
													case 4: SpeedPID_Set_max = SpeedPID_Set_4_define  ; break;
													case 5: SpeedPID_Set_max = SpeedPID_Set_5_define  ; break;
													case 6: SpeedPID_Set_max = SpeedPID_Set_6_define  ; break;
													case 7: SpeedPID_Set_max = SpeedPID_Set_7_define  ; break;
													case 8: SpeedPID_Set_max = SpeedPID_Set_8_define  ; break;
													case 9: SpeedPID_Set_max = SpeedPID_Set_9_define  ; break;
													case 10: SpeedPID_Set_max = SpeedPID_Set_10_define  ; break;
													case 11: SpeedPID_Set_max = SpeedPID_Set_11_define  ; break;
													case 12: SpeedPID_Set_max = SpeedPID_Set_12_define  ; break;
													case 13: SpeedPID_Set_max = SpeedPID_Set_13_define  ; break;
													case 14: SpeedPID_Set_max = SpeedPID_Set_14_define  ; break;
													default: break;
												}
												printf("max_num= %d    SpeedPID_Set_max = %d \r\n",SpeedPID_Set_max_num,SpeedPID_Set_max);
													SpeedPID_Set_max_num++ ;
													if( SpeedPID_Set_max_num > 9)  SpeedPID_Set_max_num = 0;				//若需解锁更大速度请将 SpeedPID_Set_max_num 最大值限定改动
												while(!key3);//等待按键松开
											}
								
									}
	
									
									
									if(key4==0)
									{  
										systick_delay_ms(4);
                                   
										if(key4==0)
											{  
										
												switch( SpeedPID_Set_min_num )
												{
													case 0: SpeedPID_Set_min = SpeedPID_Set_0_define  ; break;
													case 1: SpeedPID_Set_min = SpeedPID_Set_1_define  ; break;
													case 2: SpeedPID_Set_min = SpeedPID_Set_2_define  ; break;
													case 3: SpeedPID_Set_min = SpeedPID_Set_3_define  ; break;
													case 4: SpeedPID_Set_min = SpeedPID_Set_4_define  ; break;
													case 5: SpeedPID_Set_min = SpeedPID_Set_5_define  ; break;
													case 6: SpeedPID_Set_min = SpeedPID_Set_6_define  ; break;
													case 7: SpeedPID_Set_min = SpeedPID_Set_7_define  ; break;
													case 8: SpeedPID_Set_min = SpeedPID_Set_8_define  ; break;
													case 9: SpeedPID_Set_min = SpeedPID_Set_9_define  ; break;
													case 10: SpeedPID_Set_min = SpeedPID_Set_10_define  ; break;
													case 11: SpeedPID_Set_min = SpeedPID_Set_11_define  ; break;
													case 12: SpeedPID_Set_min = SpeedPID_Set_12_define  ; break;
													case 13: SpeedPID_Set_min = SpeedPID_Set_13_define  ; break;
													case 14: SpeedPID_Set_min = SpeedPID_Set_14_define  ; break;
													default: break;
												}
													printf("min_num= %d    SpeedPID_Set_min = %d \r\n",SpeedPID_Set_min_num,SpeedPID_Set_min);//若需解锁更大速度请将 SpeedPID_Set_max_num 最大值限定改动
														SpeedPID_Set_min_num++ ;
													if( SpeedPID_Set_min_num > 9 )  SpeedPID_Set_min_num = 0;
												while(!key4);//等待按键松开
											}
								
									}	
		}
	}
	





void  mpu6050(void)     //陀螺仪主函数
	{

	if(1)//0表示不执行
		{	
		    while(0)
			{
		          printf("mpu_dmp_get_data=%d\r\n",mpu_dmp_get_data(&pitch,&roll,&yaw));
				
			}
				//while(mpu_dmp_get_data(&pitch,&roll,&yaw));
   printf("mpu_dmp_get_data=%d\r\n",mpu_dmp_get_data(&pitch,&roll,&yaw));
		if(mpu_dmp_get_data(&pitch,&roll,&yaw)==0)
			{ 
						temp=MPU_Get_Temperature();	//得到温度值
			MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//得到加速度传感器数据
				MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//得到陀螺仪数据
				
							//三轴加速度参数值换算 -- 【 参数（g）= 相关寄存器读取值 / 灵敏度 】
									acce_val_x = aacx*0.0006103515625;
									acce_val_y = aacy*0.0006103515625;
									acce_val_z = aacz*0.0006103515625;
							//*************************************	
				
				if(1)
				{ 
					printf("\r\naacx=%.2f",acce_val_x);
					printf("    aacy=%.2f",acce_val_y);
					printf("    aacz=%.2f",acce_val_z);
				}
				if(1)//0表示不执行
				{ 
					if(temp<0)
					{
						temp=-temp;		//转为正数
					}
					temp=pitch*10;
					if(temp<0)
					{
						temp=-temp;		//转为正数
					}			printf("    pitch=%.2f",temp);
					
					temp=roll*10;
					if(temp<0)
					{
						temp=-temp;		//转为正数
					}			printf("    roll=%.2f",temp);
					
					temp=yaw*10;
//					if(temp<0)
//					{
//					temp=-temp;		//转为正数
//					}		
					printf("    yaw=%.2f",temp);

				}
			}
			
		}
			return;
	}                                      

	




void read_ad_test()
{
   uint16 data[10];
//		data[0] = adc_ave(ADC0_SE10, ADC_10bit, 50 );//采集A7脚电压，数字量，精度10位
//		data[1] = adc_ave(ADC1_SE10, ADC_10bit, 50 );//采集B4引脚电压，数字量，精度10位
//		data[2] = adc_ave(ADC1_SE14, ADC_10bit, 50 );  //采集B10引脚电压，数字量，精度10位
//		data[3] = adc_ave(ADC0_SE12, ADC_10bit, 50 );//采集B2脚电压，数字量，精度10位
//		data[4] = adc_ave(ADC0_SE17, ADC_10bit,50 );  //采集E24引脚电压，数字量，精度10位
//		data[5] = adc_ave(ADC0_SE11, ADC_10bit, 50 );//采集A8引脚电压，数字量，精度10位
//		data[6] = adc_ave(ADC0_SE8, ADC_10bit, 50 ); //采集B0引脚电压，数字量，精度10位	
//		data[7] = adc_ave(ADC1_SE12, ADC_10bit, 50 ); //采集B6引脚电压，数字量，精度10位
//		

		data[0] = adc_ave(ADC0_SE8, ADC_10bit, 50 );//采集B0脚电压，数字量，精度10位
		data[1] = adc_ave(ADC0_SE10, ADC_10bit, 50 );//采集A7引脚电压，数字量，精度10位
		data[2] = adc_ave(ADC1_SE10, ADC_10bit, 50 );  //采集B4引脚电压，数字量，精度10位
		data[3] = adc_ave(ADC1_SE14, ADC_10bit, 50 );//采集B10脚电压，数字量，精度10位
		data[4] = adc_ave(ADC1_SE13, ADC_10bit,50 );  //采集B7引脚电压，数字量，精度10位

		data[5] = adc_ave(ADC0_SE12, ADC_10bit, 50 );//采集B2引脚电压，数字量，精度10位
		data[6] = adc_ave(ADC0_SE11, ADC_10bit, 50 ); //采集A8引脚电压，数字量，精度10位	
		data[7] = adc_ave(ADC1_SE12, ADC_10bit, 50 ); //采集B6引脚电压，数字量，精度10位
    data[8] = adc_ave(ADC0_SE17, ADC_10bit, 50 ); //采集E24引脚电压，数字量，精度10位	
		data[9] = adc_ave(ADC1_SE11, ADC_10bit, 50 ); //采集A17引脚电压，数字量，精度10位
		
//	
//	 printf("A7=%d	",data[0]);
//	 printf("B4=%d	",data[1]);
//	 printf("B10=%d	",data[2]);
//	 printf("B2=%d	",data[3]);
//	 printf("E24=%d	",data[4]);
//	 printf("A8=%d	",data[5]);
//	 printf("B0=%d	",data[6]);
//	 printf("B6=%d	",data[7]);
//	printf("\r\n");
//	
   for( l=0;l < 10 ;l++)
	  {
			
			if( l== 2)
			{
				data[2]=  data[2] * AD_1_multiple;
		
			  printf("[2]=%d	",data[2]);
			}else
			{
	    printf("[%d]=%d	",l,data[l]);
				if(l==4)
				{
				}
			}
			
	  }printf("\r\n");printf("\r\n");
	
	
}
