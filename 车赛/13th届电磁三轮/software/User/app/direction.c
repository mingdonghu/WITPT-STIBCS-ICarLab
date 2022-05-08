/*!
 * @file       direction.c
 * @brief      小车方向传感器及控制平台程序
 * @author     胡明栋
 * @application   恩智浦杯三轮电磁组
 * @date       2018.7
 */

#include "direction.h"
#include "control.h"
#include "speed.h"

uint8 d = 0;
float Turn_control_out = 0;
float Turn_error = 0;  
uint8 m = 0;
uint8 n = 0;
uint8 cflag = 0;
 

    
/*EMC中值滤波法****/
int16 g_ValueOfAD[3]={0};		//获取的电感值      //[0] 左   [1] 右    [2] 中 
int16 g_ValueOfADFilter[4]={0};	//阶梯滤波的电感值
uint16 MIN_v[3] = {0,0,0};                     //[0] 左   [1] 右    [2] 中  //归一化
uint16 MAX_v[3] ;            

 
//方向PD控制
void Direction_control()
{
  static float Turn_error_last_1 = 0;
  static float Turn_error_last_2 = 0;
  static float Turn_error_last_3 = 0;
  static float Turn_error_last_4 = 0;
  static float Turn_error_last_5 = 0; 
  static float Turn_error_last_6 = 0;
  float Dir_nP = 0;
  float Dir_nD = 0;
    
  switch ( d )
  {
     case 1:                           //Speed_Set:380
             /***方向环比例控制**/
            Dir_nP = Dir_Kp1 * Turn_error ; 
           /***方向环微分控制***/
            Dir_nD = Dir_Kd1 * ( Turn_error - Turn_error_last_1 ) ;
            Turn_error_last_1 = Turn_error ;
            Turn_control_out = Dir_nP + Dir_nD ;
            break ;  
            
    case 2:                          //Speed_Set:470
            /***方向环比例控制**/
             Dir_nP = Dir_Kp2 * Turn_error ; 
            /***方向环微分控制***/
            Dir_nD = Dir_Kd2 * ( Turn_error - Turn_error_last_2 ) ;
            Turn_error_last_2 = Turn_error ;
            Turn_control_out = Dir_nP + Dir_nD ;
            break ;
            
    case 3:                         //Speed_Set:530
           /***方向环比例控制**/
            Dir_nP = Dir_Kp3 * Turn_error ; 
          /***方向环微分控制***/
           Dir_nD = Dir_Kd3 * ( Turn_error - Turn_error_last_3 ) ;
           Turn_error_last_3 = Turn_error ;
           Turn_control_out = Dir_nP + Dir_nD ;
           break ;
           
    case 4:                         //Speed_Set: 500
           /***方向环比例控制**/
            Dir_nP = Dir_Kp4 * Turn_error ; 
          /***方向环微分控制***/
           Dir_nD = Dir_Kd4 * ( Turn_error - Turn_error_last_4 ) ;
           Turn_error_last_4 = Turn_error ;
           Turn_control_out = Dir_nP + Dir_nD ;
           break ;
           
     case 5:                                //右弯道稳定处理    
           /***方向环比例控制**/
            Dir_nP = Dir_Kp5 * Turn_error ; 
          /***方向环微分控制***/
           Dir_nD = Dir_Kd5 * ( Turn_error - Turn_error_last_5 ) ;
           Turn_error_last_5 = Turn_error ;
           Turn_control_out = Dir_nP + Dir_nD ;
           break ;
           
     case 6:                                //左弯道稳定处理    
           /***方向环比例控制**/
            Dir_nP = Dir_Kp6 * Turn_error ; 
          /***方向环微分控制***/
           Dir_nD = Dir_Kd6 * ( Turn_error - Turn_error_last_6 ) ;
           Turn_error_last_6 = Turn_error ;
           Turn_control_out = Dir_nP + Dir_nD ;
           break ;      
     default :break;
  }
  
  //方向控制输出限幅
   if(Turn_control_out >= 70)
    Turn_control_out = 70 ;
  else if(Turn_control_out <= -70)
    Turn_control_out = -70 ;
  
  printf("Turn_control_out:%f\n",Turn_control_out);
  
}

//电磁传感器初始化
void EMC_init()           
{ 
   //ADC通道初始化    
   adc_init (ADC0_SE9);                 //C1   //中       
  // adc_init (ADC0_SE14);                //F6     //左 
   adc_init (ADC0_SE15);                //F7     //左   //带标识的主控板
   adc_init (ADC0_SE13);                //F5      //右
   
 // (中值滤波)获取每个电感的最大值
   get_ADMAXvalue_init();
}

 // (中值滤波)获取每个电感的最大值， 
void get_ADMAXvalue_init()       
{
  uint16 i,j;
  uint32  valu_get[3] = {0};
  MAX_v[0] = MAX_v[1] = MAX_v[2] = 0;
  
  for( i = 0; i < 5000; i++)
  { 
    valu_get[0] = adc_ave (ADC0_SE14, ADC_10bit,50 );     //F6    left 
    //valu_get[0] = adc_ave (ADC0_SE15, ADC_10bit,50 );     //F7    left    //带标识的主控板
    valu_get[1] = adc_ave (ADC0_SE13, ADC_10bit, 50 );   //F5     right
    valu_get[2] = adc_ave (ADC0_SE9, ADC_10bit, 50 );   //C1     mid
    for( j = 0; j < 3;j++)
    {
      if(valu_get[j] > MAX_v[j] )
        MAX_v[j] = valu_get[j];
    }
    DELAY_US(1);
  }
  
}

// (中值滤波)电感信号采集    
void Read_ADC()
{
  
     uint8   max_front = 0 ;
     int16  i,j,k,temp;
     int16  ad_valu[3][5],ad_valu1[3],ad_sum[3];
     int16 ValueOfADOld[4],ValueOfADNew[4];  
     float sensor_to_one[3];
     
     for(i=0;i<5;i++)
     {
         ad_valu[0][i] = adc_ave (ADC0_SE14, ADC_10bit,50 );     //F6    left
         //ad_valu[0][i] = adc_ave (ADC0_SE15, ADC_10bit,50 );     //F7    left   //带标识
         ad_valu[1][i] = adc_ave (ADC0_SE13, ADC_10bit, 50 );   //F5     right
         ad_valu[2][i] = adc_ave (ADC0_SE9, ADC_10bit, 50 );   //C1     mid		 
     }
/*=========================冒泡排序升序==========================*///舍弃最大值和最小值
     for(i=0;i<3;i++)
     {
        for(j=0;j<3;j++)
        {
           for(k=0;k<3-j;k++)
           {
              if(ad_valu[i][k] > ad_valu[i][k+1])        //前面的比后面的大  则进行交换
              {
                 temp = ad_valu[i][k+1];
                 ad_valu[i][k+1] = ad_valu[i][k];
                 ad_valu[i][k] = temp;
              }
           }
        }
     }
/*===========================中值滤波=================================*/
  for(i=0;i<3;i++)    //求中间三项的和
  {
    ad_sum[i] = ad_valu[i][1] + ad_valu[i][2] + ad_valu[i][3];
    ad_valu1[i] = ad_sum[i] / 3;
  }

  for(i=0;i<3;i++)            //将数值中个位数除掉
  {
    g_ValueOfAD[i] = (int16)(ad_valu1[i]/10*10);

    //采集梯度平滑，每次采集最大变化40
    ValueOfADOld[i] = g_ValueOfADFilter[i];
    ValueOfADNew[i] = g_ValueOfAD[i];
      
    if(ValueOfADNew[i]>=ValueOfADOld[i])
    {
      g_ValueOfADFilter[i] = ((ValueOfADNew[i]-ValueOfADOld[i])>50?(ValueOfADOld[i]+50):ValueOfADNew[i]);
    }
    else
    {
      g_ValueOfADFilter[i] = ((ValueOfADNew[i]-ValueOfADOld[i])<-60?(ValueOfADOld[i]-60):ValueOfADNew[i]);
    }
  }
      
  //AD采样值归一化处理
  for(i=0;i<3;i++)
  {
     sensor_to_one[i] = (float)(g_ValueOfADFilter[i]-MIN_v[i])/(float)(MAX_v[i]-MIN_v[i]);
     if(sensor_to_one[i] <= 0.0)
       sensor_to_one[i] = 0.001;
     if(sensor_to_one[i] > 1.0)
       sensor_to_one[i] = 1.0;
     g_ValueOfAD[i] =(uint8) ( 100 * sensor_to_one[i] );  //归一化后的值，0~100之间
  }
 
#if 0 
  printf("\n左边采样为%d", g_ValueOfAD[0]);
  printf("       中间一采样为%d", g_ValueOfAD[2]);
  printf("       右边采样为%d\n", g_ValueOfAD[1]);
#endif
     
   //选出感应最强的电感
  for( i = 0; i <3 ; i++ )
  {
    if( g_ValueOfAD[max_front] < g_ValueOfAD[i] )
      max_front = i  ;
  }
   n = max_front ;
  //寻找特征值,识别赛道元素  
  if( n==0)  //left max      
  {
    cflag = 2 ;     //弯道稳定处理，左弯      
  }
  else if( n==1 ) //right max                        
  {
    cflag = 3 ;     //弯道稳定处理，右弯       
  }
  else if( n==2 )   //mid max
  {
    cflag = 1 ;  //直道、弯道方向控制处理
  }

  //方向偏差计算
  switch( cflag )
  {     
    //midMAX      // (zuo - you)/(zuo + you) ,直道弯道
    case 1:  
      Turn_error = ExtractDifferentialRatioAnd( g_ValueOfAD[0] , g_ValueOfAD[1] );
      // printf("直道弯道Turn_error:%f\n",Turn_error);      
      if( Speed_Set == 560 )
      {
        d = 1 ;   //直道弯道控制标志位
      }
      else if( Speed_Set == 470 )
      {
        d = 2 ;   //直道弯道控制标志位
      }
      else if( Speed_Set == 530 )
      {
        d = 3 ;   //直道弯道控制标志位
      }
      else if( Speed_Set == 500 )
      {
        d = 4;     //直道弯道控制标志位
      }
      break;
           
    //left max     //(zuo - zhong)/(zuo + you + zhong)
    case 2 :  
      Turn_error = DifferentialRatioAnd( g_ValueOfAD[0] , g_ValueOfAD[2] , g_ValueOfAD[1] );
      //printf("left max Turn_error:%f\n",Turn_error);
      d = 5;       //右弯道稳定处理
      break;
    
    //right max      //(zhong - you)/(zuo + you + zhong)
    case 3 : 
      Turn_error = DifferentialRatioAnd( g_ValueOfAD[2] , g_ValueOfAD[1] , g_ValueOfAD[0] );
      //printf("right max  Turn_error:%f\n",Turn_error); 
      d = 6;       //左弯道稳定处理
      break;
           
    default :break;
  }  
      	 
}


