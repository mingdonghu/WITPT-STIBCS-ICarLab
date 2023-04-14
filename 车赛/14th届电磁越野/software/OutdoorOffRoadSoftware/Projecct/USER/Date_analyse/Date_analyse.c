/******************** (C) COPYRIGHT 2011 ********************* ********************
 * 文件名       ：Date_analyse.c
 * 描述         ：电感数据采集与分析
 *
 * 实验平台     ：野火kinetis开发板
 * 库版本       ：
 * 嵌入系统     ：
 *
 * 作者         ：oО殇のＳo 
**********************************************************************************/   
#include "Date_analyse.h"
#include "headfile.h"
#include "common.h"


/*****************************电感相关函数定义****************************/
#define   NM    3
#define SECTOR_ADM 253//扇区
uint16 duty;//求出偏差值后给舵机的具体数据
#define g_min 6 //信号丢失限定值

float Dir_Kp_Turn_error_13 = Dir_Kp_Turn_error_1_13;   //1和3号电感误差倍数
float Dir_Kp_Turn_error_04 = Dir_Kp_Turn_error_1_04;  //0和4号电感误差倍数
float Dir_Kp_Turn_error_57 = Dir_Kp_Turn_error_1_57;  //5和7号电感误差倍数
float Dir_Kp_Turn_error_01 = 1;  //5和7号电感误差倍数  
float Dir_Kp_Turn_error_34 = 1;//5和7号电感误差倍数

float ultrasonic_distance=0;//超声波_距离

	uint8 i;
	uint8 num=0,mun_1=0;	
	uint16  Count_delay = 0 ;
	uint8 roundabout_num_delayed=0;
	uint8 roundabou_num_first=1;
	uint8 roundabou_num_second=1;
	uint8  flag_area_second_last=0;
//AD中值滤波法
	int16  g_ValueOfAD[10]={0};		//获取的电感值
	int16  g_ValueOfAD_last[10]={0};		//获取的电感值

	uint16 AD_max_123;//记录123电感的最大值
	uint8 flag_area_first  =0;//第一次记录各个区域的标志位
	uint8 flag_area_second=0;//第二次记录各个区域的标志位
	uint16 MIN_v[10] = {0};       //[0]~[7]:五个电感  //归一化最小值
	uint16 MAX_v[10] = {0} ;             //[0]~[7]:五个电感  //归一化最大值
	uint32 roundabou_num=0;

	uint8	flag_roundabou=7;
/****************************/

int16  AD_valu[10],AD_V[10][NM],chazhi,chazhi_old;
float  AD[8],sensor_to_one[10];
float  Slope_AD_1;  // 用于坡道检测
int16  max_v[10],min_v[10];  //电感标定 采集值
int16  Position_transit[10];  //记录过渡点归一化的值

int16  AD_sum[10]; 

int16  position = 2,position_back = 1;
float  max_value,AD_0_max,AD_1_max,AD_2_max,AD_3_max;
int  q=0;//某些循环用到

int16  asd4[4] ,asd5[5],asd_5[5];  
 
 
 
 
void testt(void)//测试的时候用
{
		 
		//	uint16  i;
				asd4[0] = 41;
				asd4[1] = 42; 
				asd4[2] = 43;
				asd4[3] = 43; 
	
				asd5[0] = 253;
				asd5[1] = 254; 
				asd5[2] = 255;
				asd5[3] = 256; 
				asd5[4] = 257; 
	if(key3==0)	
		
	{
		printf("FLASH_GetSectorSize=%d\r\n",	 FLASH_GetSectorSize());//获取扇区大小
   FLASH_EraseSector(SECTOR_ADM);//FLASH擦除253扇区   SECTOR_ADM == 253
  FLASH_WriteSector(SECTOR_ADM,(const uint8 *)asd5,16,0);		//储存数据		 
 

			 while(!key3);
	 }
	if(key4==0)	
		
	{	
         asd_5[0] = flash_read(SECTOR_ADM,0,uint32);
         asd_5[1] = flash_read(SECTOR_ADM,4,uint32);
         asd_5[2] = flash_read(SECTOR_ADM,8,uint8);
         asd_5[3] = flash_read(SECTOR_ADM,6,uint8);
         asd_5[4] = flash_read(SECTOR_ADM,8,uint8);
				for (q = 0; q < 5; q++) 
				{
					printf("asd_5[%d] = %d	",q,asd_5[q]);
				}
					printf("\r\n");
//				for (q = 0; q < 4; q++) 
//				{
//					printf("asd4[%d] = %d	",q,asd4[q]);
//				}
//					printf("\r\n");
			while(!key4);
	 }
}

//电磁传感器初始化
void EMCS_init(void)
{
  
	
   uint32  valu_get[10] = {0};
//    MAX_v[0] = MAX_v[1] = MAX_v[2] = MAX_v[3] = MAX_v[4] = 0;


	//ADC通道初始化    
	adc_init(ADC0_SE8);  //B0引脚，具体通道与引脚对应关系可以查看MK60DN10_adc.h
	adc_init(ADC1_SE10); //B4引脚，具体通道与引脚对应关系可以查看MK60DN10_adc.h
	adc_init(ADC0_SE10); //A7引脚，具体通道与引脚对应关系可以查看MK60DN10_adc.h
	adc_init(ADC1_SE12); //B6引脚，具体通道与引脚对应关系可以查看MK60DN10_adc.h
	adc_init(ADC0_SE11); //A8引脚，具体通道与引脚对应关系可以查看MK60DN10_adc.h
	adc_init(ADC1_SE14); //B10引脚，具体通道与引脚对应关系可以查看MK60DN10_adc.h
	adc_init(ADC0_SE17); //E24引脚，具体通道与引脚对应关系可以查看MK60DN10_adc.h
	adc_init(ADC0_SE12); //B2引脚，具体通道与引脚对应关系可以查看MK60DN10_adc.h
	adc_init(ADC1_SE13);//B7引脚，具体通道与引脚对应关系可以查看MK60DN10_adc.h
	adc_init(ADC1_SE17);//A17引脚，具体通道与引脚对应关系可以查看MK60DN10_adc.h
   
	  MAX_v[0]=EMCS_MAX;//固定电感最大值 下同
	  MAX_v[1]=EMCS_MAX;
	  MAX_v[2]=EMCS_MAX;
	  MAX_v[3]=EMCS_MAX;
	  MAX_v[4]=EMCS_MAX;
	  MAX_v[5]=EMCS_MAX;
	  MAX_v[6]=EMCS_MAX;
	  MAX_v[7]=EMCS_MAX;
	  MAX_v[8]=EMCS_MAX;
	  MAX_v[9]=EMCS_MAX;
	
//	//获取每个电感的最大值
//	for( i = 0; i < 5000; i++)
//     { 
//        valu_get[0] = adc_ave(ADC0_SE10, ADC_10bit,50 );     //采集a7引脚电压，数字量，精度10位
//        valu_get[1] = adc_ave(ADC1_SE10, ADC_10bit, 50 );   //采集B4引脚电压，数字量，精度10位
//        valu_get[2] = adc_ave(ADC1_SE14, ADC_10bit, 50 );  //采集B10引脚电压，数字量，精度10位
//        valu_get[3] = adc_ave(ADC0_SE12, ADC_10bit, 50 );//采集B2脚电压，数字量，精度10位
//        valu_get[4] = adc_ave(ADC0_SE17, ADC_10bit, 50 );		//采集e24引脚电压，数字量，精度10位
//				valu_get[5] = adc_ave(ADC0_SE11, ADC_10bit, 50 );  //采集A8引脚电压，数字量，精度10位
//        valu_get[6] = adc_ave(ADC0_SE8, ADC_10bit, 50 );//采集B0引脚电压，数字量，精度10位
//        valu_get[7] = adc_ave(ADC1_SE12, ADC_10bit, 50 );		//采集B6引脚电压，数字量，精度10位
//	
//	
//			 	
//        for( j = 0; j < 8;j++)
//          {
//             if(valu_get[j] > MAX_v[j] )
//							{
//               MAX_v[j] = valu_get[j];
//							}
//          }
//     	systick_delay_ms(1);
//     }
//		 
////		 if(key4==0)//按下key4重新获取最大值
////		 {
////			 	printf("FLASH_GetSectorSize=%d\r\n",	 FLASH_GetSectorSize());//打印扇区大小
////				FLASH_EraseSector(SECTOR_ADM);//FLASH擦除253扇区   SECTOR_ADM == 253
////			 FLASH_WriteSector(SECTOR_ADM,(const uint8 *)MAX_v,10,0);		//储存数据		

////			 

////			 
////			 
////			}else
////		 {
////			 
////         MAX_v[0] = flash_read(SECTOR_ADM,0,uint8);
////         MAX_v[1] = flash_read(SECTOR_ADM,2,uint8);
////         MAX_v[2] = flash_read(SECTOR_ADM,4,uint8);
////         MAX_v[3] = flash_read(SECTOR_ADM,6,uint8);
////         MAX_v[4] = flash_read(SECTOR_ADM,8,uint8);
////		 
////		 }
//			
//			
//		 		   for(i=0;i<8;i++)
//		{
//		 	printf(" MAX_v[%d]=%d	 ", i,MAX_v[i] );
//		 }printf ("\r\n");
//		 
	
	return;
}



/*************************************************************************
*  函数名称   Read_ADC
*  功能说明： AD采集
*  参数说明：         
*  函数返回： 无
*  修改时间：
*  备    注：
*************************************************************************/
void Read_ADC(void)
{		
		int16  i = 0;
    int16  ad_valu[10][5],ad_valuMID[10],ad_sum[10];
//		static int16 ValueOfADOld[5]={0};
		int16 g_ValueOfADFilter[10]={0}; //阶梯滤波后的电感值
//		int16 ValueOfADNew[5];  
     float sensor_to_one[10];
     
     for(i=0;i<5;i++)
     {	 
        ad_valu[0][i] = adc_ave(ADC0_SE8,  ADC_10bit, 50 );//采集B0引脚电压，数字量，精度10位
        ad_valu[1][i] = adc_ave(ADC0_SE10, ADC_10bit, 50 );//采集A7引脚电压，数字量，精度10位
        ad_valu[2][i] = adc_ave(ADC1_SE10, ADC_10bit, 50 );//采集B4引脚电压，数字量，精度10位
        ad_valu[3][i] = adc_ave(ADC1_SE14, ADC_10bit, 50 );//采集B10引脚电压，数字量，精度10位
        ad_valu[4][i] = adc_ave(ADC1_SE13, ADC_10bit, 50 );//采集B7引脚电压，数字量，精度10位
				ad_valu[5][i] = adc_ave(ADC0_SE12, ADC_10bit, 50 );//采集B2引脚电压，数字量，精度10位
				ad_valu[6][i] = adc_ave(ADC0_SE11, ADC_10bit, 50 );//采集A8引脚电压，数字量，精度10位
        ad_valu[7][i] = adc_ave(ADC1_SE12, ADC_10bit, 50 );//采集B6引脚电压，数字量，精度10位
				ad_valu[8][i] = adc_ave(ADC0_SE17, ADC_10bit, 50 );//采集E24引脚电压，数字量，精度10位
        ad_valu[9][i] = adc_ave(ADC1_SE17, ADC_10bit, 50 );	//采集A17引脚电压，数字量，精度10位
     }
////	 
//// /**************冒泡排序升序***************/
////	 for(i=0;i<5;i++)   //五个ADC通道
////	 {
////			for(j=0;j<5;j++)  //每个ADC通道获取五次值
////			{
////				 for(k=0;k<5-1-j;k++)
////				 {
////					 if(ad_valu[i][k] > ad_valu[i][k+1])        //前面的比后面的大  则进行交换
////						{
////							 ad_valu[i][k] = ad_valu[i][k] ^ ad_valu[i][k+1];
////			        ad_valu[i][k+1] = ad_valu[i][k] ^ ad_valu[i][k+1];
////			        ad_valu[i][k] = ad_valu[i][k] ^ ad_valu[i][k+1];
////						}
////				 }
////			}
////	 }
//		 /*****************************/
     for(i=0;i<10;i++)    //求中间三项的和，舍弃最大值和最小值
      {
        ad_sum[i] = ad_valu[i][1] + ad_valu[i][2] + ad_valu[i][3];
        ad_valuMID[i] = ad_sum[i] / 3;
      }
			//打印未滤波得值
//				for(i=0;i<8;i++)
//		{
//		 	printf("ad_valuMID[%d]=%d	 ",i , ad_valuMID[i] );
//			
//		 }
//		printf ("\r\n");
		

     for(i=0;i<10;i++)            //将数值中个位数除掉
      {
	    g_ValueOfAD[i] = (int16)(ad_valuMID[i]/10*10);
			g_ValueOfADFilter[i] = g_ValueOfAD[i]; //将阶梯滤波屏蔽！未用到阶梯滤波
	    /***采集梯度平滑(阶梯滤波)***********************************************/
	//    ValueOfADOld[i] = g_ValueOfADFilter[i];  //记录上一次阶梯滤波的电感值
//	    ValueOfADNew[i] = g_ValueOfAD[i];

//        if(ValueOfADNew[i] >= ValueOfADOld[i])
//         {
//           g_ValueOfADFilter[i] = ((ValueOfADNew[i]-ValueOfADOld[i])>50?(ValueOfADOld[i]+50):ValueOfADNew[i]);
//         }
//	   else
//         {
//	       g_ValueOfADFilter[i] = ((ValueOfADNew[i]-ValueOfADOld[i])<-60?(ValueOfADOld[i]-60):ValueOfADNew[i]);
//         }
      }
    //AD采样值归一化处理

    for(i=0;i<10;i++)
     {
        sensor_to_one[i] = (float)(g_ValueOfADFilter[i]-MIN_v[i])/(float)(MAX_v[i]-MIN_v[i]);
        if(sensor_to_one[i] <= 0.0)
																	sensor_to_one[i] = 0.001;
        if(sensor_to_one[i] > 1.0)
																	sensor_to_one[i] = 1.0;
        g_ValueOfAD[i] =(uint8) ( 100 * sensor_to_one[i] );  //归一化后的值，0~100之间
    }
			
//		 			
//AD_judge_limit_min =20
		//AD_judge_limit_max=90
		

	if ( ( g_ValueOfAD[0] >AD_judge_limit_min  || g_ValueOfAD[2] * AD_1_multiple >AD_judge_limit_min || 
g_ValueOfAD[4] >AD_judge_limit_min  ) && (( g_ValueOfAD[1] < AD_judge_limit_max  && g_ValueOfAD[2] * AD_1_multiple < AD_judge_limit_max &&  g_ValueOfAD[3] < AD_judge_limit_max   ))) //AD_judge_limit//是否进行区域判定限定值
	{
	
//		printf("uH[1]=%d ",g_ValueOfAD[1]);
//		printf("uH[2]=%d : %f ",g_ValueOfAD[2],g_ValueOfAD[2] * AD_1_multiple);
//		printf("uH[3]=%d ",g_ValueOfAD[3]);
//						printf("\r\n");

		//第一次大范围三区域判断
		for( i = 0; i < 5; i+=2 )
		{		
				if(AD_max_123 < g_ValueOfAD[i])		
			{
				AD_max_123 = g_ValueOfAD[i];
				flag_area_first = i;
			}
			if(g_ValueOfAD[2]*AD_1_multiple > g_ValueOfAD[flag_area_first])
			{
			   AD_max_123 = g_ValueOfAD[2] * AD_1_multiple ;
					flag_area_first  = 2;
			}
		}		AD_max_123=0;		//最大值复位


//第二次精确6区域判断

		switch(flag_area_first)
		{
				
			case 0:	
							if(g_ValueOfAD[2] < AD_Area_limit_1  ) 
							{
							flag_area_second = 9 ;
							}	else if(g_ValueOfAD[2] < AD_Area_limit_2  ) 
											{
											flag_area_second = 0 ;
											}				else 
															{
															flag_area_second = 1;
															} 
							break;
						
			case 2: 	flag_area_second = 3;break;
			case 4: 	
								if(g_ValueOfAD[2] < AD_Area_limit_1  ) 
								{
									flag_area_second =10 ;
								}else  if(g_ValueOfAD[2] < AD_Area_limit_2  ) 
												{
													flag_area_second =5 ;
												}
													else 
															{
															flag_area_second = 4;
															};
								break;
			default: break;
		}
		
		
		//补充判定
		if( g_ValueOfAD[0] <AD_judge_limit_min && g_ValueOfAD[4] < AD_judge_limit_min &&  ( flag_area_second == 9 || flag_area_second ==10))
		{
					if( g_ValueOfAD[1] > g_ValueOfAD[3] )
					{
								flag_area_second = 9;
					}else{
								flag_area_second = 10;
					}
		}


	} 
	

//	
//	if(key29)
//{
//	if(g_ValueOfAD[5] > AD_crossroad_limit && g_ValueOfAD[7] > AD_crossroad_limit )
//															{			// AD_crossroad_limit = 70、、567号电感全大于AD_crossroad_limit识别十字路口
////																	
//														//十字路口判断
//															flag_area_second=8;
//															//printf("crossroad\r\n");
//															}else//未满足上述条件！识别为正常路况
//																{
//															//	未为满足上述条件不改变标志位
//																}
//																
//																
//		
//				
//			//环岛判断
//				 if( roundabou_num_first  && g_ValueOfAD[8] > 45 && ( g_ValueOfAD[1] > AD_roundabout_limit || g_ValueOfAD[3] > AD_roundabout_limit ))//未区别坡道
//					{
//									roundabout_num_delayed = 0;//每次识别环岛后在次延时 防止同一次多次识别环岛
//									roundabou_num_first = 0;
//												 if(g_ValueOfAD[5] > g_ValueOfAD[7] )
//																 {
//																						if( roundabou_num_second )
//																						{
//																															printf("zuo11111111111\r\n");
//																															Count_delay = 0;
//																															roundabou_num_second=0;
//																															Dir_Kp_Turn_error_01 =3 ;
//																															Dir_Kp_Turn_error_34 = 0.7;
//																								
//																						}else{						printf("zuo2222222222\r\n");
//																															Count_delay = 650;
//																															roundabou_num_second=1;
//																															Dir_Kp_Turn_error_01 = 0.8;
//																															Dir_Kp_Turn_error_34 = 1.2;
//																															Dir_Kp_Turn_error_04 = 0;
//																									}
//																						 
//													}else if(g_ValueOfAD[5] < g_ValueOfAD[7] )//右环岛
//																{
//																	
//																						if( roundabou_num_second )
//																						{	
//																															printf("you11111111111\r\n");
//																															Count_delay = 0;
//																															roundabou_num_second=0;
//																															Dir_Kp_Turn_error_01 = 0.7 ;
//																															Dir_Kp_Turn_error_34 = 3;
//																							
//																								}else{				printf("you2222222222\r\n");
//																															Count_delay = 550;
//																															roundabou_num_second=1;
//																															Dir_Kp_Turn_error_01 = 1.2;
//																															Dir_Kp_Turn_error_34 = 0.8;
//																															Dir_Kp_Turn_error_04 = 0;
//																									}
//												 
//																 }else{					 	printf("		[5] = [7]\r\n");}
//											

//					}
//			

//		//环岛延时
//			roundabout_num_delayed++;
//			if(roundabout_num_delayed > 130)
//			{
//					roundabout_num_delayed = 161;//防止溢出
//						roundabou_num_first = 1;
//			}

//			Count_delay++;
//			if(Count_delay > 750 )
//			{
//							Count_delay=651;//防止溢出 若Count_delay未置零则每次都需执行这段函数
////						gpio_set(A16,1);//关闭小灯
////				systick_delay_ms(1000);//延时100ms
////				gpio_set(A16,0);//点亮小灯
//				
//								Dir_Kp_Turn_error_04 = Dir_Kp_Turn_error_1_04;
//								Dir_Kp_Turn_error_01 = 1;
//								Dir_Kp_Turn_error_34 = 1;
//								roundabou_num_second=1;  //若判定第一次进环且没有判定第二次出环，则将第一次进环判定使能
//			}
//			
//			
//}	
	

	
	if(key29)
{
	if(g_ValueOfAD[5] > AD_crossroad_limit && g_ValueOfAD[7] > AD_crossroad_limit )
															{			// AD_crossroad_limit = 70、、567号电感全大于AD_crossroad_limit识别十字路口
//																	
														//十字路口判断
															flag_area_second=8;
															//printf("crossroad\r\n");
															}else//未满足上述条件！识别为正常路况
																{
															//	未为满足上述条件不改变标志位
																}
																
																
		
				
			//环岛判断
				 if(  g_ValueOfAD[8] > 70 && ( g_ValueOfAD[1] > AD_roundabout_limit || g_ValueOfAD[3] > AD_roundabout_limit ))//未区别坡道
					{
//									roundabout_num_delayed = 0;//每次识别环岛后在次延时 防止同一次多次识别环岛
//									roundabou_num_first = 0;
										flag_area_second = 7 ;

					}
			

		//环岛延时
			roundabout_num_delayed++;
			if(roundabout_num_delayed > 143)
			{
					roundabout_num_delayed = 161;//防止溢出
						roundabou_num_first = 1;
			}

//			Count_delay++;
//			if(Count_delay > 750 )
//			{
//							Count_delay=651;//防止溢出 若Count_delay未置零则每次都需执行这段函数
////						gpio_set(A16,1);//关闭小灯
////				systick_delay_ms(1000);//延时100ms
////				gpio_set(A16,0);//点亮小灯
//				
//								Dir_Kp_Turn_error_04 = Dir_Kp_Turn_error_1_04;
//								Dir_Kp_Turn_error_01 = 1;
//								Dir_Kp_Turn_error_34 = 1;
//								roundabou_num_second=1;  //若判定第一次进环且没有判定第二次出环，则将第一次进环判定使能
//			}
			
			
}	
		 if( roundabou_num_first)
		 {
			//区域判定并且确定误差值
					switch ( flag_area_second )
					{

						case 9:  Turn_error = Turn_error_max;break;//临界点                                                                      
						
						
						case 0:										;
						case 1:               		;
						case 2:               		;
						case 3:               		;	
						case 5:                   ;
						case 4:  
									  Turn_error = ( Dir_Kp_Turn_error_13 * (float)( Dir_Kp_Turn_error_01 * g_ValueOfAD[1] - Dir_Kp_Turn_error_34 * g_ValueOfAD[3])
											+ Dir_Kp_Turn_error_04 * (float)(  Dir_Kp_Turn_error_01 * g_ValueOfAD[0] - Dir_Kp_Turn_error_34 * g_ValueOfAD[4])) /
										(Dir_Kp_Turn_error_13 *(g_ValueOfAD[1] +g_ValueOfAD[3] )+ AD_1_multiple * g_ValueOfAD[2] + Dir_Kp_Turn_error_04 * (g_ValueOfAD[0] + g_ValueOfAD[4]));
											
											
//										 Turn_error = ( Dir_Kp_Turn_error_13 * (float)( Dir_Kp_Turn_error_01 * g_ValueOfAD[1] - Dir_Kp_Turn_error_34 * g_ValueOfAD[3])
//											+ Dir_Kp_Turn_error_04 * (float)(  Dir_Kp_Turn_error_01 * g_ValueOfAD[0] - Dir_Kp_Turn_error_34 * g_ValueOfAD[4])) /
//										(Dir_Kp_Turn_error_13 *(g_ValueOfAD[1] +g_ValueOfAD[3] )+ AD_1_multiple * g_ValueOfAD[2] +  (g_ValueOfAD[0] + g_ValueOfAD[4]));
//									
									break;//正常跑//2号电感原MAX=900可以正常跑但最大值调到200就不能正常跑！为了兼容环岛！将2号电感放大4.3被（未调到最佳数值）
						
					
						case 10:   Turn_error = -Turn_error_max;break; // 临界点
						
						case 6: 			;break;
						case 7:  
//							Turn_error= Dir_Kp_Turn_error_57 * (float)(g_ValueOfAD[5]-g_ValueOfAD[7])/(g_ValueOfAD[5]+g_ValueOfAD[7]);
									roundabout_num_delayed = 0;//每次识别环岛后在次延时 防止同一次多次识别环岛
									roundabou_num_first = 0;
							

					    		if(g_ValueOfAD[5] > g_ValueOfAD[7])
							    {
									    Turn_error =0.4; 
							
							    }
							   else
							    {
										 Turn_error = -0.4 ;
							    }  
								 break;
						
						case 8:	Turn_error = Dir_Kp_Turn_error_57 * (float)(g_ValueOfAD[1]-g_ValueOfAD[3]) /(g_ValueOfAD[1]+g_ValueOfAD[2]+g_ValueOfAD[3]) ;break;//十字路口
						
			
					  default: break;
									
								}
		
					}

//		else{
//			  	Turn_error= Dir_Kp_Turn_error_57 * (float)(g_ValueOfAD[5]-g_ValueOfAD[7])/(g_ValueOfAD[5]+g_ValueOfAD[7]);
//				}
//	

				
 
//速度设定控制环	
		if(  flag_area_second ==  10||flag_area_second ==  9)
			{
				SpeedPID_Set = SpeedPID_Set_min ;
					
			}	
			else {   //直到加速弯道减速
//				Turn_error = 0.5;
//printf("%f\r\n",( (myabs(Turn_error) * Dir_Kp / Turn_error_max < 1 ? (myabs(Turn_error) * Dir_Kp / Turn_error_max) : 1 ) ))				 ; 
				SpeedPID_Set = SpeedPID_Set_max -	(((myabs(Turn_error) * Dir_Kp / Turn_error_max < 1) ? (myabs(Turn_error) * Dir_Kp / Turn_error_max) : 1 ) ) * ( SpeedPID_Set_max - SpeedPID_Set_min );
						

			//	SpeedPID_Set = SpeedPID_Set_max -	( myabs(Turn_error) * Dir_Kp / Turn_error_max)  * ( SpeedPID_Set_max - SpeedPID_Set_min );
	
							} 
	
		
		if(key24)
		{
			
			SpeedPID_Set=( flag_area_second == 3 ? SpeedPID_Set_5_define : SpeedPID_Set_5_define);
			//SpeedPID_Set=( flag_area_second == 3 ? SpeedPID_Set_3_define : SpeedPID_Set_0_define);

		}
		return ;
}



/*
*Function: 方向环位置式PD控制
*
*  形参对应的实参为全局变量
* Turn_error : 小车相对于赛道中心线的偏差
* Dir_Kp : 方向环比例系数
* Dir_Kd : 方向环微分系数
*函数调用示例：direction_control(&Turn_error,&Dir_Kp,&Dir_Kd)；  
*/
void direction_control(float *Turn_error,uint16 *Dir_Kp,uint16 *Dir_Kd)
{
	static float Turn_error_last = 0;

	float Dir_nP = 0;
    float Dir_nD = 0;
	float Turn_control_out = 0;
	/***方向环比例控制**/
    Dir_nP = (*Dir_Kp)  *(*Turn_error) ; 
    /***方向环微分控制***/
    Dir_nD = (*Dir_Kd) * (*Turn_error - Turn_error_last  );
    Turn_error_last =* Turn_error ;
    Turn_control_out = Dir_nP + Dir_nD ;
	//方向控制输出限幅         //具体限幅根据舵机占空比对应角度的范围来定
    Turn_control_out = (Turn_control_out>=Turn_error_max ? Turn_error_max : Turn_control_out);
    Turn_control_out = (Turn_control_out<=-Turn_error_max  ? -Turn_error_max : Turn_control_out);//1230 1500 1810

    /***舵机驱动函数：duty = mid+Turn_control_out或mid-Turn_control_out***/

		duty = Direction_Median-Turn_control_out;//左-右>0  应左转      小于舵机中值左转 大于舵机中值左转

if(key25)
{
num++;
		if(num > 20|| (flag_area_second_last != flag_area_second))//当区域判定改变的时候打印该数值
{
		flag_area_second_last = flag_area_second;
num=0;
//	
		printf("[0]=%d	",g_ValueOfAD[0]);
		printf("[1]=%d	",g_ValueOfAD[1]);
		printf("[2]=%d	",g_ValueOfAD[2]);
		printf("[3]=%d	",g_ValueOfAD[3]);
		printf("[4]=%d	",g_ValueOfAD[4]);
		printf("[5]=%d	",g_ValueOfAD[5]);
		printf("[7]=%d	",g_ValueOfAD[7]);
		printf("[8]=%d	",g_ValueOfAD[8]);
	
//		printf("flag_area_first=%d	",flag_area_first);
		printf("f_s=%d	",flag_area_second);
//		printf("Turn_control_out=%f	",Turn_control_out);
//	
//	if(flag_area_second ==5 ) 	printf("Turn_error=%d	",Turn_error);
		printf("dt=%d	\r\n",duty);
}
}

  //舵机占空比设置	
	ftm_pwm_duty(ftm1,ftm_ch1,(uint32)duty);  
	return;
}




/*
*  速度PID控制
* 调用实例： Speed_control(&SpeedPID_Set,&Spe_Kp,&Spe_Ki,&Spe_Kd);
*上例中函数形参为指针，为了使用将实参确定为全局变量，已在前面声明，调用是取其变量的地址
*  SpeedPID_Set ： 速度设定值
*  Spe_Kp  ：   速度环比例系数
*  Spe_Ki  ：   速度环积分系数
*  Spe_Kd  ：   速度环微分系数
*/
void Speed_control(float *Speed_Set,float *Spe_Kp,float *Spe_Ki,float *Spe_Kd)
{
	

  static float Speed_error_last = 0 ;  //上一次速度偏差
  static float Speed_error_last_last = 0 ;  //上上次速度偏差
  static float  Speed_control_out_Last = 0;
  float Speed_control_out = 0;  //速度环输出
  float Spe_nP = 0;  //比例项
  float Spe_nI = 0;  //积分项
  float Spe_nD = 0;  //微分项
  int16 Feedback_Speed = 0;  //反馈速度
  float Speed_error = 0;    //速度偏差
	
//printf("Speed_Set=	%f\r\n",SpeedPID_Set);//
  /*获取电机速度*/
  Feedback_Speed = ftm_quad_get(ftm2);
  ftm_quad_clean(ftm2);
//	printf("Feedback_Speed=	%d\r\n",Feedback_Speed>>1);//
  /*计算速度偏差*/
  Speed_error =  (*Speed_Set) - ( Feedback_Speed>>1);//备份时2019年4月26日( Feedback_Speed>>1);等价于 ( Feedback_Speed/2);
  //printf("Speed_Set=	%f	\r\n",*Speed_Set);
	
  /*增量式PID*/
  Spe_nP = (*Spe_Kp) * (Speed_error- Speed_error_last) ;
//  printf("Speed_error=	%f	",Speed_error);
//	printf("Speed_error_last=	%f	",Speed_error_last);
//	
//  
  Spe_nI = (*Spe_Ki) * Speed_error;
  Spe_nD = (*Spe_Kd) * (  Speed_error - 2 * Speed_error_last + Speed_error_last_last ) ;
  Speed_error_last_last = Speed_error_last ;//历史偏差数据
  Speed_error_last = Speed_error ;
  
  /*****速度环积分限幅***********************/
  Spe_nI = ( Spe_nI > spe_integral_limit ? spe_integral_limit : Spe_nI ) ;  //integral 在headfile.h中宏定义
  Spe_nI = ( Spe_nI < -spe_integral_limit ? -spe_integral_limit : Spe_nI ) ;
  /**********速度环输出************************/
  Speed_control_out = Spe_nP + Spe_nI + Spe_nD ;
//		printf("Spe_nP=	%f	",Spe_nP);
//		printf("Spe_nI=	%f	",Spe_nI);
//		printf("Spe_nD=	%f	",Spe_nD);
  /**************************************************/
			//	printf("Speed_control_out_Last=	%f	",Speed_control_out_Last);
			//jh	printf("Speed_control_out=	%f	",Speed_control_out);
  Speed_control_out += Speed_control_out_Last ;
	  		
//				printf("Speed_control_out_Last=	%f	",Speed_control_out_Last);
//				printf("Speed_control_out=	%f	",Speed_control_out);		
  Speed_control_out_Last = Speed_control_out ;

  /***************速度环输出限幅*************/
  Speed_control_out = ( Speed_control_out > spe_output_limit ? spe_output_limit : Speed_control_out ) ;
  Speed_control_out = ( Speed_control_out < 0 ? 0 : Speed_control_out ) ;
	//	printf("Speed_control_out=	%f	",Speed_control_out);
//  printf("Speed_control_out = %f\r\n",Speed_control_out);
////   
//  printf("ultrasonic_distance=	%f\r\n\r\n",ultrasonic_distance);
 

//	
//	
	if(Feedback_Speed < 300)
	{
	  Speed_control_out=650;
	}
		
	 Speed_control_out = ( ultrasonic_distance < 90 ? ultrasonic_distance_spe  : Speed_control_out    ) ;
	Speed_control_out = ( ultrasonic_distance < 42 ? 0  : Speed_control_out    ) ;
					
		
		

  /****调用电机占空比设置函数**************/
	if(ultrasonic_distance < 20 )
	{
			ftm_pwm_duty(ftm0,ftm_ch7,0);     //D7   //电机正转
			ftm_pwm_duty(ftm0,ftm_ch5,250);       //D5
	 }
	else{
				ftm_pwm_duty(ftm0,ftm_ch7,(uint32)Speed_control_out);     //D7   //电机正转
			ftm_pwm_duty(ftm0,ftm_ch5,0);       //D5
	
			}
  /****************************************/
if(0)
{
	mun_1++;
	if(mun_1>4)
	{
		mun_1=0;
			 printf("ultrasonic_distance = %f	",ultrasonic_distance);
			  printf("SpeedPID_Set = %.1f	",SpeedPID_Set);
		 printf("Feedback_Speed = %d	",Feedback_Speed);
		//	
		//		  printf("Speed_error = %f	",Speed_error);
		//	  printf("Spe_nP = %f	",Spe_nP);
		//		
		//		  printf("Spe_nI = %f	",Spe_nI);
		//		  printf("Spe_nD = %f	",Spe_nD);
		//  printf("Speed_error = %f	",Speed_error);
						printf("Speed_control_out=	%f	",Speed_control_out);
				printf("\r\n");
		//	
	}
}	
  return;
}







/********************下列程序已移动至中断中/比例存在问题2019年4月2日20点45分********************************/
//超声波测距    RX是接收端
void ultrasonic(void)     
{
	int j=0;
	int i=0;
	int num_i=0;
	
	uint32 ultrasonic_times[5]={0};//超声波_时间
	uint32 ultrasonic_times_mean=0;
	static uint8 	Ultrasonic_num = 0;//超声波历史计数
	static float Ultrasonic[6]={0};//超声波历史记录

//	*************//
			while(!PAin(9))//当RX（ECHO信号回响）为零时等待
			{	
	//			printf("1");
			PEout(26)=1;          //trig is H   
				systick_delay(10);    //delay 10us
				PEout(26)=0;          //trig  is L   
			}	
				pit_time_start(pit0);	//pit0   开始计时
			while( pit_time_get(pit0) < pit_time_get_limit && PAin(9));		//当RX为1计数并等待//pit_time_get_limit 为超声波计数约束条件
			ultrasonic_times_mean=pit_time_get(pit0);//获取当前pit0计时时间
//			if(pit_time_get(pit0) > pit_time_get_limit ) 
//			{
//					printf("Ultrasonic excess");//若超出约束件则蓝牙反馈已超出
//					return;
//			}
		
			pit_close(pit0);	//pit0停止计时
	ultrasonic_distance=(float)(ultrasonic_times_mean*1.0801)*1.8 * 0.0001 ;    
/*************/
			
			
			//		*************//	下面是滤波算法！若需要，替换上面即可
//		for(num_i = 0; num_i < 5; num_i++) //每个ADC通道获取五次值
//	{

//			while(!PAin(9))//当RX（ECHO信号回响）为零时等待
//			{	
//				printf("1");
//			PEout(26)=1;          //trig is H   
//				systick_delay(10);    //delay 10us
//				PEout(26)=0;          //trig  is L   
//			}	
//				pit_time_start(pit0);	//pit0   开始计时
//			while( pit_time_get(pit0) < pit_time_get_limit && PAin(9));		//当RX为1计数并等待//pit_time_get_limit 为超声波计数约束条件
//			ultrasonic_times[num_i]=pit_time_get(pit0);//获取当前pit0计时时间
////			if(pit_time_get(pit0) > pit_time_get_limit ) 
////			{
////					printf("Ultrasonic excess");//若超出约束件则蓝牙反馈已超出
////					return;
////			}
//		
//			pit_close(pit0);	//pit0停止计时
//	}
//			for(j=0;j<5;j++)  // 冒泡排序
//			{
//					 if(ultrasonic_times[num_i] > ultrasonic_times[num_i+1])        //前面的比后面的大  则进行交换 
//						{
//							ultrasonic_times[num_i] = ultrasonic_times[num_i] ^ ultrasonic_times[num_i+1];
//			        ultrasonic_times[num_i+1] = ultrasonic_times[num_i] ^ ultrasonic_times[num_i+1];
//			        ultrasonic_times[num_i] = ultrasonic_times[num_i] ^ ultrasonic_times[num_i+1];
//						}				 
//			}
//	ultrasonic_times_mean=(ultrasonic_times[1] + ultrasonic_times[2]+ ultrasonic_times[3])/3;  //求平均
//		ultrasonic_distance=(float)(ultrasonic_times_mean*1.0801)*1.8 * 0.0001 ;     //单位是厘米(cm)
//			
//		*************//				
//			
//			
//			
			
			Ultrasonic_num++;
			if(Ultrasonic_num > 0 )
			{
				Ultrasonic_num=0;
			
			for(i = 0; i <5; i++) //记录历史数据
			{
			 Ultrasonic[i]= Ultrasonic[i+1];
			//	printf("Ultrasonic[%d]=%f	",i,Ultrasonic[i]);
			} Ultrasonic[5] = ultrasonic_distance;
			//	printf("Ultrasonic[%d]=%f	",i,Ultrasonic[i]);
		//	printf("\r\n");
			
			//判断历史数据是否是连续减小，若是则判断是横断路障
			//if( Ultrasonic[0] + 1 > Ultrasonic[1] &&  Ultrasonic[1] + 1 > Ultrasonic[2] &&  Ultrasonic[2] + 1 > Ultrasonic[3] &&  Ultrasonic[3] + 1 > Ultrasonic[4] &&  Ultrasonic[4] + 1 > Ultrasonic[5] )
			if( Ultrasonic[0]  > Ultrasonic[1] &&  Ultrasonic[1]  > Ultrasonic[2] &&  Ultrasonic[2]  > Ultrasonic[3] &&  Ultrasonic[3]  > Ultrasonic[4] &&  Ultrasonic[4]  > Ultrasonic[5] )
			{
				printf("lian xu jian xiao\r\n");
					if( ultrasonic_distance < Barricades_distance_limit) //判断是否进入横断路障 终端
					{
						printf("jinru  heng duan lu zhang\r\n");
						enable_irq(PIT2_IRQn);			 
					}
					else
					{
					 disable_irq(PIT2_IRQn);
					}
			}
//			printf("ultrasonic_distance = %f	\r\n",ultrasonic_distance);
			}
			
			
	//	printf("ultrasonic_times_mean= %d\r\n ", ultrasonic_times_mean);

	return;
}


