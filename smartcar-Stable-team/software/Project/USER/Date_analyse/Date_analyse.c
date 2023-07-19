/******************** (C) COPYRIGHT 2011 ********************* ********************
 * �ļ���       ��Date_analyse.c
 * ����         ��������ݲɼ������
 *
 * ʵ��ƽ̨     ��Ұ��kinetis������
 * ��汾       ��
 * Ƕ��ϵͳ     ��
 *
 * ����         ��o����Σ�o 
**********************************************************************************/   
#include "Date_analyse.h"
#include "headfile.h"
#include "common.h"


/*****************************�����غ�������****************************/
#define   NM    3
#define SECTOR_ADM 253//����
uint16 duty;//���ƫ��ֵ�������ľ�������
#define g_min 6 //�źŶ�ʧ�޶�ֵ

float Dir_Kp_Turn_error_13 = Dir_Kp_Turn_error_1_13;   //1��3�ŵ������
float Dir_Kp_Turn_error_04 = Dir_Kp_Turn_error_1_04;  //0��4�ŵ������
float Dir_Kp_Turn_error_57 = Dir_Kp_Turn_error_1_57;  //5��7�ŵ������
float Dir_Kp_Turn_error_01 = 1;  //5��7�ŵ������  
float Dir_Kp_Turn_error_34 = 1;//5��7�ŵ������

float ultrasonic_distance=0;//������_����

	uint8 i;
	uint8 num=0,mun_1=0;	
	uint16  Count_delay = 0 ;
	uint8 roundabout_num_delayed=0;
	uint8 roundabou_num_first=1;
	uint8 roundabou_num_second=1;
	uint8  flag_area_second_last=0;
//AD��ֵ�˲���
	int16  g_ValueOfAD[10]={0};		//��ȡ�ĵ��ֵ
	int16  g_ValueOfAD_last[10]={0};		//��ȡ�ĵ��ֵ

	uint16 AD_max_123;//��¼123��е����ֵ
	uint8 flag_area_first  =0;//��һ�μ�¼��������ı�־λ
	uint8 flag_area_second=0;//�ڶ��μ�¼��������ı�־λ
	uint16 MIN_v[10] = {0};       //[0]~[7]:������  //��һ����Сֵ
	uint16 MAX_v[10] = {0} ;             //[0]~[7]:������  //��һ�����ֵ
	uint32 roundabou_num=0;

	uint8	flag_roundabou=7;
/****************************/

int16  AD_valu[10],AD_V[10][NM],chazhi,chazhi_old;
float  AD[8],sensor_to_one[10];
float  Slope_AD_1;  // �����µ����
int16  max_v[10],min_v[10];  //��б궨 �ɼ�ֵ
int16  Position_transit[10];  //��¼���ɵ��һ����ֵ

int16  AD_sum[10]; 

int16  position = 2,position_back = 1;
float  max_value,AD_0_max,AD_1_max,AD_2_max,AD_3_max;
int  q=0;//ĳЩѭ���õ�

int16  asd4[4] ,asd5[5],asd_5[5];  
 
 
 
 
void testt(void)//���Ե�ʱ����
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
		printf("FLASH_GetSectorSize=%d\r\n",	 FLASH_GetSectorSize());//��ȡ������С
   FLASH_EraseSector(SECTOR_ADM);//FLASH����253����   SECTOR_ADM == 253
  FLASH_WriteSector(SECTOR_ADM,(const uint8 *)asd5,16,0);		//��������		 
 

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

//��Ŵ�������ʼ��
void EMCS_init(void)
{
  
	
   uint32  valu_get[10] = {0};
//    MAX_v[0] = MAX_v[1] = MAX_v[2] = MAX_v[3] = MAX_v[4] = 0;


	//ADCͨ����ʼ��    
	adc_init(ADC0_SE8);  //B0���ţ�����ͨ�������Ŷ�Ӧ��ϵ���Բ鿴MK60DN10_adc.h
	adc_init(ADC1_SE10); //B4���ţ�����ͨ�������Ŷ�Ӧ��ϵ���Բ鿴MK60DN10_adc.h
	adc_init(ADC0_SE10); //A7���ţ�����ͨ�������Ŷ�Ӧ��ϵ���Բ鿴MK60DN10_adc.h
	adc_init(ADC1_SE12); //B6���ţ�����ͨ�������Ŷ�Ӧ��ϵ���Բ鿴MK60DN10_adc.h
	adc_init(ADC0_SE11); //A8���ţ�����ͨ�������Ŷ�Ӧ��ϵ���Բ鿴MK60DN10_adc.h
	adc_init(ADC1_SE14); //B10���ţ�����ͨ�������Ŷ�Ӧ��ϵ���Բ鿴MK60DN10_adc.h
	adc_init(ADC0_SE17); //E24���ţ�����ͨ�������Ŷ�Ӧ��ϵ���Բ鿴MK60DN10_adc.h
	adc_init(ADC0_SE12); //B2���ţ�����ͨ�������Ŷ�Ӧ��ϵ���Բ鿴MK60DN10_adc.h
	adc_init(ADC1_SE13);//B7���ţ�����ͨ�������Ŷ�Ӧ��ϵ���Բ鿴MK60DN10_adc.h
	adc_init(ADC1_SE17);//A17���ţ�����ͨ�������Ŷ�Ӧ��ϵ���Բ鿴MK60DN10_adc.h
   
	  MAX_v[0]=EMCS_MAX;//�̶�������ֵ ��ͬ
	  MAX_v[1]=EMCS_MAX;
	  MAX_v[2]=EMCS_MAX;
	  MAX_v[3]=EMCS_MAX;
	  MAX_v[4]=EMCS_MAX;
	  MAX_v[5]=EMCS_MAX;
	  MAX_v[6]=EMCS_MAX;
	  MAX_v[7]=EMCS_MAX;
	  MAX_v[8]=EMCS_MAX;
	  MAX_v[9]=EMCS_MAX;
	
//	//��ȡÿ����е����ֵ
//	for( i = 0; i < 5000; i++)
//     { 
//        valu_get[0] = adc_ave(ADC0_SE10, ADC_10bit,50 );     //�ɼ�a7���ŵ�ѹ��������������10λ
//        valu_get[1] = adc_ave(ADC1_SE10, ADC_10bit, 50 );   //�ɼ�B4���ŵ�ѹ��������������10λ
//        valu_get[2] = adc_ave(ADC1_SE14, ADC_10bit, 50 );  //�ɼ�B10���ŵ�ѹ��������������10λ
//        valu_get[3] = adc_ave(ADC0_SE12, ADC_10bit, 50 );//�ɼ�B2�ŵ�ѹ��������������10λ
//        valu_get[4] = adc_ave(ADC0_SE17, ADC_10bit, 50 );		//�ɼ�e24���ŵ�ѹ��������������10λ
//				valu_get[5] = adc_ave(ADC0_SE11, ADC_10bit, 50 );  //�ɼ�A8���ŵ�ѹ��������������10λ
//        valu_get[6] = adc_ave(ADC0_SE8, ADC_10bit, 50 );//�ɼ�B0���ŵ�ѹ��������������10λ
//        valu_get[7] = adc_ave(ADC1_SE12, ADC_10bit, 50 );		//�ɼ�B6���ŵ�ѹ��������������10λ
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
////		 if(key4==0)//����key4���»�ȡ���ֵ
////		 {
////			 	printf("FLASH_GetSectorSize=%d\r\n",	 FLASH_GetSectorSize());//��ӡ������С
////				FLASH_EraseSector(SECTOR_ADM);//FLASH����253����   SECTOR_ADM == 253
////			 FLASH_WriteSector(SECTOR_ADM,(const uint8 *)MAX_v,10,0);		//��������		

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
*  ��������   Read_ADC
*  ����˵���� AD�ɼ�
*  ����˵����         
*  �������أ� ��
*  �޸�ʱ�䣺
*  ��    ע��
*************************************************************************/
void Read_ADC(void)
{		
		int16  i = 0;
    int16  ad_valu[10][5],ad_valuMID[10],ad_sum[10];
//		static int16 ValueOfADOld[5]={0};
		int16 g_ValueOfADFilter[10]={0}; //�����˲���ĵ��ֵ
//		int16 ValueOfADNew[5];  
     float sensor_to_one[10];
     
     for(i=0;i<5;i++)
     {	 
        ad_valu[0][i] = adc_ave(ADC0_SE8,  ADC_10bit, 50 );//�ɼ�B0���ŵ�ѹ��������������10λ
        ad_valu[1][i] = adc_ave(ADC0_SE10, ADC_10bit, 50 );//�ɼ�A7���ŵ�ѹ��������������10λ
        ad_valu[2][i] = adc_ave(ADC1_SE10, ADC_10bit, 50 );//�ɼ�B4���ŵ�ѹ��������������10λ
        ad_valu[3][i] = adc_ave(ADC1_SE14, ADC_10bit, 50 );//�ɼ�B10���ŵ�ѹ��������������10λ
        ad_valu[4][i] = adc_ave(ADC1_SE13, ADC_10bit, 50 );//�ɼ�B7���ŵ�ѹ��������������10λ
				ad_valu[5][i] = adc_ave(ADC0_SE12, ADC_10bit, 50 );//�ɼ�B2���ŵ�ѹ��������������10λ
				ad_valu[6][i] = adc_ave(ADC0_SE11, ADC_10bit, 50 );//�ɼ�A8���ŵ�ѹ��������������10λ
        ad_valu[7][i] = adc_ave(ADC1_SE12, ADC_10bit, 50 );//�ɼ�B6���ŵ�ѹ��������������10λ
				ad_valu[8][i] = adc_ave(ADC0_SE17, ADC_10bit, 50 );//�ɼ�E24���ŵ�ѹ��������������10λ
        ad_valu[9][i] = adc_ave(ADC1_SE17, ADC_10bit, 50 );	//�ɼ�A17���ŵ�ѹ��������������10λ
     }
////	 
//// /**************ð����������***************/
////	 for(i=0;i<5;i++)   //���ADCͨ��
////	 {
////			for(j=0;j<5;j++)  //ÿ��ADCͨ����ȡ���ֵ
////			{
////				 for(k=0;k<5-1-j;k++)
////				 {
////					 if(ad_valu[i][k] > ad_valu[i][k+1])        //ǰ��ıȺ���Ĵ�  ����н���
////						{
////							 ad_valu[i][k] = ad_valu[i][k] ^ ad_valu[i][k+1];
////			        ad_valu[i][k+1] = ad_valu[i][k] ^ ad_valu[i][k+1];
////			        ad_valu[i][k] = ad_valu[i][k] ^ ad_valu[i][k+1];
////						}
////				 }
////			}
////	 }
//		 /*****************************/
     for(i=0;i<10;i++)    //���м�����ĺͣ��������ֵ����Сֵ
      {
        ad_sum[i] = ad_valu[i][1] + ad_valu[i][2] + ad_valu[i][3];
        ad_valuMID[i] = ad_sum[i] / 3;
      }
			//��ӡδ�˲���ֵ
//				for(i=0;i<8;i++)
//		{
//		 	printf("ad_valuMID[%d]=%d	 ",i , ad_valuMID[i] );
//			
//		 }
//		printf ("\r\n");
		

     for(i=0;i<10;i++)            //����ֵ�и�λ������
      {
	    g_ValueOfAD[i] = (int16)(ad_valuMID[i]/10*10);
			g_ValueOfADFilter[i] = g_ValueOfAD[i]; //�������˲����Σ�δ�õ������˲�
	    /***�ɼ��ݶ�ƽ��(�����˲�)***********************************************/
	//    ValueOfADOld[i] = g_ValueOfADFilter[i];  //��¼��һ�ν����˲��ĵ��ֵ
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
    //AD����ֵ��һ������

    for(i=0;i<10;i++)
     {
        sensor_to_one[i] = (float)(g_ValueOfADFilter[i]-MIN_v[i])/(float)(MAX_v[i]-MIN_v[i]);
        if(sensor_to_one[i] <= 0.0)
																	sensor_to_one[i] = 0.001;
        if(sensor_to_one[i] > 1.0)
																	sensor_to_one[i] = 1.0;
        g_ValueOfAD[i] =(uint8) ( 100 * sensor_to_one[i] );  //��һ�����ֵ��0~100֮��
    }
			
//		 			
//AD_judge_limit_min =20
		//AD_judge_limit_max=90
		

	if ( ( g_ValueOfAD[0] >AD_judge_limit_min  || g_ValueOfAD[2] * AD_1_multiple >AD_judge_limit_min || 
g_ValueOfAD[4] >AD_judge_limit_min  ) && (( g_ValueOfAD[1] < AD_judge_limit_max  && g_ValueOfAD[2] * AD_1_multiple < AD_judge_limit_max &&  g_ValueOfAD[3] < AD_judge_limit_max   ))) //AD_judge_limit//�Ƿ���������ж��޶�ֵ
	{
	
//		printf("uH[1]=%d ",g_ValueOfAD[1]);
//		printf("uH[2]=%d : %f ",g_ValueOfAD[2],g_ValueOfAD[2] * AD_1_multiple);
//		printf("uH[3]=%d ",g_ValueOfAD[3]);
//						printf("\r\n");

		//��һ�δ�Χ�������ж�
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
		}		AD_max_123=0;		//���ֵ��λ


//�ڶ��ξ�ȷ6�����ж�

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
		
		
		//�����ж�
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
//															{			// AD_crossroad_limit = 70����567�ŵ��ȫ����AD_crossroad_limitʶ��ʮ��·��
////																	
//														//ʮ��·���ж�
//															flag_area_second=8;
//															//printf("crossroad\r\n");
//															}else//δ��������������ʶ��Ϊ����·��
//																{
//															//	δΪ���������������ı��־λ
//																}
//																
//																
//		
//				
//			//�����ж�
//				 if( roundabou_num_first  && g_ValueOfAD[8] > 45 && ( g_ValueOfAD[1] > AD_roundabout_limit || g_ValueOfAD[3] > AD_roundabout_limit ))//δ�����µ�
//					{
//									roundabout_num_delayed = 0;//ÿ��ʶ�𻷵����ڴ���ʱ ��ֹͬһ�ζ��ʶ�𻷵�
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
//													}else if(g_ValueOfAD[5] < g_ValueOfAD[7] )//�һ���
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

//		//������ʱ
//			roundabout_num_delayed++;
//			if(roundabout_num_delayed > 130)
//			{
//					roundabout_num_delayed = 161;//��ֹ���
//						roundabou_num_first = 1;
//			}

//			Count_delay++;
//			if(Count_delay > 750 )
//			{
//							Count_delay=651;//��ֹ��� ��Count_delayδ������ÿ�ζ���ִ����κ���
////						gpio_set(A16,1);//�ر�С��
////				systick_delay_ms(1000);//��ʱ100ms
////				gpio_set(A16,0);//����С��
//				
//								Dir_Kp_Turn_error_04 = Dir_Kp_Turn_error_1_04;
//								Dir_Kp_Turn_error_01 = 1;
//								Dir_Kp_Turn_error_34 = 1;
//								roundabou_num_second=1;  //���ж���һ�ν�����û���ж��ڶ��γ������򽫵�һ�ν����ж�ʹ��
//			}
//			
//			
//}	
	

	
	if(key29)
{
	if(g_ValueOfAD[5] > AD_crossroad_limit && g_ValueOfAD[7] > AD_crossroad_limit )
															{			// AD_crossroad_limit = 70����567�ŵ��ȫ����AD_crossroad_limitʶ��ʮ��·��
//																	
														//ʮ��·���ж�
															flag_area_second=8;
															//printf("crossroad\r\n");
															}else//δ��������������ʶ��Ϊ����·��
																{
															//	δΪ���������������ı��־λ
																}
																
																
		
				
			//�����ж�
				 if(  g_ValueOfAD[8] > 70 && ( g_ValueOfAD[1] > AD_roundabout_limit || g_ValueOfAD[3] > AD_roundabout_limit ))//δ�����µ�
					{
//									roundabout_num_delayed = 0;//ÿ��ʶ�𻷵����ڴ���ʱ ��ֹͬһ�ζ��ʶ�𻷵�
//									roundabou_num_first = 0;
										flag_area_second = 7 ;

					}
			

		//������ʱ
			roundabout_num_delayed++;
			if(roundabout_num_delayed > 143)
			{
					roundabout_num_delayed = 161;//��ֹ���
						roundabou_num_first = 1;
			}

//			Count_delay++;
//			if(Count_delay > 750 )
//			{
//							Count_delay=651;//��ֹ��� ��Count_delayδ������ÿ�ζ���ִ����κ���
////						gpio_set(A16,1);//�ر�С��
////				systick_delay_ms(1000);//��ʱ100ms
////				gpio_set(A16,0);//����С��
//				
//								Dir_Kp_Turn_error_04 = Dir_Kp_Turn_error_1_04;
//								Dir_Kp_Turn_error_01 = 1;
//								Dir_Kp_Turn_error_34 = 1;
//								roundabou_num_second=1;  //���ж���һ�ν�����û���ж��ڶ��γ������򽫵�һ�ν����ж�ʹ��
//			}
			
			
}	
		 if( roundabou_num_first)
		 {
			//�����ж�����ȷ�����ֵ
					switch ( flag_area_second )
					{

						case 9:  Turn_error = Turn_error_max;break;//�ٽ��                                                                      
						
						
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
									break;//������//2�ŵ��ԭMAX=900���������ܵ����ֵ����200�Ͳ��������ܣ�Ϊ�˼��ݻ�������2�ŵ�зŴ�4.3����δ���������ֵ��
						
					
						case 10:   Turn_error = -Turn_error_max;break; // �ٽ��
						
						case 6: 			;break;
						case 7:  
//							Turn_error= Dir_Kp_Turn_error_57 * (float)(g_ValueOfAD[5]-g_ValueOfAD[7])/(g_ValueOfAD[5]+g_ValueOfAD[7]);
									roundabout_num_delayed = 0;//ÿ��ʶ�𻷵����ڴ���ʱ ��ֹͬһ�ζ��ʶ�𻷵�
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
						
						case 8:	Turn_error = Dir_Kp_Turn_error_57 * (float)(g_ValueOfAD[1]-g_ValueOfAD[3]) /(g_ValueOfAD[1]+g_ValueOfAD[2]+g_ValueOfAD[3]) ;break;//ʮ��·��
						
			
					  default: break;
									
								}
		
					}

//		else{
//			  	Turn_error= Dir_Kp_Turn_error_57 * (float)(g_ValueOfAD[5]-g_ValueOfAD[7])/(g_ValueOfAD[5]+g_ValueOfAD[7]);
//				}
//	

				
 
//�ٶ��趨���ƻ�	
		if(  flag_area_second ==  10||flag_area_second ==  9)
			{
				SpeedPID_Set = SpeedPID_Set_min ;
					
			}	
			else {   //ֱ�������������
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
*Function: ����λ��ʽPD����
*
*  �βζ�Ӧ��ʵ��Ϊȫ�ֱ���
* Turn_error : С����������������ߵ�ƫ��
* Dir_Kp : ���򻷱���ϵ��
* Dir_Kd : ����΢��ϵ��
*��������ʾ����direction_control(&Turn_error,&Dir_Kp,&Dir_Kd)��  
*/
void direction_control(float *Turn_error,uint16 *Dir_Kp,uint16 *Dir_Kd)
{
	static float Turn_error_last = 0;

	float Dir_nP = 0;
    float Dir_nD = 0;
	float Turn_control_out = 0;
	/***���򻷱�������**/
    Dir_nP = (*Dir_Kp)  *(*Turn_error) ; 
    /***����΢�ֿ���***/
    Dir_nD = (*Dir_Kd) * (*Turn_error - Turn_error_last  );
    Turn_error_last =* Turn_error ;
    Turn_control_out = Dir_nP + Dir_nD ;
	//�����������޷�         //�����޷����ݶ��ռ�ձȶ�Ӧ�Ƕȵķ�Χ����
    Turn_control_out = (Turn_control_out>=Turn_error_max ? Turn_error_max : Turn_control_out);
    Turn_control_out = (Turn_control_out<=-Turn_error_max  ? -Turn_error_max : Turn_control_out);//1230 1500 1810

    /***�������������duty = mid+Turn_control_out��mid-Turn_control_out***/

		duty = Direction_Median-Turn_control_out;//��-��>0  Ӧ��ת      С�ڶ����ֵ��ת ���ڶ����ֵ��ת

if(key25)
{
num++;
		if(num > 20|| (flag_area_second_last != flag_area_second))//�������ж��ı��ʱ���ӡ����ֵ
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

  //���ռ�ձ�����	
	ftm_pwm_duty(ftm1,ftm_ch1,(uint32)duty);  
	return;
}




/*
*  �ٶ�PID����
* ����ʵ���� Speed_control(&SpeedPID_Set,&Spe_Kp,&Spe_Ki,&Spe_Kd);
*�����к����β�Ϊָ�룬Ϊ��ʹ�ý�ʵ��ȷ��Ϊȫ�ֱ���������ǰ��������������ȡ������ĵ�ַ
*  SpeedPID_Set �� �ٶ��趨ֵ
*  Spe_Kp  ��   �ٶȻ�����ϵ��
*  Spe_Ki  ��   �ٶȻ�����ϵ��
*  Spe_Kd  ��   �ٶȻ�΢��ϵ��
*/
void Speed_control(float *Speed_Set,float *Spe_Kp,float *Spe_Ki,float *Spe_Kd)
{
	

  static float Speed_error_last = 0 ;  //��һ���ٶ�ƫ��
  static float Speed_error_last_last = 0 ;  //���ϴ��ٶ�ƫ��
  static float  Speed_control_out_Last = 0;
  float Speed_control_out = 0;  //�ٶȻ����
  float Spe_nP = 0;  //������
  float Spe_nI = 0;  //������
  float Spe_nD = 0;  //΢����
  int16 Feedback_Speed = 0;  //�����ٶ�
  float Speed_error = 0;    //�ٶ�ƫ��
	
//printf("Speed_Set=	%f\r\n",SpeedPID_Set);//
  /*��ȡ����ٶ�*/
  Feedback_Speed = ftm_quad_get(ftm2);
  ftm_quad_clean(ftm2);
//	printf("Feedback_Speed=	%d\r\n",Feedback_Speed>>1);//
  /*�����ٶ�ƫ��*/
  Speed_error =  (*Speed_Set) - ( Feedback_Speed>>1);//����ʱ2019��4��26��( Feedback_Speed>>1);�ȼ��� ( Feedback_Speed/2);
  //printf("Speed_Set=	%f	\r\n",*Speed_Set);
	
  /*����ʽPID*/
  Spe_nP = (*Spe_Kp) * (Speed_error- Speed_error_last) ;
//  printf("Speed_error=	%f	",Speed_error);
//	printf("Speed_error_last=	%f	",Speed_error_last);
//	
//  
  Spe_nI = (*Spe_Ki) * Speed_error;
  Spe_nD = (*Spe_Kd) * (  Speed_error - 2 * Speed_error_last + Speed_error_last_last ) ;
  Speed_error_last_last = Speed_error_last ;//��ʷƫ������
  Speed_error_last = Speed_error ;
  
  /*****�ٶȻ������޷�***********************/
  Spe_nI = ( Spe_nI > spe_integral_limit ? spe_integral_limit : Spe_nI ) ;  //integral ��headfile.h�к궨��
  Spe_nI = ( Spe_nI < -spe_integral_limit ? -spe_integral_limit : Spe_nI ) ;
  /**********�ٶȻ����************************/
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

  /***************�ٶȻ�����޷�*************/
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
					
		
		

  /****���õ��ռ�ձ����ú���**************/
	if(ultrasonic_distance < 20 )
	{
			ftm_pwm_duty(ftm0,ftm_ch7,0);     //D7   //�����ת
			ftm_pwm_duty(ftm0,ftm_ch5,250);       //D5
	 }
	else{
				ftm_pwm_duty(ftm0,ftm_ch7,(uint32)Speed_control_out);     //D7   //�����ת
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







/********************���г������ƶ����ж���/������������2019��4��2��20��45��********************************/
//���������    RX�ǽ��ն�
void ultrasonic(void)     
{
	int j=0;
	int i=0;
	int num_i=0;
	
	uint32 ultrasonic_times[5]={0};//������_ʱ��
	uint32 ultrasonic_times_mean=0;
	static uint8 	Ultrasonic_num = 0;//��������ʷ����
	static float Ultrasonic[6]={0};//��������ʷ��¼

//	*************//
			while(!PAin(9))//��RX��ECHO�źŻ��죩Ϊ��ʱ�ȴ�
			{	
	//			printf("1");
			PEout(26)=1;          //trig is H   
				systick_delay(10);    //delay 10us
				PEout(26)=0;          //trig  is L   
			}	
				pit_time_start(pit0);	//pit0   ��ʼ��ʱ
			while( pit_time_get(pit0) < pit_time_get_limit && PAin(9));		//��RXΪ1�������ȴ�//pit_time_get_limit Ϊ����������Լ������
			ultrasonic_times_mean=pit_time_get(pit0);//��ȡ��ǰpit0��ʱʱ��
//			if(pit_time_get(pit0) > pit_time_get_limit ) 
//			{
//					printf("Ultrasonic excess");//������Լ���������������ѳ���
//					return;
//			}
		
			pit_close(pit0);	//pit0ֹͣ��ʱ
	ultrasonic_distance=(float)(ultrasonic_times_mean*1.0801)*1.8 * 0.0001 ;    
/*************/
			
			
			//		*************//	�������˲��㷨������Ҫ���滻���漴��
//		for(num_i = 0; num_i < 5; num_i++) //ÿ��ADCͨ����ȡ���ֵ
//	{

//			while(!PAin(9))//��RX��ECHO�źŻ��죩Ϊ��ʱ�ȴ�
//			{	
//				printf("1");
//			PEout(26)=1;          //trig is H   
//				systick_delay(10);    //delay 10us
//				PEout(26)=0;          //trig  is L   
//			}	
//				pit_time_start(pit0);	//pit0   ��ʼ��ʱ
//			while( pit_time_get(pit0) < pit_time_get_limit && PAin(9));		//��RXΪ1�������ȴ�//pit_time_get_limit Ϊ����������Լ������
//			ultrasonic_times[num_i]=pit_time_get(pit0);//��ȡ��ǰpit0��ʱʱ��
////			if(pit_time_get(pit0) > pit_time_get_limit ) 
////			{
////					printf("Ultrasonic excess");//������Լ���������������ѳ���
////					return;
////			}
//		
//			pit_close(pit0);	//pit0ֹͣ��ʱ
//	}
//			for(j=0;j<5;j++)  // ð������
//			{
//					 if(ultrasonic_times[num_i] > ultrasonic_times[num_i+1])        //ǰ��ıȺ���Ĵ�  ����н��� 
//						{
//							ultrasonic_times[num_i] = ultrasonic_times[num_i] ^ ultrasonic_times[num_i+1];
//			        ultrasonic_times[num_i+1] = ultrasonic_times[num_i] ^ ultrasonic_times[num_i+1];
//			        ultrasonic_times[num_i] = ultrasonic_times[num_i] ^ ultrasonic_times[num_i+1];
//						}				 
//			}
//	ultrasonic_times_mean=(ultrasonic_times[1] + ultrasonic_times[2]+ ultrasonic_times[3])/3;  //��ƽ��
//		ultrasonic_distance=(float)(ultrasonic_times_mean*1.0801)*1.8 * 0.0001 ;     //��λ������(cm)
//			
//		*************//				
//			
//			
//			
			
			Ultrasonic_num++;
			if(Ultrasonic_num > 0 )
			{
				Ultrasonic_num=0;
			
			for(i = 0; i <5; i++) //��¼��ʷ����
			{
			 Ultrasonic[i]= Ultrasonic[i+1];
			//	printf("Ultrasonic[%d]=%f	",i,Ultrasonic[i]);
			} Ultrasonic[5] = ultrasonic_distance;
			//	printf("Ultrasonic[%d]=%f	",i,Ultrasonic[i]);
		//	printf("\r\n");
			
			//�ж���ʷ�����Ƿ���������С���������ж��Ǻ��·��
			//if( Ultrasonic[0] + 1 > Ultrasonic[1] &&  Ultrasonic[1] + 1 > Ultrasonic[2] &&  Ultrasonic[2] + 1 > Ultrasonic[3] &&  Ultrasonic[3] + 1 > Ultrasonic[4] &&  Ultrasonic[4] + 1 > Ultrasonic[5] )
			if( Ultrasonic[0]  > Ultrasonic[1] &&  Ultrasonic[1]  > Ultrasonic[2] &&  Ultrasonic[2]  > Ultrasonic[3] &&  Ultrasonic[3]  > Ultrasonic[4] &&  Ultrasonic[4]  > Ultrasonic[5] )
			{
				printf("lian xu jian xiao\r\n");
					if( ultrasonic_distance < Barricades_distance_limit) //�ж��Ƿ������·�� �ն�
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


