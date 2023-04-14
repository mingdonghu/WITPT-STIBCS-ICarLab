#ifndef __MYIIC_H
#define __MYIIC_H
//#include "sys.h"  // SMT32
#include "common.h"

//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//IIC ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/5/6
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	
   	   		   
//IO��������
//#define SDA_IN()  {GPIOB->MODER&=~(3<<(9*2));GPIOB->MODER|=0<<9*2;}	//PB9����ģʽ
//#define SDA_OUT() {GPIOB->MODER&=~(3<<(9*2));GPIOB->MODER|=1<<9*2;} //PB9���ģʽ
#define SDA_IN()  gpio_ddr(D0,GPI)   //����B11Ϊ���ģʽ	
#define SDA_OUT() gpio_ddr(D0,GPO)  //����B11Ϊ���ģʽ

//IO��������	 
#define IIC_SCL    PDout(2) //SCL
#define IIC_SDA    PDout(0) //SDA	 
#define READ_SDA   PDin(0)  //����SDA 
#define   systick_delay_us_iic   systick_delay_us(180);//IIC��ʱ����
//IIC���в�������
void IIC_Init(void);                //��ʼ��IIC��IO��				 
void IIC_Start(void);				//����IIC��ʼ�ź�
void IIC_Stop(void);	  			//����IICֹͣ�ź�
void IIC_Send_Byte(uint8 txd);			//IIC����һ���ֽ�
uint8 IIC_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽ�
uint8 IIC_Wait_Ack(void); 				//IIC�ȴ�ACK�ź�
void IIC_Ack(void);					//IIC����ACK�ź�
void IIC_NAck(void);				//IIC������ACK�ź�

void IIC_Write_One_Byte(uint8 daddr,uint8 addr,uint8 data);
uint8 IIC_Read_One_Byte(uint8 daddr,uint8 addr);	  
#endif
















