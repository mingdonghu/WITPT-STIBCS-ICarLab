#ifndef _Date_analyse_h
#define _Date_analyse_h



#include "headfile.h"
#include "misc.h"
#include "common.h"
#include "MK60_port.h"
#include "MK60_gpio.h"
#include "MK60_port_cfg.h"


/*****ȫ�ֱ���������***************/
//���±��������main�����ж��壬Ϊ����������.c��Ӧ���ڴ˽���ȫ������
//Date_analyse.h�з�����ر���������
extern float Turn_error;
extern float Turn_error_later;
extern uint16 Dir_Kp;
extern uint16 Dir_Kd;
extern float	RightAngle_Dir_Kp;
extern int Spe_gears;
extern uint16 SpeedPID_Set_max;
extern uint16 SpeedPID_Set_min;
extern float Spe_Kp; //�ٶȻ�����ϵ��
extern float Spe_Ki; //�ٶȻ�����ϵ��
extern float Spe_Kd; //�ٶȻ�΢��ϵ��
extern float SpeedPID_Set;  //�ٶ��趨ֵ

			
/***********************************/


void EMCS_init(void);//��Ŵ�������ʼ��
void SC_black_Init(void);// ���ֵ����
void Read_ADC(void);//��ȡ���ֵ
void Date_analyse(void);
void testt(void);//�����ú���
void ultrasonic(void) ;  
void Speed_control(float *Speed_Set,float *Spe_Kp,float *Spe_Ki,float *Spe_Kd); //�ٶȻ�


void direction_control(float *Turn_error,uint16 *Dir_Kp,uint16 *Dir_Kd);//


#endif
