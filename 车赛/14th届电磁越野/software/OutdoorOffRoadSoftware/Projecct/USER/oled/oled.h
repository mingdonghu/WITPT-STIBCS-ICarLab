#ifndef __OLED_H__
#define __OLED_H__
#include "headfile.h"

void OLED_Clear(void);//���� 
void OLED_Init(void);//��ʼ��OLED  
void OLED_ON(void);//����OLED  
void OLED_OFF(void);//OLED����  
void OLED_Refresh_Gram(void);//�����Դ浽OLED  
 
void OLED_DrawPoint(uint8 x,uint8 y,uint8 t);//����  
void OLED_Fill_s(uint8 x1,uint8 y1,uint8 x2,uint8 y2,uint8 dot);//���  
void OLED_ShowChar(uint8 x,uint8 y,uint8 chr,uint8 size,uint8 mode);//��ʾ�ַ�  
void OLED_ShowNum(uint8 x,uint8 y,uint32 num,uint8 len,uint8 size);//��ʾ2������  
void OLED_ShowString(uint8 x,uint8 y,const uint8 *p,uint8 size);//��ʾ�ַ���


#endif
