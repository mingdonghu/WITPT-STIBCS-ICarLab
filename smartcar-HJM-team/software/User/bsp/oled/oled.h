#ifndef __OLED_H__
#define __OLED_H__
#include "include.h"



void OLED_Init(void);//初始化OLED  
void OLED_ON(void);//唤醒OLED  
void OLED_OFF(void);//OLED休眠  
void OLED_Refresh_Gram(void);//更新显存到OLED  
void OLED_Clear(void);//清屏  
void OLED_DrawPoint(uint8 x,uint8 y,uint8 t);//画点  
void OLED_Fill(uint8 x1,uint8 y1,uint8 x2,uint8 y2,uint8 dot);//填充  
void OLED_ShowChar(uint8 x,uint8 y,uint8 chr,uint8 size,uint8 mode);//显示字符  
void OLED_ShowNum(uint8 x,uint8 y,uint32 num,uint8 len,uint8 size);//显示2个数字  
void OLED_ShowString(uint8 x,uint8 y,const uint8 *p,uint8 size);//显示字符串


#endif