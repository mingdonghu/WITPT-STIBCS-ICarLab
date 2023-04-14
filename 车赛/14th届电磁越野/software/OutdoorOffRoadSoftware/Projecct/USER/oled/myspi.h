#ifndef __MYSPI_H__
#define __MYSPI_H__

#include "headfile.h"

#define OLED_CMD 0   //��������  
#define OLED_DATA 1 //�������� 

#define OLED_CLK    PCout(19)    // CLKʱ��    d0  
#define OLED_MOSI   PCout(18)    // MOSI     d1  
#define OLED_RST    PCout(17)    // RET��λ    ret  
#define OLED_DC     PCout(16)    // ����|����    dc  ��0��������1�������ݣ� 

void OLED_SPI_Init(void); //����MCU��SPI  
void SPI_WriteByte(uint8_t addr,uint8_t data); //��Ĵ�����ַдһ��byte������  
void WriteCmd(unsigned char cmd); //д����  
void WriteData(unsigned char data); //д����  

#endif


