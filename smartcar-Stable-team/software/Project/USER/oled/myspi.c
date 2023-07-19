#include "myspi.h"

/*��������: GPIOģ��SPI�˿ڳ�ʼ��                                         */  
void OLED_SPI_Init(void)  
{
/*  
    GPIO_InitTypeDef GPIO_InitStructure;  
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA ,ENABLE);//ʹ��PA�˿�ʱ��  
    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4; //�˿�����  
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;//�������  
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;//IO���ٶ�Ϊ50MHz  
    GPIO_Init(GPIOA,&GPIO_InitStructure);//�����趨������ʼ��GPIOA  
*/
	
  gpio_init(C19,GPO,1);//d0
  gpio_init(C18,GPO,1);//d1
  gpio_init(C17,GPO,1);//res
  gpio_init(C16,GPO,1);//dc
  
} 

/*��������: ͨ��SPIO���ģ��SPIͨ��Э��,��ģ��(SSD1306)д��һ���ֽ�        */ 
void SPI_WriteByte(unsigned char data,unsigned char cmd)  
{

    unsigned char i=0;  
    OLED_DC =cmd;  
    OLED_CLK=0;  
    for(i=0;i<8;i++)  
    {  
        OLED_CLK=0;  
        if(data&0x80)OLED_MOSI=1; //�Ӹ�λ����λ  
        else OLED_MOSI=0;  
        OLED_CLK=1;  
        data<<=1;  
    }  
    OLED_CLK=1;  
    OLED_DC=1;  

} 

/*��������: д����                                                        */  
void WriteCmd(unsigned char cmd)  
{  
    SPI_WriteByte(cmd,OLED_CMD);  
} 

/*��������: д����                                                        */  
void WriteData(unsigned char data)  
{  
    SPI_WriteByte(data,OLED_DATA);  
}  















