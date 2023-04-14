#include <Wire.h>
#include <avr/io.h>
#include <avr/interrupt.h>
//A4(SDA)  A5(SCL)
char Res;
char usart_rxcmd_flag;
char USART_RX_BUF[16];
char usart_rxcmd_cnt;
char usart_rx_timeout;

unsigned char ok_flag;
unsigned char fail_flag;

unsigned short lenth_val = 0;
unsigned char i2c_rx_buf[16];
unsigned char dirsend_flag=0;
int serial_putc( char c, struct __file * )
{
  Serial.write( c );
  return c;
}
void printf_begin(void)
{
  fdevopen( &serial_putc, 0 );
}

ISR(TIMER1_COMPA_vect)
{
   digitalWrite(7, !digitalRead(7)); 
  if(usart_rx_timeout>0)
  {
    usart_rx_timeout--;
    if(usart_rx_timeout==0)
    {
    }
  }
}

void SensorWrite(unsigned char addr,unsigned char* datbuf,unsigned char cnt) 
{
  unsigned short result=0;
  // step 1: instruct sensor to read echoes
  Wire.beginTransmission(82); // transmit to device #82 (0x52)
  // the address specified in the datasheet is 164 (0xa4)
  // but i2c adressing uses the high 7 bits so it's 82
  Wire.write(byte(addr));      // sets distance data address (addr)
  // step 2: wait for address to write
  delay(1);                   // datasheet suggests at least 30uS
  // step 3: send data to sensor
  Wire.write(datbuf, cnt);    // write cnt bytes to slave device
  Wire.endTransmission();      // stop transmitting
}
void SensorRead(unsigned char addr,unsigned char* datbuf,unsigned char cnt) 
{
  unsigned short result=0;
  // step 1: instruct sensor to read echoes
  Wire.beginTransmission(82); // transmit to device #82 (0x52)
  // the address specified inx the datasheet is 164 (0xa4)
  // but i2c adressing uses the high 7 bits so it's 82
  Wire.write(byte(addr));      // sets distance data address (addr)
  Wire.endTransmission();      // stop transmitting
  // step 2: wait for readings to happen
  delay(1);                   // datasheet suggests at least 30uS
  // step 3: request reading from sensor
  Wire.requestFrom(82, cnt);    // request cnt bytes from slave device #82 (0x52)
  // step 5: receive reading from sensor
  if (cnt <= Wire.available()) { // if two bytes were received
    *datbuf++ = Wire.read();  // receive high byte (overwrites previous reading)
    *datbuf++ = Wire.read(); // receive low byte as lower 8 bits
  }
}
void setup() {
  // put your setup code here, to run once:
  Wire.begin();        // join i2c bus (address optional for master)
  pinMode(6,OUTPUT);
  pinMode(7,OUTPUT);
  digitalWrite(6,HIGH);
  digitalWrite(7,HIGH);
  Serial.begin(9600,SERIAL_8N1); 
  printf_begin();
  printf("start\n\r");
  // ��ʼ��timer1
  cli();          // ����ȫ���ж�
  TCCR1A = 0;     //����TCCR1AȫΪ0
  TCCR1B = 0;     // [size=1em]����TCCR1BȫΪ0

  //���ñȽϵ�ֵ�ô�С
  OCR1A = 15;
  // ����CTCģʽ
  TCCR1B |= (1 << WGM12);
  // ���� CS10��CS12λ
  TCCR1B |= (1 << CS10);
  TCCR1B |= (1 << CS12);
  // ����Ƚ��ж�
  TIMSK1 |= (1 << OCIE1A);
  sei();          // ����ȫ���ж�
  printf("s5-1#");
}
void loop() {
  // put your main code here, to run repeatedly:
  //digitalWrite(12,LOW);
  //digitalWrite(13,LOW);
 
  //if(dirsend_flag)
  {
    SensorRead(0x00,i2c_rx_buf,2);
    lenth_val=i2c_rx_buf[0];
    lenth_val=lenth_val<<8;
    lenth_val|=i2c_rx_buf[1];
    printf("i2c=%dmm\n",lenth_val);// print the lenth_val
    delay(300); 
    if(lenth_val>110)
    {
      if(dirsend_flag==0)
      {
        dirsend_flag=1;
        SensorWrite(0x0e,&dirsend_flag,1);
        printf("dir=%d\n",dirsend_flag);// print the dirsend_flag
        delay(300);
      }
    }
    else if(lenth_val<90)
    {
      if(dirsend_flag==1)
      {
        dirsend_flag=0;
        SensorWrite(0x0e,&dirsend_flag,1);
        printf("dir=%d\n",dirsend_flag);// print the dirsend_flag
        delay(300);
      }
    }
  }
}
