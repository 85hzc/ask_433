/*********************************************************
//˵����ASK���߽�����򣬽�EV1527���룬��С����Ϊ300~500us��ʹ��ǰ��Ҫ���룬����һ�°������ɿ���
//      4��LEDȫ����Ϩ��ֻ��LED2��LED3������5s���ڰ���ң���������Է���LED��������˸���κ�Ϩ�𼴿���ɶ��롣
//      �Ժ��ң�����Ϳ��ԶԸ�ASK_Demo�����ң�ء���ASK_Demo���յ������źź���������ڣ�Ȼ���ж�ID��ok�������Ӧ��LED��100ms��Ϩ��     
//��Ƭ����STM8S003F3P6
//�����ڲ�16Mhz
//���ߣ��ٿ�ͬѧ��2168916131@qq.com
//ʱ�䣺20170708
***********************************************************/
#include "All_Includes.h"

extern unsigned char timer_4_count;
extern unsigned char timer_4_countover;

unsigned char in_bit = 0;       
unsigned char rx_start = 0;     
unsigned char rx_data_ok = 0;

unsigned char recvbit[4];
unsigned char recvbitcount = 0;

unsigned char recvbyte[40];
unsigned char recvbytecount = 0;
unsigned char Recv_data[5];

unsigned char in_bit_n = 0;       

unsigned char SelfAddr[2]={0, 0};
///////////////////ASK��ʼ������///////////////////////
void Ask_Init()
{
  PD_DDR_DDR3=0;        //KEY1����
  PD_CR1_C13=0;         //��������
  PD_CR2_C23=0;         //��ֹ�ⲿ�ж�
}
///////////////////ASK������///////////////////////
void Ask_process()
{
  unsigned char key_value=0,i;
  Ask_Init();   //ASK��ʼ��
  ReadSelfAddr();   //��eeprom��洢��ID
  
  while(1)
  {

    ProcessRecv();      //������պ���
    
    key_value=key_scan();
    if(key_value==0x01)
    {
        //����
      Led_on(1);
      Learn_Sender();   
      Led_off_all();
    }
    else if(key_value==0x02)
    {
        //ɾ������
      Dele_Sender();
      
      for(i=0; i<5; i++)
      {//ɾ����ɣ������ָʾ
        Led_on(1);
        //Led_on(2);
        //Led_off(3);
        //Led_off(4);
        delay_ms(100);
        Led_off(1);
        //Led_off(2);
        //Led_on(3);
        //Led_on(4);
        delay_ms(100);
      }
    }
  }
}
///////////////////���պ���///////////////////////
void Recieve()
{
  //һ�������Ȱ����ŵ�״̬��ȡ�ˣ�Ȼ���жϸ�ǰ����Ƿ�һ������һ����ʱ��Ž��к�������
  in_bit_n = inport;	//inport��ASKģ������ݽ�
  if(in_bit == in_bit_n)	
  {
    return;
  }
  in_bit = in_bit_n;
  //P3_7 = in_bit;	 //��ֵ����LED��
  if(timer_4_countover)
  {//��ʱ����
	RecieveError();
	return;	
  }
  // ����4 �ε�ƽ�仯������ȷ��1 bit
  if((timer_4_count > min_time_l)&&(timer_4_count < max_time_l)) 
  {	//խ����,4~14,����200us~700us
    if(in_bit) //�ߵ�ƽ,����Ϊ�ߵ�ƽ����ʵ֮ǰ�ǵ͵�ƽ��
    {
	recvbit[recvbitcount] = 0x00;	//�Ͷ�
    }
    else	//�͵�ƽ
    {
	recvbit[recvbitcount] = 0x01;	//�߶�
    }
  }
  else if((timer_4_count > min_time_h)&&(timer_4_count < max_time_h))
  {	//�����壬16~60������800us~3000us
    if(in_bit)
    {
	recvbit[recvbitcount] = 0x02;	 //�ͳ�
    }
    else
    {
	recvbit[recvbitcount] = 0x03;	//�߳�
    }
  }
  else
  {//����
	RecieveError();
	return;
  }
  timer_4_count = 0;
  timer_4_countover = 0;

 // 1527	
  recvbitcount++;
  if(recvbitcount < 2) 
  {
	return;
  }
  else
  {
    //�����жϵĵ�ƽ��Ӧ���Ǹ�ʵ�ʵ��෴�ģ���Ϊֻ�е�ƽ�仯�ˣ��Ż�����Ӧ�������仯�Ļ���ֱ���˳��ġ�
    if((recvbit[0] == 1)&&(recvbit[1] == 2))   //�߶̵ͳ�
    {
	recvbyte[recvbytecount] = 0;
    }
    else if((recvbit[0] == 3)&&(recvbit[1] == 0))  //�߳��Ͷ�
    {
	recvbyte[recvbytecount] = 1;
    }
    else
    {
	RecieveError();
	return;
    }		
  }
  recvbytecount++;	 	//���յ����ֽ�����1��
  recvbitcount = 0;		//
  if(recvbytecount < RECV_BIT_NUMBER)
  {// δ������
    return;
  }
  recvbytecount = 0;
  timer_4_count = 0;
  rx_data_ok = 1;
}
///////////////////���մ�����///////////////////////
void RecieveError()
{
  rx_start = 0;
  rx_data_ok = 0;

  recvbitcount = 0;
  recvbytecount = 0;
	
  timer_4_count = 0;
  timer_4_countover = 0;

}
///////////////////������պ���///////////////////////
void ProcessRecv()
{
  unsigned char i,j;
  unsigned char p=0;
  unsigned char temp;
  
  Recieve();

  if(rx_data_ok)
  {
    rx_data_ok = 0;
    for(i=0;i<RECV_BIT_NUMBER;i=i+8)
    {//�ں�
      temp=0;
      for(j=i;j<(i+8);j++)
      {
        temp += (recvbyte[j]<<(7-(j-i)));
      }
      Recv_data[p++]=temp;
      //UART0_TX(temp);
    }
    Uart_Sendbyte(Recv_data[0]);
    Uart_Sendbyte(Recv_data[1]);
    Uart_Sendbyte(Recv_data[2]);
    Uart_Sendbyte(SelfAddr[0]);
    Uart_Sendbyte(SelfAddr[1]);
    ProcessOut();
  }
  else 
  {

  }
}

//////////////////���뺯��///////////////////
void Learn_Sender()
{
  unsigned long LearnDelay;
  unsigned char i,j;
  unsigned char p=0;
  unsigned char temp;
  LearnDelay = GetTimer();
  rx_data_ok = 0;
  while(1)
  {//5���Ӷ���ʱ��
    //WDT_CountClear();
    Recieve();
    if(rx_data_ok)
    {
        rx_data_ok = 0;

        EEPROM_EREASE();
        for(i=0;i<RECV_BIT_NUMBER;i=i+8)
        {//�ں�
          temp=0;
          for(j=i;j<(i+8);j++)
          {
            temp += (recvbyte[j]<<(7-(j-i)));
          }
          Recv_data[p++]=temp;
        }
        Uart_Sendbyte(Recv_data[0]);
        Uart_Sendbyte(Recv_data[1]);
        Uart_Sendbyte(Recv_data[2]);
        EEPROM_Byte_Write(EE_ADDR0, Recv_data[0]);
        delay_ms(1);
        EEPROM_Byte_Write(EE_ADDR1, Recv_data[1]);

        ReadSelfAddr();
        for(i=0; i<10; i++)
        {//������ɣ������ָʾ
          Led_on(1);
          //Led_on(2);
          //Led_off(3);
          //Led_off(4);
          delay_ms(100);
          Led_off(1);
          //Led_off(2);
          //Led_on(3);
          //Led_on(4);
          delay_ms(100);
        }
        //EX1 = 1;
        return;
    }
    if(SpanTime(LearnDelay) > 5000)
        return;
  }
}

///////////////////��ȡID����///////////////////////
void ReadSelfAddr()
{
  SelfAddr[0]= EEPROM_Byte_Read(EE_ADDR0);
  SelfAddr[1]= EEPROM_Byte_Read(EE_ADDR1);
  
  Uart_Sendbyte(SelfAddr[0]);
  Uart_Sendbyte(SelfAddr[1]);  
}

///////////////////������뺯��///////////////////////
void Dele_Sender()
{
  EEPROM_Byte_Write(EE_ADDR0, 0x00);
  EEPROM_Byte_Write(EE_ADDR1, 0x00);
  ReadSelfAddr();
}
///////////////////���������///////////////////////
void ProcessOut()
{
  if((Recv_data[0]==SelfAddr[0])&&(Recv_data[1]==SelfAddr[1]))
  {//ƥ��ID
    switch(Recv_data[2]&0x0f)
    {
        case 0x01:
            Led_on(1);
            delay_ms(100);
            Led_off(1);
            delay_ms(100);
            break;
        case 0x02:
            Led_on(2);
            delay_ms(100);
            Led_off(2);
            delay_ms(100);
            break;
        case 0x04:
            Led_on(3);
            delay_ms(100);
            Led_off(3);
            delay_ms(100);
            break;
        case 0x08:
            Led_on(4);
            delay_ms(100);
            Led_off(4);
            delay_ms(100);
            break;
        default:
            break;
    }
  }
}
///////////////////������뺯��///////////////////////
void Write_Coder(unsigned char a,unsigned char b)
{
  EEPROM_EREASE();
  delay_ms(1);
  Uart_Sendbyte(3);

  Uart_Sendbyte(a);
  Uart_Sendbyte(b);
  
  EEPROM_Byte_Write(EE_ADDR0, a);
  delay_ms(1);
  EEPROM_Byte_Write(EE_ADDR1, b);
  
  Uart_Sendbyte(a);
  Uart_Sendbyte(b);
}


