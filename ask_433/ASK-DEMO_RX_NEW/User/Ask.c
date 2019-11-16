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
extern FM24C_Data_S fm24c_data;

unsigned char in_bit = 0;
unsigned char rx_start = 0;
unsigned char rx_data_ok = 0;

unsigned char recvbit[4];
unsigned char recvbitcount = 0;

unsigned char recvbyte[64];
unsigned char recvbytecount = 0;
unsigned char Recv_data[ASK_SEND_LEN];

unsigned char in_bit_n = 0;

unsigned char pwm_duty=0;
unsigned char instNum=0;
FM24C_Data_S EE_dev_data;

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
        delay_ms(100);
        Led_off(1);
        delay_ms(100);
      }
    }
  }
}
///////////////////���պ���///////////////////////
void Recieve()
{
  //һ�������Ȱ����ŵ�״̬��ȡ�ˣ�Ȼ���жϸ�ǰ����Ƿ�һ������һ����ʱ��Ž��к�������
  in_bit_n = inport;    //inport��ASKģ������ݽ�
  if(in_bit == in_bit_n)
  {
    return;
  }
  in_bit = in_bit_n;
  //P3_7 = in_bit;   //��ֵ����LED��
  if(timer_4_countover)
  {//��ʱ����
    RecieveError();
    return; 
  }
  // ����4 �ε�ƽ�仯������ȷ��1 bit
  if((timer_4_count > min_time_l)&&(timer_4_count < max_time_l))
  { //խ����,4~14,����200us~700us
    if(in_bit) //�ߵ�ƽ,����Ϊ�ߵ�ƽ����ʵ֮ǰ�ǵ͵�ƽ��
    {
      recvbit[recvbitcount] = 0x00;   //�Ͷ�
    }
    else    //�͵�ƽ
    {
      recvbit[recvbitcount] = 0x01;   //�߶�
    }
  }
  else if((timer_4_count > min_time_h)&&(timer_4_count < max_time_h))
  { //�����壬16~60������800us~3000us
    if(in_bit)
    {
      recvbit[recvbitcount] = 0x02;    //�ͳ�
    }
    else
    {
      recvbit[recvbitcount] = 0x03;   //�߳�
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
  recvbytecount++;      //���յ����ֽ�����1��
  recvbitcount = 0;     //
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
    }

    Uart_Senddata(Recv_data,ASK_SEND_LEN);
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
  uint8_t i;
  
  EE_dev_data.assoAddr[0]= EEPROM_Byte_Read(EE_ADDR0);
  EE_dev_data.assoAddr[1]= EEPROM_Byte_Read(EE_ADDR1);
  EE_dev_data.instNum = EEPROM_Byte_Read(EE_ADDR_InsNum);
  pwm_duty = EEPROM_Byte_Read(EE_Duty);

  EE_dev_data.dev[0].devType = EEPROM_Byte_Read(EE_ADDR_DevType);
  EE_dev_data.dev[0].devAddr = EEPROM_Byte_Read(EE_ADDR_DevAddr);

  for(i=0;i<8;i++)
  {
    EE_dev_data.dev[0].key[i].keyType = EEPROM_Byte_Read(EE_ADDR_KeyVal+i*4);
    EE_dev_data.dev[0].key[i].scene = EEPROM_Byte_Read(EE_ADDR_KeyVal+i*4+2);
  }
}

///////////////////������뺯��///////////////////////
void Dele_Sender()
{
  uint8_t i;
  EEPROM_Byte_Write(EE_ADDR0, 0x00);
  EEPROM_Byte_Write(EE_ADDR1, 0x00);

  EEPROM_Byte_Write(EE_ADDR_DevType, 0x00);
  EEPROM_Byte_Write(EE_ADDR_DevAddr, 0x00);

  for(i=0;i<8;i++)
  {
    EEPROM_Byte_Write(EE_ADDR_KeyVal+i, 0x00);
  }
  
  ReadSelfAddr();
}
///////////////////���������///////////////////////
void ProcessOut()
{
  uint8_t i;

  #ifdef READ_REALTIME
  FM24C_ReadDevInfo(0);
  //ƥ��ID���豸��ַ
  if((Recv_data[0]==fm24c_data.assoAddr[0])&&
     (Recv_data[1]==fm24c_data.assoAddr[1])/*&&
     (Recv_data[2]==fm24c_data.dev[0].devAddr)*/)
  {
    for(i=0;i<8;i++)
    {
      if(Recv_data[3]==fm24c_data.dev[0].key[i].keyType)
      {
        switch(Recv_data[3])//KEY Action
        {
            case LIGHT_OFF:
                Set_Pwm(0);//0-256
                Led_on(1);
                delay_ms(100);
                Led_off(1);
                delay_ms(100);
                break;
            case LIGHT_ON:
                Set_Pwm(pwm_duty);//0-256
                Led_on(2);
                delay_ms(100);
                Led_off(2);
                delay_ms(100);
                break;
            case LIGHT_UP:
                pwm_duty = pwm_duty>=255?255:pwm_duty++;
                Set_Pwm(pwm_duty);//0-256
                Led_on(3);
                delay_ms(100);
                Led_off(3);
                delay_ms(100);
                break;
            case LIGHT_DOWN:
                pwm_duty = pwm_duty<=10?10:pwm_duty--;
                Set_Pwm(pwm_duty);//0-256
                Led_on(4);
                delay_ms(100);
                Led_off(4);
                delay_ms(100);
                break;
            case SCENE_1:
                Set_Pwm(fm24c_data.dev[0].key[i].scene);
                break;
            case SCENE_2:
                Set_Pwm(fm24c_data.dev[0].key[i].scene);
                break;
            case SCENE_3:
                Set_Pwm(fm24c_data.dev[0].key[i].scene);
                break;
            case SCENE_4:
                Set_Pwm(fm24c_data.dev[0].key[i].scene);
                break;
            default:
                break;
        }
      }
    }
  }
  #else
  //ƥ��ID���豸��ַ
  if((Recv_data[0]==EE_dev_data.assoAddr[0])&&
     (Recv_data[1]==EE_dev_data.assoAddr[1])&&
     (Recv_data[2]==EE_dev_data.dev[0].devAddr))
  {

    //key = (Recv_data[3]>>4)&0x0f; //KEY Type

    for(i=0;i<8;i++)
    {
      if(Recv_data[3]==EE_dev_data.dev[0].key[i].keyType)
      {
        switch(EE_dev_data.dev[0].key[i].keyType)//KEY Action
        {
            case LIGHT_OFF:
                Set_Pwm(0);//0-256
                Led_on(1);
                delay_ms(100);
                Led_off(1);
                delay_ms(100);
                break;
            case LIGHT_ON:
                Set_Pwm(pwm_duty);//0-256
                Led_on(2);
                delay_ms(100);
                Led_off(2);
                delay_ms(100);
                break;
            case LIGHT_UP:
                pwm_duty = pwm_duty>=255?255:pwm_duty++;
                Set_Pwm(pwm_duty);//0-256
                Led_on(3);
                delay_ms(100);
                Led_off(3);
                delay_ms(100);
                break;
            case LIGHT_DOWN:
                pwm_duty = pwm_duty<=10?10:pwm_duty--;
                Set_Pwm(pwm_duty);//0-256
                Led_on(4);
                delay_ms(100);
                Led_off(4);
                delay_ms(100);
                break;
            case SCENE_1:
                Set_Pwm(EE_dev_data.dev[0].key[i].scene);
                break;
            case SCENE_2:
                Set_Pwm(EE_dev_data.dev[0].key[i].scene);
                break;
            case SCENE_3:
                Set_Pwm(EE_dev_data.dev[0].key[i].scene);
                break;           
            case SCENE_4:
                Set_Pwm(EE_dev_data.dev[0].key[i].scene);
                break;
            default:
                break;
        }
      }
    }
  }
  #endif
}
///////////////////������뺯��///////////////////////
void Write_Coder()
{
  uint8_t i,j;

  EEPROM_EREASE();

  delay_ms(1);
  EEPROM_Byte_Write(EE_ADDR0, fm24c_data.assoAddr[0]);
  delay_ms(1);
  EEPROM_Byte_Write(EE_ADDR1, fm24c_data.assoAddr[1]);
  delay_ms(1);
  EEPROM_Byte_Write(EE_ADDR_InsNum, fm24c_data.instNum);

  for(j=0;j<MAX_DEV_NUM;j++)
  {
    delay_ms(1);
    EEPROM_Byte_Write(EE_ADDR_DevAddr+j*0x10, fm24c_data.dev[j].devAddr);
  
    for(i=0;i<8;i++)
    {
      delay_ms(1);
      EEPROM_Byte_Write(EE_ADDR_KeyVal+j*0x10+i*4, fm24c_data.dev[j].key[i].keyType);
      EEPROM_Byte_Write(EE_ADDR_KeyVal+j*0x10+i*4+2, fm24c_data.dev[j].key[i].scene);
    }
  }
}


