/*********************************************************
//说明：ASK无线解码程序，解EV1527编码，最小脉宽为300~500us。使用前需要对码，即按一下按键后松开，
//      4个LED全亮后熄灭只留LED2，LED3常亮，5s钟内按下遥控器，可以发现LED分两组闪烁三次后熄灭即可完成对码。
//      以后该遥控器就可以对该ASK_Demo板进行遥控。该ASK_Demo板收到无线信号后，先输出串口，然后判断ID，ok后点亮对应的LED，100ms后熄灭。     
//单片机：STM8S003F3P6
//晶振：内部16Mhz
//作者：少凯同学，2168916131@qq.com
//时间：20170708
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

///////////////////ASK初始化函数///////////////////////
void Ask_Init()
{
  PD_DDR_DDR3=0;        //KEY1输入
  PD_CR1_C13=0;         //悬浮输入
  PD_CR2_C23=0;         //禁止外部中断
}
///////////////////ASK处理函数///////////////////////
void Ask_process()
{
  unsigned char key_value=0,i;

  Ask_Init();   //ASK初始化
  ReadSelfAddr();   //读eeprom里存储的ID

  while(1)
  {
    ProcessRecv();      //处理接收函数

    key_value=key_scan();
    if(key_value==0x01)
    {
        //对码
      Led_on(1);
      Learn_Sender();
      Led_off_all();
    }
    else if(key_value==0x02)
    {
      //删除对码
      Dele_Sender();

      for(i=0; i<5; i++)
      {//删码完成，跑马灯指示
        Led_on(1);
        delay_ms(100);
        Led_off(1);
        delay_ms(100);
      }
    }
  }
}
///////////////////接收函数///////////////////////
void Recieve()
{
  //一进来就先把引脚的状态读取了，然后判断跟前面的是否一样，不一样的时候才进行后续运算
  in_bit_n = inport;    //inport是ASK模块的数据脚
  if(in_bit == in_bit_n)
  {
    return;
  }
  in_bit = in_bit_n;
  //P3_7 = in_bit;   //把值丢给LED口
  if(timer_4_countover)
  {//超时错误
    RecieveError();
    return; 
  }
  // 接收4 次电平变化，才能确定1 bit
  if((timer_4_count > min_time_l)&&(timer_4_count < max_time_l))
  { //窄脉冲,4~14,就是200us~700us
    if(in_bit) //高电平,现在为高电平，其实之前是低电平的
    {
      recvbit[recvbitcount] = 0x00;   //低短
    }
    else    //低电平
    {
      recvbit[recvbitcount] = 0x01;   //高短
    }
  }
  else if((timer_4_count > min_time_h)&&(timer_4_count < max_time_h))
  { //宽脉冲，16~60，就是800us~3000us
    if(in_bit)
    {
      recvbit[recvbitcount] = 0x02;    //低长
    }
    else
    {
      recvbit[recvbitcount] = 0x03;   //高长
    }
  }
  else
  {//出错
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
    //这里判断的电平，应该是跟实际的相反的，因为只有电平变化了，才会做相应处理，不变化的话是直接退出的。
    if((recvbit[0] == 1)&&(recvbit[1] == 2))   //高短低长
    {
      recvbyte[recvbytecount] = 0;
    }
    else if((recvbit[0] == 3)&&(recvbit[1] == 0))  //高长低短
    {
      recvbyte[recvbytecount] = 1;
    }
    else
    {
      RecieveError();
      return;
    }
  }
  recvbytecount++;      //接收到的字节数加1。
  recvbitcount = 0;     //
  if(recvbytecount < RECV_BIT_NUMBER)
  {// 未接收完
    return;
  }
  recvbytecount = 0;
  timer_4_count = 0;
  rx_data_ok = 1;
}
///////////////////接收错误函数///////////////////////
void RecieveError()
{
  rx_start = 0;
  rx_data_ok = 0;

  recvbitcount = 0;
  recvbytecount = 0;
    
  timer_4_count = 0;
  timer_4_countover = 0;

}
///////////////////处理接收函数///////////////////////
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
    {//融合
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

//////////////////对码函数///////////////////
void Learn_Sender()
{
  unsigned long LearnDelay;
  unsigned char i,j;
  unsigned char p=0;
  unsigned char temp;
  LearnDelay = GetTimer();
  rx_data_ok = 0;
  while(1)
  {//5秒钟对码时间
    //WDT_CountClear();
    Recieve();
    if(rx_data_ok)
    {
        rx_data_ok = 0;

        EEPROM_EREASE();
        for(i=0;i<RECV_BIT_NUMBER;i=i+8)
        {//融合
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
        {//对码完成，跑马灯指示
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

///////////////////读取ID函数///////////////////////
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

///////////////////清除对码函数///////////////////////
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
///////////////////输出处理函数///////////////////////
void ProcessOut()
{
  uint8_t i;

  #ifdef READ_REALTIME
  FM24C_ReadDevInfo(0);
  //匹配ID和设备地址
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
  //匹配ID和设备地址
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
///////////////////清除对码函数///////////////////////
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


