
#include "main.h"
#include "stm32f1xx_hal.h"
#include "gpio.h"
#include "softspi.h"

static u8  SD_Type=0;//SD卡的类型 
u16 spi_speed = 100; //the spi speed(0-255),0 is fastest
//这个读写速度回间接影响到芯片能查找多少文件，这个值过大会导致查询时间过长，引起看门狗复位，这里改为通过函数确定该值的大小
u8 sd_state = 0;
//delay 1us?¨actually not??it maybe is several us??I don't test it??
void usleep(u16 i)
{
    delayus(i);
}

//set CS low
void CS_Enable(void)
{
    //set CS low
    SPI_CS_L
}

//set CS high and send 8 clocks
void CS_Disable(void)
{
    //set CS high
    SPI_CS_H;
    //send 8 clocks
    SDWriteByte(0xff);
}

//write a byte
void SDWriteByte(u8 data)
{
    u8 i;

      __disable_irq();
    //write 8 bits(MSB)
    for (i = 0; i < 8; i++)
    {
        SPI_CLK_L;
        usleep(spi_speed);
        
        if (data & 0x80)
            SPI_MO_H
        else
            SPI_MO_L

        data <<= 1;
        SPI_CLK_H
        usleep(spi_speed);
    }

    //when DI is free,it should be set high
    SPI_MO_H;
    __enable_irq();
}

//read a byte
u8 SDReadByte(void)
{
    u8 data = 0x00, i;

    //read 8 bit(MSB)
    for (i = 0; i < 8; i++)
    {
        SPI_CLK_L;
        usleep(spi_speed);
        SPI_CLK_H;
        data <<= 1;
        if (SPI_MI)
            data |= 0x01;
        usleep(spi_speed);
    }

    return data;
}
//取消选择,释放SPI总线
void SD_DisSelect(void)
{
    SPI_CS_H;
    SDWriteByte(0xff);//提供额外的8个时钟
}


//等待卡准备好
//返回值:0,准备好了;其他,错误代码
static u8 SD_WaitReady(void)
{
    u32 t=0;
    do
    {
        if(SDReadByte()==0XFF)
            return 0;//OK
        t++;
    }while(t<0XFFFFFF);//等待 
    return 1;
}

//选择sd卡,并且等待卡准备OK
//返回值:0,成功;1,失败;
u8 SD_Select(void)
{
    SPI_CS_L;
    
    if(SD_WaitReady()==0)
        return 0;//等待成功
        
    SD_DisSelect();
    return 1;//等待失败
}
//send a command and send back the response
u8 SDSendCmd(u8 cmd, u32 arg, u8 crc)
{
    u8 r1;  
    u8 Retry=0; 
    SD_DisSelect();//取消上次片选
    if(SD_Select())
    {
        printf("sd select failed\r\n");
        return 0XFF;//片选失效 
    }
    printf("send cmd%d\r\n",cmd);
    //发送
    SDWriteByte(cmd | 0x40);//分别写入命令
    SDWriteByte(arg >> 24);
    SDWriteByte(arg >> 16);
    SDWriteByte(arg >> 8);
    SDWriteByte(arg);
    SDWriteByte(crc);
    
    if(cmd==CMD12)
        SDWriteByte(0xff);//Skip a stuff byte when stop reading
    //等待响应，或超时退出
    Retry=0X1F;
    do
    {
        r1=SDReadByte();
    }while((r1&0X80) && Retry--);    
    //返回状态值
    return r1;
}

//reset SD card
u8 SDReset(void)
{
    u8 i, r1, time = 0;

    //set CS high
    CS_Disable();

    //send 128 clocks
    for (i = 0; i < 80; i++)
    {
        SDWriteByte(0xff);
    }

    //set CS low
    CS_Enable();

    //send CMD0 till the response is 0x01
    do
    {
        r1 = SDSendCmd(CMD0, 0, 0x95);
        time++;
        
        //if time out,set CS high and return r1
        if (time > 254)
        {
            //set CS high and send 8 clocks
            CS_Disable();
            return r1;
        }
    } while (r1 != 0x01);

    //set CS high and send 8 clocks
    CS_Disable();

    return 0;
}

//initial SD card(send CMD55+ACMD41 or CMD1)
/*
void SDIOinit(void)
{
    GPIO_InitTypeDef GPIO_Initure;
    
    __HAL_RCC_GPIOF_CLK_ENABLE();                   //使能GPIOF时钟
    
      
    //PF7 8 9
    GPIO_Initure.Pin=GPIO_PIN_7|GPIO_PIN_9;
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;              //复用推挽输出
    GPIO_Initure.Pull=GPIO_PULLUP;                  //上拉
    GPIO_Initure.Speed=GPIO_SPEED_FAST;             //快速    
    HAL_GPIO_Init(GPIOF,&GPIO_Initure);             //初始化
     
    GPIO_Initure.Pin=GPIO_PIN_8;
    GPIO_Initure.Mode=GPIO_MODE_INPUT;              //复用推挽输出
    GPIO_Initure.Pull=GPIO_PULLUP;                  //上拉
    GPIO_Initure.Speed=GPIO_SPEED_FAST;             //快速    
    HAL_GPIO_Init(GPIOF,&GPIO_Initure);             //初始化
    //片选 PF10
    GPIO_Initure.Pin=GPIO_PIN_10;                                       //片选 PF10
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;          //开输出
    GPIO_Initure.Pull=GPIO_PULLUP;                  //上拉
    GPIO_Initure.Speed=GPIO_SPEED_FAST;             //快速    
//    GPIO_Initure.Alternate=GPIO_AF5_SPI5;           //复用为SPI5S
    HAL_GPIO_Init(GPIOF,&GPIO_Initure);             //初始化

    HAL_GPIO_WritePin(GPIOF,GPIO_PIN_10,GPIO_PIN_SET);   //  拉高
}
*/
u8 SDInit(void)
{
    u8 r1,a;      // 存放SD卡的返回值
    u16 retry;  // 用来进行超时计数
    u8 buf[4];  
    u16 i;
    u8 time = 0;

/*
    SDIOinit();
*/
    SPI_GPIO_Soft_Init();

    SDSetLowSpeed(); //速度设为低速

    for(i=0;i<10;i++)
        SDWriteByte(0XFF);//发送最少74个脉冲

    retry=20;
    CS_Enable();

    do
    {
        r1=SDSendCmd(CMD0,0,0x95);//进入IDLE状态
    }while((r1!=0X01) && retry--);

    printf("after idle r1=0x%x\r\n",r1);

    if(r1==0X01)
    {
        a = SDSendCmd(CMD8,0x1AA,0x87);
        if(a==1)//SD V2.0
        {
            for(i=0;i<4;i++)buf[i]=SDReadByte();    //Get trailing return value of R7 resp
            if(buf[2]==0X01&&buf[3]==0XAA)//卡是否支持2.7~3.6V
            {
                retry=0XFFFE;
                do
                {
                    SDSendCmd(CMD55,0,0X01);    //发送CMD55
                    r1=SDSendCmd(ACMD41,0x40000000,0X01);//发送CMD41
                }while(r1&&retry--);
                if(retry&&SDSendCmd(CMD58,0,0X01)==0)//鉴别SD2.0卡版本开始
                {
                    for(i=0;i<4;i++)buf[i]=SDReadByte();//得到OCR值
                    if(buf[0]&0x40)SD_Type=SD_TYPE_V2HC;    //检查CCS
                    else SD_Type=SD_TYPE_V2;   
                }
            }
            else//SD V1.x/ MMC V3
            {
                SDSendCmd(CMD55,0,0X01);        //发送CMD55
                r1=SDSendCmd(ACMD41,0,0X01); //发送CMD41
                if(r1<=1)
                {       
                    SD_Type=SD_TYPE_V1;
                    retry=0XFFFE;
                    do //等待退出IDLE模式
                    {
                        SDSendCmd(CMD55,0,0X01);    //发送CMD55
                        r1=SDSendCmd(ACMD41,0,0X01);//发送CMD41
                    }while(r1&&retry--);
                }else//MMC卡不支持CMD55+CMD41识别
                {
                    SD_Type=SD_TYPE_MMC;//MMC V3
                    retry=0XFFFE;
                    do //等待退出IDLE模式
                    {                                               
                        r1=SDSendCmd(CMD1,0,0X01);//发送CMD1
                    }while(r1&&retry--);  
                }
                if(retry==0||SDSendCmd(CMD16,512,0X01)!=0)
                    SD_Type=SD_TYPE_ERR;//错误的卡
            }
        }
    }

    //set CS high and send 8 clocks
    CS_Disable();
    SDSetHighSpeed(); //速度设为高速

    if(SD_Type)
        return 0;
    else if(r1)
        return r1;

    return 0xaa;//其他错误
}

void SDSetLowSpeed(void)
{
    spi_speed = 10;
}

void SDSetHighSpeed(void)
{
    spi_speed = 2;
}

//read a single sector
u8 SDReadSector(u32 addr, u8 *buffer)
{
    u8 r1;
    u16 i, time = 0;

    //set CS low
    CS_Enable();

    //send CMD17 for single block read
    // addr=addr*512;
    r1 = SDSendCmd(CMD17, addr, 0x55);
    //if CMD17 fail,return
    if (r1 != 0x00)
    {
        //set CS high and send 8 clocks
        CS_Disable();
        return r1;
    }

    //continually read till get the start byte 0xfe
    do
    {
        r1 = SDReadByte();
        time++;
    
        //if time out,set CS high and return r1
        if (time > 30000)
        {
            //set CS high and send 8 clocks
            CS_Disable();
            return r1;
        }
    } while (r1 != 0xfe);

    //read 512 Bits of data
    for (i = 0; i < 512; i++)
    {
        buffer[i] = SDReadByte();
    }

    //read two bits of CRC
    SDReadByte();
    SDReadByte();

    //set CS high and send 8 clocks
    CS_Disable();

    return 0;
}

//read multiple sectors
u8 SDReadMultiSector(u32 addr, u8 sector_num, u8 *buffer)
{
    u16 i, time = 0;
    u8 r1;

    //set CS low
    CS_Enable();

    //send CMD18 for multiple blocks read
    r1 = SDSendCmd(CMD18, addr, 0xff);
    //if CMD18 fail,return
    if (r1 != 0x00)
    {
        //set CS high and send 8 clocks
        CS_Disable();
        return r1;
    }

    //read sector_num sector
    do
    {
        //continually read till get start byte
        do
        {
            r1 = SDReadByte();
            time++;
        
            //if time out,set CS high and return r1
            if (time > 30000 || ((r1 & 0xf0) == 0x00 && (r1 & 0x0f)))
            {
                //set CS high and send 8 clocks
                CS_Disable();
                return r1;
            }
        } while (r1 != 0xfe);
        time = 0;

        //read 512 Bits of data
        for (i = 0; i < 512; i++)
        {
            *buffer++ = SDReadByte();
        }

        //read two bits of CRC
        SDReadByte();
        SDReadByte();
    } while (--sector_num);
    time = 0;

    //stop multiple reading
    r1 = SDSendCmd(CMD12, 0, 0xff);

    //set CS high and send 8 clocks
    CS_Disable();

    return 0;
}

//write a single sector
u8 SDWriteSector(u32 addr, u8 *buffer)
{
    u16 i, time = 0;
    u8 r1;

    //set CS low
    CS_Enable();

    do
    {
        do
        {
            //send CMD24 for single block write
            r1 = SDSendCmd(CMD24, addr, 0xff);
            time++;
            
            //if time out,set CS high and return r1
            if (time > 60000)
            {
                //set CS high and send 8 clocks
                CS_Disable();
                return r1;
            }
        } while (r1 != 0x00);
        time = 0;

        //send some dummy clocks
        for (i = 0; i < 5; i++)
        {
            SDWriteByte(0xff);
        }

        //write start byte
        SDWriteByte(0xfe);

        //write 512 bytes of data
        for (i = 0; i < 512; i++)
        {
            SDWriteByte(buffer[i]);
        }

        //write 2 bytes of CRC
        SDWriteByte(0xff);
        SDWriteByte(0xff);

        //read response
        r1 = SDReadByte();
        time++;
    
        //if time out,set CS high and return r1
        if (time > 60000)
        {
            //set CS high and send 8 clocks
            CS_Disable();
            return r1;
        }
    } while ((r1 & 0x1f) != 0x05);
    time = 0;

    //check busy
    do
    {
        r1 = SDReadByte();
        time++;
    
        //if time out,set CS high and return r1
        if (time > 60000)
        {
            //set CS high and send 8 clocks
            CS_Disable();
            return r1;
        }
    } while (r1 != 0xff);

    //set CS high and send 8 clocks
    CS_Disable();

    return 0;
}

//write several blocks
u8 SDWriteMultiSector(u32 addr, u8 sector_num, u8 *buffer)
{
    u16 i, time = 0;
    u8 r1;

    //set CS low
    CS_Enable();

    //send CMD25 for multiple block read
    r1 = SDSendCmd(CMD25, addr, 0xff);
    //if CMD25 fail,return
    if (r1 != 0x00)
    {
        //set CS high and send 8 clocks
        CS_Disable();
        return r1;
    }

    do
    {
        do
        {
            //send several dummy clocks
            for (i = 0; i < 5; i++)
            {
                SDWriteByte(0xff);
            }

            //write start byte
            SDWriteByte(0xfc);

            //write 512 byte of data
            for (i = 0; i < 512; i++)
            {
                SDWriteByte(*buffer++);
            }

            //write 2 byte of CRC
            SDWriteByte(0xff);
            SDWriteByte(0xff);

            //read response
            r1 = SDReadByte();
            time++;
        
            //if time out,set CS high and return r1
            if (time > 254)
            {
                //set CS high and send 8 clocks
                CS_Disable();
                return r1;
            }
        } while ((r1 & 0x1f) != 0x05);
        time = 0;

        //check busy
        do
        {
            r1 = SDReadByte();
            printf("n%d", r1);
            time++;
        
            //if time out,set CS high and return r1
            if (time > 30000)
            {
                //set CS high and send 8 clocks
                CS_Disable();
                return r1;
            }
        } while (r1 != 0xff);
        time = 0;
    } while (--sector_num);

    //send stop byte
    SDWriteByte(0xfd);

    //check busy
    do
    {
        r1 = SDReadByte();
        time++;
    
        //if time out,set CS high and return r1
        if (time > 30000)
        {
            //set CS high and send 8 clocks
            CS_Disable();
            return r1;
        }
    } while (r1 != 0xff);

    //set CS high and send 8 clocks
    CS_Disable();

    return 0;
}

//get CID or CSD
u8 SDGetCIDCSD(u8 cid_csd, u8 *buffer)
{
    u8 r1;
    u16 i, time = 0;

    //set CS low
    CS_Enable();

    //send CMD10 for CID read or CMD9 for CSD
    do
    {
        if (cid_csd == CID)
            r1 = SDSendCmd(CMD10, 0, 0xff);
        else
            r1 = SDSendCmd(CMD9, 0, 0xff);
        time++;
        
        //if time out,set CS high and return r1
        if (time > 254)
        {
            //set CS high and send 8 clocks
            CS_Disable();
            return r1;
        }
    } while (r1 != 0x00);
    time = 0;

    //continually read till get 0xfe
    do
    {
        r1 = SDReadByte();
        time++;
        //if time out,set CS high and return r1
        if (time > 30000)
        {
            //set CS high and send 8 clocks
            CS_Disable();
            return r1;
        }
    } while (r1 != 0xfe);

    //read 512 Bits of data
    for (i = 0; i < 16; i++)
    {
        *buffer++ = SDReadByte();
    }

    //read two bits of CRC
    SDReadByte();
    SDReadByte();

    //set CS high and send 8 clocks
    CS_Disable();

    return 0;
}


/*
u8 csddata[16] = {0};
    SDGetCIDCSD(CSD, csddata);
    u32 csize = csddata[9] + ((uint32_t)csddata[8] << 8) + ((uint32_t)(csddata[7] & 0x3f) << 16) + 1;
    u32 Capacity = csize << 9;

*/

