#ifndef _SOFTSPI_H_
#define _SOFTSPI_H_
 
#define CMD0    0   /* GO_IDLE_STATE */
#define CMD55   55  /* APP_CMD */
#define ACMD41  41  /* SEND_OP_COND (ACMD) */
#define CMD1    1   /* SEND_OP_COND */
#define CMD16   16  //命令16，设置SectorSize 应返回0x00
#define CMD17   17  /* READ_SINGLE_BLOCK */
#define CMD8    8   /* SEND_IF_COND */
#define CMD18   18  /* READ_MULTIPLE_BLOCK */
#define CMD12   12  /* STOP_TRANSMISSION */
#define CMD24   24  /* WRITE_BLOCK */
#define CMD25   25  /* WRITE_MULTIPLE_BLOCK */
#define CMD13   13  /* SEND_STATUS */
#define CMD9    9   /* SEND_CSD */
#define CMD10   10  /* SEND_CID */
#define CMD58   58  //命令58，读OCR信息
#define CMD59   59  //命令59，使能/禁止CRC，应返回0x00

#define CSD     9
#define CID     10

/* SD卡类型定义 */
#define SD_TYPE_MMC     0
#define SD_TYPE_V1      1
#define SD_TYPE_V2      2
#define SD_TYPE_V2HC    4
#define SD_TYPE_ERR     5

 //这部分应根据具体的连线来修改!                 
#define SD_CS        (GPIO_PIN_12)
#define SD_MISO      (GPIO_PIN_14)
#define SD_CLK       (GPIO_PIN_13)
#define SD_MOSI      (GPIO_PIN_15)


//MISO                             
#define  SPI_MI (HAL_GPIO_ReadPin(GPIOB,SD_MISO))

//CS
#define  SPI_CS_L   GPIOB->BSRR = (uint32_t)SD_CS << 16;       // 输出低电平       
#define  SPI_CS_H   GPIOB->BSRR = SD_CS;                       // 输出高电平
//CLK
#define  SPI_CLK_L  GPIOB->BSRR = (uint32_t)SD_CLK << 16;       // 输出低电平     
#define  SPI_CLK_H  GPIOB->BSRR = SD_CLK;                       // 输出高电平
//MOSI
#define  SPI_MO_L   GPIOB->BSRR = (uint32_t)SD_MOSI << 16;       // 输出低电平     
#define  SPI_MO_H   GPIOB->BSRR = SD_MOSI;                       // 输出高电平



//delay 1us（actually not，it maybe is several us，I don't test it）
void usleep(u16 i);
 
//set CS low
void CS_Enable();
 
//set CS high and send 8 clocks
void CS_Disable();
 
//write a byte
void SDWriteByte(u8 data);
 
//read a byte
u8 SDReadByte();

//send a command and send back the response
u8 SDSendCmd(u8 cmd,u32 arg,u8 crc);

//reset SD card
u8 SDReset();
 
//initial SD card
u8 SDInit();
//when init set speed low
//after init set speed high
void SDSetHighSpeed(void);
void SDSetLowSpeed(void);
//read a single sector
u8 SDReadSector(u32 addr,u8 * buffer);
 
//read multiple sectors
u8 SDReadMultiSector(u32 addr,u8 sector_num,u8 * buffer);
 
//write a single sector
u8 SDWriteSector(u32 addr,u8 * buffer);
 
//write multiple sectors
u8 SDWriteMultiSector(u32 addr,u8 sector_num,u8 * buffer);
 
//get CID or CSD
u8 SDGetCIDCSD(u8 cid_csd,u8 * buffer);

void SD_DisSelect(void);
u8 SD_Select(void);
 
//spi speed（0-255），0 is fastest
//u8 spi_speed;
//SD IO init
//void SDIOinit(void);
 
#endif /* SD_SPI_SOLUTION_H_ */
