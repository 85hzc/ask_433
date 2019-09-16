/**********************************************************
                     SDIO-SDCARD实验

* @ 硬件平台：战舰STM32开发板

**********************************************************/

#include "STM32_config.h"

#include "led.h"
#include "lcd.h"
#include "lcd_drive.h"

#include "sdio_sdcard.h"

#include "ff.h"
#include "diskio.h"
#include "malloc.h"

u8 Dis_buffer[16];	//显示缓存

/**********************************************************
                 文件系统公共文件操作区域
**********************************************************/

//打开文件方式宏定义，针对f_open函数
#define FA_OPEN_DEFAULT			(uint8_t)(FA_OPEN_EXISTING | FA_READ | FA_WRITE)	//可读写操作

#define FA_OPEN_READONLY		(uint8_t)(FA_OPEN_EXISTING | FA_READ)				//只读取，不执行写
#define FA_OPEN_ADD_DATA		(uint8_t)(FA_OPEN_ALWAYS | FA_READ | FA_WRITE)		//文件不存在则创建新文件
                                                                                    //可用f_lseek函数在文件上追加数据
#define FA_OPEN_NEW_FAIL		(uint8_t)(FA_CREATE_NEW | FA_READ | FA_WRITE)		//新建文件，如果存在则失败
#define FA_OPEN_NEW_COVER		(uint8_t)(FA_CREATE_ALWAYS | FA_READ | FA_WRITE)	//新建文件，如果存在则覆盖

//文件系统工作区相关定义
FATFS *fs[_VOLUMES];	//逻辑磁盘工作区
FIL *filescr;			//文件1
FIL *filedst;			//文件2
UINT br,bw;				//读写变量
FRESULT f_res;			//FatFs通用结果码

uint8_t *SDdatabuff;	//SD卡数据缓存
uint16_t rlen;			//读取到数据长度

DIR *dir;				//文件夹

FILINFO fileinfo;		//文件信息结构体

u8 test_buff[] = {"SDCard FatFs Test OK!"};	//测试字符串

/**********************************************************
* 函数功能 ---> 文件系统信息初始化
* 入口参数 ---> none
* 返回数值 ---> 0：成功
*               1：失败
* 功能说明 ---> 主要是为变量申请内存
**********************************************************/
uint8_t myf_init(void)
{
	fs[0] = (FATFS*)mymalloc(SRAMIN, sizeof(FATFS));	//为磁盘0工作区申请内存
	fs[1] = (FATFS*)mymalloc(SRAMIN, sizeof(FATFS));	//为磁盘1工作区申请内存
	filescr = (FIL*)mymalloc(SRAMIN, sizeof(FIL));		//为文件1申请内存
	filedst = (FIL*)mymalloc(SRAMIN, sizeof(FIL));		//为文件2申请内存
	dir = (DIR*)mymalloc(SRAMIN, sizeof(DIR));			//为文件夹申请内存
	SDdatabuff = (uint8_t*)mymalloc(SRAMIN, 512);		//为SD卡数据缓存申请内存
	
	if(fs[0]&&fs[1]&&filescr&&filedst&&dir&&SDdatabuff)	return 0;	//申请有一个失败, 即失败
	else	return 1;	//申请失败
}

/**********************************************************
* 函数功能 ---> 打印SD卡信息到串口
* 入口参数 ---> none
* 返回数值 ---> none
* 功能说明 ---> 1、打印卡的容量到串口
*               2、打印卡的类型到串口
*               3、打印卡的其他信息
**********************************************************/
void SD_Card_Printf_Info(void)
{
	switch(SDCardInfo.CardType)	//卡类型
	{
		case SDIO_HIGH_CAPACITY_SD_CARD:	//高容量卡
			printf("Card Type: SDHC V2.0\r\n");
			break;
			
		case SDIO_STD_CAPACITY_SD_CARD_V1_1:	//标准容量V1.1
			printf("Card Type: SDSC V1.1\r\n");
			break;
			
		case SDIO_STD_CAPACITY_SD_CARD_V2_0:	//标准容量V2.0
			printf("Card Type: SDSC V2.0\r\n");
			break;

		case SDIO_MULTIMEDIA_CARD:	//MMC卡
			printf("Card Type: MMC Card\r\n");
			break;
	}

	printf("Card ManufacturerID: %d\r\n",SDCardInfo.SD_cid.ManufacturerID);			//制造商ID
 	printf("Card RCA: %d\r\n",SDCardInfo.RCA);										//卡相对地址
	printf("Card Capacity: %d MB\r\n",(uint32_t)SDCardInfo.CardCapacity);	//显示容量
 	printf("Card BlockSize: %d\r\n\r\n",SDCardInfo.CardBlockSize);					//显示块大小
}

/**********************************************************
                           主函数
**********************************************************/
int main(void) 
{
	u8 SDtatus;	//SD卡初始化状态
	
	u8 i;	
	
	MY_NVIC_PriorityGroup_Config(NVIC_PriorityGroup_2);	//设置中断分组
	delay_init(72);	//初始化延时函数
	USARTx_Init(9600);	//初始化串口，设置波特率为9600bps
	LED_Init();	//初始化LED接口
	LCD_Init();	//初始化TFT_LCD

	mem_init(SRAMIN);	//初始化内部内存池
	myf_init();	//为文件系统申请内存

	/******************************************************
	                      显示基本信息
	                     从LCD顶部开始 
	******************************************************/
	LCD_Display_String_BK(30, 0, "Software Compiled Time:", 16, MAGENTA, WHITH);
	LCD_Display_String_BK(30, 16, __DATE__, 16, MAGENTA, WHITH);
	LCD_Display_String_BK(150, 16, __TIME__, 16, MAGENTA, WHITH);

	LCD_Display_String_BK(30, 32, "WarShip STM32.", 16, ORANGE, WHITH);
	LCD_Display_String_BK(30, 48, "SDIO SDCard&FatFs Test.", 16, ORANGE, WHITH);
	LCD_Display_String_BK(30, 64, "2014/02/26 week3", 16, ORANGE, WHITH);
	LCD_Display_String_BK(30, 80, "By@Sam Chan.", 16, ORANGE, WHITH);	
	
	sprintf((char*)Dis_buffer, "LCD ID:%04X", LCD_Manage.ID);//将LCD ID打印到lcd_id数组
	LCD_Display_String_BK(72, 96, Dis_buffer, 16, RED, WHITH);	//显示LCDID到显示屏上

	printf("Software Compiled Time: %s, %s.\r\n",__DATE__, __TIME__);	//获取软件编译时间

	/*******************我是美丽的分割线******************/
	
	/******************************************************
	                      SD卡初始化 
	******************************************************/
	if(SD_Init() != SD_OK)	//初始化失败
	{
		SDtatus = SD_Init();
		LCD_Display_String_BK(30, 112, "SD Init Faild!", 16, RED, WHITH);
		printf("The Faild is:%d\r\n", SDtatus);
	}
	else	//初始化成功了
	{
		LCD_Display_String_BK(30, 112, "SD Init OK!!!!", 16, BLUE, WHITH);	
		SD_Card_Printf_Info();	//打印卡信息到串口
		sprintf((char*)Dis_buffer, "MID is: %d", (u8)SDCardInfo.SD_cid.ManufacturerID);	//制造商ID
		LCD_Display_String_BK(30, 128, (u8*)Dis_buffer, 16, RED, WHITH);
		
		LCD_Display_String_BK(30, 144, "SD Size is:     MB", 16, BLUE, WHITH);	//显示卡容量到TFT
		Point_color = BLACK;
		LCD_Display_Array_No0(126, 144, SDCardInfo.CardCapacity, 4, 0);
	}

	/*******************我是美丽的分割线******************/
	
	/******************************************************
	                    测试FatFs用函数 
	******************************************************/
	f_res = f_mount(0, fs[0]);	//挂载SD卡
	printf("f_mount res :%d\r\n", f_res);

	//读测试
	f_res = f_open(filescr, "0:/demo.txt", FA_OPEN_DEFAULT);	//打开文件
	printf("f_open res :%d\r\n", f_res);
	
	if(f_res == FR_OK)	//打开文件成功
	{
		f_res = f_read(filescr, SDdatabuff, 30, &br);	//读取文件内容
		printf("f_read res :%d\r\n", f_res);
	
		f_res = f_close(filescr);	//关闭文件
		printf("f_close res :%d\r\n", f_res);
	
		printf("read string is: %s", SDdatabuff);	//打印到串口
		LCD_Area_Color(30, 160, 239, 192, WHITH);	//清除显示区域
		LCD_Display_String_BK(30, 160, "Read demo.txt data:", 16, RED, WHITH);
		LCD_Display_String_BK(30, 176, (u8*)SDdatabuff, 16, BLUE, WHITH);	//显示读取到的内容
	}
	else	//打开失败
	{
		LCD_Area_Color(30, 160, 239, 192, WHITH);	//清除显示区域
		LCD_Display_String_BK(30, 176, "No demo.txt File.", 16, BLUE, WHITH);
	}	

	//写测试
	f_res = f_open(filedst, "0:/test.txt", FA_OPEN_NEW_COVER);	//创建文件，如果存在则覆盖
	printf("\r\nf_open res :%d\r\n", f_res);

	f_res = f_write(filedst, test_buff, sizeof(test_buff), &bw);	//写入字符串
	printf("f_write res :%d\r\n", f_res);

	f_res = f_close(filedst);	//关闭文件
	printf("f_close res :%d\r\n", f_res);

	//打开刚才创建的文件
	f_res = f_open(filescr, "0:/test.txt", FA_OPEN_DEFAULT);	//打开文件
	printf("f_open res :%d\r\n", f_res);
	
	f_res = f_read(filescr, SDdatabuff, 30, &br);	//读取文件内容
	printf("f_read res :%d\r\n", f_res);

	f_res = f_close(filescr);	//关闭文件
	printf("f_close res :%d\r\n", f_res);

	printf("read string is: %s", SDdatabuff);
	LCD_Display_String_BK(30, 192, "Write/Read test.txt data:", 16, RED, WHITH);
	LCD_Display_String_BK(30, 208, (u8*)SDdatabuff, 16, BLUE, WHITH);	//显示读取到的内容

	/*******************我是美丽的分割线******************/

	LED1 = 0;
	while(1)
	{

		i++;
		delay_ms(5);
		if(i == 60)
		{
			LED0 = ~LED0;
			LED1 = ~LED1;
			i = 0;
		}
	}

}


