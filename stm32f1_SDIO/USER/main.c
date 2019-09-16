/**********************************************************
                     SDIO-SDCARDʵ��

* @ Ӳ��ƽ̨��ս��STM32������

**********************************************************/

#include "STM32_config.h"

#include "led.h"
#include "lcd.h"
#include "lcd_drive.h"

#include "sdio_sdcard.h"

#include "ff.h"
#include "diskio.h"
#include "malloc.h"

u8 Dis_buffer[16];	//��ʾ����

/**********************************************************
                 �ļ�ϵͳ�����ļ���������
**********************************************************/

//���ļ���ʽ�궨�壬���f_open����
#define FA_OPEN_DEFAULT			(uint8_t)(FA_OPEN_EXISTING | FA_READ | FA_WRITE)	//�ɶ�д����

#define FA_OPEN_READONLY		(uint8_t)(FA_OPEN_EXISTING | FA_READ)				//ֻ��ȡ����ִ��д
#define FA_OPEN_ADD_DATA		(uint8_t)(FA_OPEN_ALWAYS | FA_READ | FA_WRITE)		//�ļ��������򴴽����ļ�
                                                                                    //����f_lseek�������ļ���׷������
#define FA_OPEN_NEW_FAIL		(uint8_t)(FA_CREATE_NEW | FA_READ | FA_WRITE)		//�½��ļ������������ʧ��
#define FA_OPEN_NEW_COVER		(uint8_t)(FA_CREATE_ALWAYS | FA_READ | FA_WRITE)	//�½��ļ�����������򸲸�

//�ļ�ϵͳ��������ض���
FATFS *fs[_VOLUMES];	//�߼����̹�����
FIL *filescr;			//�ļ�1
FIL *filedst;			//�ļ�2
UINT br,bw;				//��д����
FRESULT f_res;			//FatFsͨ�ý����

uint8_t *SDdatabuff;	//SD�����ݻ���
uint16_t rlen;			//��ȡ�����ݳ���

DIR *dir;				//�ļ���

FILINFO fileinfo;		//�ļ���Ϣ�ṹ��

u8 test_buff[] = {"SDCard FatFs Test OK!"};	//�����ַ���

/**********************************************************
* �������� ---> �ļ�ϵͳ��Ϣ��ʼ��
* ��ڲ��� ---> none
* ������ֵ ---> 0���ɹ�
*               1��ʧ��
* ����˵�� ---> ��Ҫ��Ϊ���������ڴ�
**********************************************************/
uint8_t myf_init(void)
{
	fs[0] = (FATFS*)mymalloc(SRAMIN, sizeof(FATFS));	//Ϊ����0�����������ڴ�
	fs[1] = (FATFS*)mymalloc(SRAMIN, sizeof(FATFS));	//Ϊ����1�����������ڴ�
	filescr = (FIL*)mymalloc(SRAMIN, sizeof(FIL));		//Ϊ�ļ�1�����ڴ�
	filedst = (FIL*)mymalloc(SRAMIN, sizeof(FIL));		//Ϊ�ļ�2�����ڴ�
	dir = (DIR*)mymalloc(SRAMIN, sizeof(DIR));			//Ϊ�ļ��������ڴ�
	SDdatabuff = (uint8_t*)mymalloc(SRAMIN, 512);		//ΪSD�����ݻ��������ڴ�
	
	if(fs[0]&&fs[1]&&filescr&&filedst&&dir&&SDdatabuff)	return 0;	//������һ��ʧ��, ��ʧ��
	else	return 1;	//����ʧ��
}

/**********************************************************
* �������� ---> ��ӡSD����Ϣ������
* ��ڲ��� ---> none
* ������ֵ ---> none
* ����˵�� ---> 1����ӡ��������������
*               2����ӡ�������͵�����
*               3����ӡ����������Ϣ
**********************************************************/
void SD_Card_Printf_Info(void)
{
	switch(SDCardInfo.CardType)	//������
	{
		case SDIO_HIGH_CAPACITY_SD_CARD:	//��������
			printf("Card Type: SDHC V2.0\r\n");
			break;
			
		case SDIO_STD_CAPACITY_SD_CARD_V1_1:	//��׼����V1.1
			printf("Card Type: SDSC V1.1\r\n");
			break;
			
		case SDIO_STD_CAPACITY_SD_CARD_V2_0:	//��׼����V2.0
			printf("Card Type: SDSC V2.0\r\n");
			break;

		case SDIO_MULTIMEDIA_CARD:	//MMC��
			printf("Card Type: MMC Card\r\n");
			break;
	}

	printf("Card ManufacturerID: %d\r\n",SDCardInfo.SD_cid.ManufacturerID);			//������ID
 	printf("Card RCA: %d\r\n",SDCardInfo.RCA);										//����Ե�ַ
	printf("Card Capacity: %d MB\r\n",(uint32_t)SDCardInfo.CardCapacity);	//��ʾ����
 	printf("Card BlockSize: %d\r\n\r\n",SDCardInfo.CardBlockSize);					//��ʾ���С
}

/**********************************************************
                           ������
**********************************************************/
int main(void) 
{
	u8 SDtatus;	//SD����ʼ��״̬
	
	u8 i;	
	
	MY_NVIC_PriorityGroup_Config(NVIC_PriorityGroup_2);	//�����жϷ���
	delay_init(72);	//��ʼ����ʱ����
	USARTx_Init(9600);	//��ʼ�����ڣ����ò�����Ϊ9600bps
	LED_Init();	//��ʼ��LED�ӿ�
	LCD_Init();	//��ʼ��TFT_LCD

	mem_init(SRAMIN);	//��ʼ���ڲ��ڴ��
	myf_init();	//Ϊ�ļ�ϵͳ�����ڴ�

	/******************************************************
	                      ��ʾ������Ϣ
	                     ��LCD������ʼ 
	******************************************************/
	LCD_Display_String_BK(30, 0, "Software Compiled Time:", 16, MAGENTA, WHITH);
	LCD_Display_String_BK(30, 16, __DATE__, 16, MAGENTA, WHITH);
	LCD_Display_String_BK(150, 16, __TIME__, 16, MAGENTA, WHITH);

	LCD_Display_String_BK(30, 32, "WarShip STM32.", 16, ORANGE, WHITH);
	LCD_Display_String_BK(30, 48, "SDIO SDCard&FatFs Test.", 16, ORANGE, WHITH);
	LCD_Display_String_BK(30, 64, "2014/02/26 week3", 16, ORANGE, WHITH);
	LCD_Display_String_BK(30, 80, "By@Sam Chan.", 16, ORANGE, WHITH);	
	
	sprintf((char*)Dis_buffer, "LCD ID:%04X", LCD_Manage.ID);//��LCD ID��ӡ��lcd_id����
	LCD_Display_String_BK(72, 96, Dis_buffer, 16, RED, WHITH);	//��ʾLCDID����ʾ����

	printf("Software Compiled Time: %s, %s.\r\n",__DATE__, __TIME__);	//��ȡ�������ʱ��

	/*******************���������ķָ���******************/
	
	/******************************************************
	                      SD����ʼ�� 
	******************************************************/
	if(SD_Init() != SD_OK)	//��ʼ��ʧ��
	{
		SDtatus = SD_Init();
		LCD_Display_String_BK(30, 112, "SD Init Faild!", 16, RED, WHITH);
		printf("The Faild is:%d\r\n", SDtatus);
	}
	else	//��ʼ���ɹ���
	{
		LCD_Display_String_BK(30, 112, "SD Init OK!!!!", 16, BLUE, WHITH);	
		SD_Card_Printf_Info();	//��ӡ����Ϣ������
		sprintf((char*)Dis_buffer, "MID is: %d", (u8)SDCardInfo.SD_cid.ManufacturerID);	//������ID
		LCD_Display_String_BK(30, 128, (u8*)Dis_buffer, 16, RED, WHITH);
		
		LCD_Display_String_BK(30, 144, "SD Size is:     MB", 16, BLUE, WHITH);	//��ʾ��������TFT
		Point_color = BLACK;
		LCD_Display_Array_No0(126, 144, SDCardInfo.CardCapacity, 4, 0);
	}

	/*******************���������ķָ���******************/
	
	/******************************************************
	                    ����FatFs�ú��� 
	******************************************************/
	f_res = f_mount(0, fs[0]);	//����SD��
	printf("f_mount res :%d\r\n", f_res);

	//������
	f_res = f_open(filescr, "0:/demo.txt", FA_OPEN_DEFAULT);	//���ļ�
	printf("f_open res :%d\r\n", f_res);
	
	if(f_res == FR_OK)	//���ļ��ɹ�
	{
		f_res = f_read(filescr, SDdatabuff, 30, &br);	//��ȡ�ļ�����
		printf("f_read res :%d\r\n", f_res);
	
		f_res = f_close(filescr);	//�ر��ļ�
		printf("f_close res :%d\r\n", f_res);
	
		printf("read string is: %s", SDdatabuff);	//��ӡ������
		LCD_Area_Color(30, 160, 239, 192, WHITH);	//�����ʾ����
		LCD_Display_String_BK(30, 160, "Read demo.txt data:", 16, RED, WHITH);
		LCD_Display_String_BK(30, 176, (u8*)SDdatabuff, 16, BLUE, WHITH);	//��ʾ��ȡ��������
	}
	else	//��ʧ��
	{
		LCD_Area_Color(30, 160, 239, 192, WHITH);	//�����ʾ����
		LCD_Display_String_BK(30, 176, "No demo.txt File.", 16, BLUE, WHITH);
	}	

	//д����
	f_res = f_open(filedst, "0:/test.txt", FA_OPEN_NEW_COVER);	//�����ļ�����������򸲸�
	printf("\r\nf_open res :%d\r\n", f_res);

	f_res = f_write(filedst, test_buff, sizeof(test_buff), &bw);	//д���ַ���
	printf("f_write res :%d\r\n", f_res);

	f_res = f_close(filedst);	//�ر��ļ�
	printf("f_close res :%d\r\n", f_res);

	//�򿪸ղŴ������ļ�
	f_res = f_open(filescr, "0:/test.txt", FA_OPEN_DEFAULT);	//���ļ�
	printf("f_open res :%d\r\n", f_res);
	
	f_res = f_read(filescr, SDdatabuff, 30, &br);	//��ȡ�ļ�����
	printf("f_read res :%d\r\n", f_res);

	f_res = f_close(filescr);	//�ر��ļ�
	printf("f_close res :%d\r\n", f_res);

	printf("read string is: %s", SDdatabuff);
	LCD_Display_String_BK(30, 192, "Write/Read test.txt data:", 16, RED, WHITH);
	LCD_Display_String_BK(30, 208, (u8*)SDdatabuff, 16, BLUE, WHITH);	//��ʾ��ȡ��������

	/*******************���������ķָ���******************/

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


