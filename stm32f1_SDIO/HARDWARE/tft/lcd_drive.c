/**********************************************************
* @ File name -> lcd_drive.c
* @ Version   -> V1.0
* @ Date      -> 12-15-2013
* @ Brief     -> TFT�����ײ���������
**********************************************************/

#include "lcd_drive.h"

//=========================================================
#if _USER_GPIO_OR_FSMC      //��������ʹ��FSMC����LCD
//========================================================

/**********************************************************
* �������� ---> д���ݵ�LCD�Ĵ�������Ҫд�����Ĵ�����ַ��
* ��ڲ��� ---> reg_val��Ҫд�������
* ������ֵ ---> none
* ����˵�� ---> none
**********************************************************/
void LCD_Write_Register(u16 reg_val)
{
      LCD_WR_REG(reg_val);
}
/**********************************************************
* �������� ---> д���ݵ�LCD RAM
* ��ڲ��� ---> dat��Ҫд�������
* ������ֵ ---> none
* ����˵�� ---> none
**********************************************************/
void LCD_Write_Data(u16 dat)
{
      LCD_WR_Data(dat);
}
/**********************************************************
* �������� ---> ��ȡLCD����
* ��ڲ��� ---> none
* ������ֵ ---> ��ȡ��������
* ����˵�� ---> none
**********************************************************/
u16 LCD_Read_Data(void)
{
      return (*(__IO u16 *)(LCD_DAT_ADD));
}
/**********************************************************
* �������� ---> ��LCDĳ���Ĵ���д������
* ��ڲ��� ---> reg��Ҫд�����ݵļĴ�����ַ
*               dat��Ҫд�������
* ������ֵ ---> none
* ����˵�� ---> none
**********************************************************/
void LCD_WriteRegisterData(u16 reg, u16 dat)
{
      LCD_WR_REG(reg);
      LCD_WR_Data(dat);
}
/**********************************************************
* �������� ---> ��ȡLCDĳ���Ĵ�����ֵ
* ��ڲ��� ---> reg��Ҫ��ȡ���ݵļĴ�����ַ
* ������ֵ ---> �Ĵ�����ֵ
* ����˵�� ---> none
**********************************************************/
u16 LCD_ReadRegisterData(u16 reg)
{
      LCD_Write_Register(reg);
      delay_us(5);
      return LCD_Read_Data();
}
/**********************************************************
* �������� ---> FSMC��ʼ��
* ��ڲ��� ---> none
* ������ֵ ---> none
* ����˵�� ---> none
**********************************************************/
void STM32_FSMC_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
      
	FSMC_NORSRAMInitTypeDef FSMC_TFTLCD_InitStructure;

	FSMC_NORSRAMTimingInitTypeDef ReadWrite_Time;
	FSMC_NORSRAMTimingInitTypeDef Write_Time;
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_FSMC,ENABLE);	//ʹ��FSMCʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE | RCC_APB2Periph_GPIOG | RCC_APB2Periph_AFIO, ENABLE);	//��������ʱ��

	/*	��ʼ��GPIODΪ�����������	*/
	/*	PD.0ΪFSMC_D2    PD.1ΪFSMC_D3	*/
	/*	PD.4ΪFSMC_NOE    PD.5ΪFSMC_NWE	*/
	/*	PD.8ΪFSMC_D13    PD.9ΪFSMC_D14    PD.10ΪFSMC_D15	*/
	/*	PD.14ΪFSMC_D0    PD.15ΪFSMC_D1	*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_8 | \
	                              GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//GPIO��ת�ٶ�Ϊ50MHz
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//����Ϊ�����������

	GPIO_Init(GPIOD, &GPIO_InitStructure);	//��ʼ��GPIO��ؽṹ��

	/*	��ʼ��GPIOEΪ�����������	*/
	/*	PE.7ΪFSMC_D4    PE.8ΪFSMC_D5	*/
	/*	PE.9ΪFSMC_D6    PE.10ΪFSMC_D7	*/
	/*	PE.11ΪFSMC_D8    PE.12ΪFSMC_D9	*/
	/*	PE.13ΪFSMC_D10    PE.14ΪFSMC_D11	*/
	/*	PE.5ΪFSMC_D12	*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | \
	                              GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//GPIO��ת�ٶ�Ϊ50MHz
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//����Ϊ�����������

	GPIO_Init(GPIOE, &GPIO_InitStructure);	//��ʼ��GPIO��ؽṹ��

	/*	��ʼ��GPIOGΪ�����������	*/
	/*	PG.0ΪFSMC_A10    PG.12ΪFSMC_NE4	*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//GPIO��ת�ٶ�Ϊ50MHz
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//����Ϊ�����������

	GPIO_Init(GPIOG, &GPIO_InitStructure);	//��ʼ��GPIO��ؽṹ��

	
	/*	��ʼ��FSMC����Ĵ���	*/

	/*	��ʼ����дʱ��	*/
	ReadWrite_Time.FSMC_AddressSetupTime = 0x01;	//FSMC��ַ����ʱ��Ϊ2��HCLKʱ�ӣ�1/36MHz = 27ns
	ReadWrite_Time.FSMC_AddressHoldTime = 0x00;	//��ַ����ʱ�䣬ģʽAδ�õ�
	ReadWrite_Time.FSMC_DataSetupTime = 0xff;	//���ݽ���ʱ��Ϊ16��HCLK
	ReadWrite_Time.FSMC_BusTurnAroundDuration = 0x00;	//
	ReadWrite_Time.FSMC_CLKDivision = 0x00;	//
	ReadWrite_Time.FSMC_DataLatency = 0x00;	//
	ReadWrite_Time.FSMC_AccessMode = FSMC_AccessMode_A;	//ѡ��ģʽA

	/*	��ʼ��дʱ��	*/
	Write_Time.FSMC_AddressSetupTime = 0x00;	//��ַ����ʱ��Ϊ1��HCLK
	Write_Time.FSMC_AddressHoldTime = 0x00;	//��ַ����ʱ�䣬ģʽAδ�õ�
	Write_Time.FSMC_DataSetupTime = 0x03;	//���ݽ���ʱ���ڵ��ĸ�HCLKʱ�ӿ�ʼ
	Write_Time.FSMC_BusTurnAroundDuration = 0x00;	//
	Write_Time.FSMC_CLKDivision = 0x00;	//
	Write_Time.FSMC_DataLatency = 0x00;	//
	Write_Time.FSMC_AccessMode = FSMC_AccessMode_A;	//ѡ��ģʽA	
	
	FSMC_TFTLCD_InitStructure.FSMC_Bank = FSMC_Bank1_NORSRAM4;	//��bank1����4
	FSMC_TFTLCD_InitStructure.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable;	//��ַ/���ݵ�ַ���ù��ܹر�
	FSMC_TFTLCD_InitStructure.FSMC_MemoryType = FSMC_MemoryType_SRAM;	//��Һ����ʾ��������SRAM��
	FSMC_TFTLCD_InitStructure.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_16b;	//ѡ�����ݿ��Ϊ16bit
	FSMC_TFTLCD_InitStructure.FSMC_BurstAccessMode = FSMC_BurstAccessMode_Disable;	//���鴫��ģʽ�ر�
	FSMC_TFTLCD_InitStructure.FSMC_AsynchronousWait = FSMC_AsynchronousWait_Disable;	// 
	FSMC_TFTLCD_InitStructure.FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low;	//�͵�ƽ�ı����� 
	FSMC_TFTLCD_InitStructure.FSMC_WrapMode = FSMC_WrapMode_Disable;	//
	FSMC_TFTLCD_InitStructure.FSMC_WaitSignalActive = FSMC_WaitSignalActive_BeforeWaitState;	//
	FSMC_TFTLCD_InitStructure.FSMC_WriteOperation = FSMC_WriteOperation_Enable;	//��д����
	FSMC_TFTLCD_InitStructure.FSMC_WaitSignal = FSMC_WaitSignal_Disable;	//����NWAIT�źţ�������������ȴ��ź�
	FSMC_TFTLCD_InitStructure.FSMC_ExtendedMode = FSMC_ExtendedMode_Enable;	//����չģʽ�������дʹ�ò�ͬ��ʱ��
	FSMC_TFTLCD_InitStructure.FSMC_WriteBurst = FSMC_WriteBurst_Disable;	//д����ʼ�մ����첽ģʽ

	FSMC_TFTLCD_InitStructure.FSMC_ReadWriteTimingStruct = &ReadWrite_Time;	//��дʱ��
	FSMC_TFTLCD_InitStructure.FSMC_WriteTimingStruct = &Write_Time;	//дʱ��
	
	FSMC_NORSRAMInit(&FSMC_TFTLCD_InitStructure);	//��ʼ��FSMC����

	FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM4, ENABLE);	//ʹ��FSMC��1��4��
}

//=========================================================
#else //����ʹ����ͨGPIO����LCD
//=========================================================

/**********************************************************
* �������� ---> д���ݵ�����
* ��ڲ��� ---> val��Ҫд�������
* ������ֵ ---> none
* ����˵�� ---> none
**********************************************************/
void LCD_Write_Bus(u16 val)
{
      #if _USER_8BIT_16BIT  //��������ʹ��16bit����
      /* ʹ��16bits�������߿��ʱ���߼�����ʱ�� */

      #else //����ʹ��8bit����
	  /* ʹ��8bits�������߿��ʱ���߼�����ʱ�� */

      #endif
}
/**********************************************************
* �������� ---> д���ݵ�LCD�Ĵ�������Ҫд�����Ĵ�����ַ��
* ��ڲ��� ---> reg_val��Ҫд�������
* ������ֵ ---> none
* ����˵�� ---> none
**********************************************************/
void LCD_Write_Register(u16 reg_val)
{
      LCD_RS = 0; //���͵���COM Address
      LCD_Write_Bus(reg_val);      
}
/**********************************************************
* �������� ---> д���ݵ�LCD RAM
* ��ڲ��� ---> dat��Ҫд�������
* ������ֵ ---> none
* ����˵�� ---> none
**********************************************************/
void LCD_Write_Data(u16 dat)
{
      LCD_RS = 1; //���͵���data
      LCD_Write_Bus(reg_val);
}
/**********************************************************
* �������� ---> ��ȡLCD����
* ��ڲ��� ---> none
* ������ֵ ---> ��ȡ��������
* ����˵�� ---> none
**********************************************************/
u16 LCD_Read_Data(void)
{
	#if _USER_8BIT_16BIT  //��������ʹ��16bit����
	/* ʹ��16bits�������߿��ʱ���߼�����ʱ�� */

	#else //����ʹ��8bit����
	/* ʹ��8bits�������߿��ʱ���߼�����ʱ�� */

	#endif     
}
/**********************************************************
* �������� ---> ��LCDĳ���Ĵ���д������
* ��ڲ��� ---> reg��Ҫд�����ݵļĴ�����ַ
*               dat��Ҫд�������
* ������ֵ ---> none
* ����˵�� ---> none
**********************************************************/
void LCD_WriteRegisterData(u16 reg, u16 dat)
{
      LCD_CS = 0;
      LCD_Write_Register(reg);
      LCD_Write_Data(dat);
      LCD_CS = 1;
}
/**********************************************************
* �������� ---> ��ȡLCDĳ���Ĵ�����ֵ
* ��ڲ��� ---> reg��Ҫ��ȡ���ݵļĴ�����ַ
* ������ֵ ---> �Ĵ�����ֵ
* ����˵�� ---> none
**********************************************************/
u16 LCD_ReadRegisterData(u16 reg)
{
      u16 tem;
      
      LCD_CS = 0;
      LCD_Write_Register(reg);
      delay_us(5);
      tem = LCD_Read_Data();
      LCD_CS = 1;
      return tem;
}
/**********************************************************
* �������� ---> ͨѶ�����ƶ˿ڳ�ʼ��
* ��ڲ��� ---> none
* ������ֵ ---> none
* ����˵�� ---> none
**********************************************************/
void LCD_GPIO_Init(void)
{
	/* ��дGPIO����LCD�����ߡ������ߵȳ�ʼ������ */
}

//=========================================================
#endif	//end _USER_GPIO_OR_FSMC
//=========================================================

/**********************************************************
* �������� ---> ���⡢��λ���ƶ˿ڳ�ʼ��
* ��ڲ��� ---> none
* ������ֵ ---> none
* ����˵�� ---> none
**********************************************************/
void LCD_RST_BL_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* �������õ������ߵ�IOʱ�� */

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	//��������ʱ��

	/******����ܽ�******/

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;	//��ʼ��GPIOB.0 ---> GPIOB.15
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;	//GPIO��ת�ٶ�Ϊ50MHz
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	//����Ϊ�������

	GPIO_Init(GPIOB, &GPIO_InitStructure);	//��ʼ��GPIO��ؽṹ��

	GPIO_ResetBits(GPIOB, GPIO_Pin_0);	//����͵�ƽ

	/******************************************************
	         ����������ѡ������Ҫ��д�ĳ�ʼ������
	******************************************************/

	/******��λ�ܽ�******/
      
	#if _LCD_RESET_Soft   //��������ʹ��������Ƹ�λ����ʼ���˿�

		/* ��д����LCD_RST�ܽų�ʼ������ */

	#endif

	/******�������߿��ѡ��ܽ�******/

	#if _USER_PSB_Soft    //��������ʹ���������

		/* ��д��ʼ��LCD�������߿�ȿ����߳�ʼ������ */

		#if _USER_8BIT_16BIT  //��������ʹ��16bit
			/* LCD�������߿�ȹܽŵ�ƽ��16bitsΪ�͵�ƽ */
			LCD_IM0 = 0;
		#else //����ʹ��8bit
			/* LCD�������߿�ȹܽŵ�ƽ��8bitsΪ�ߵ�ƽ */
			LCD_IM0 = 1;
		#endif

	#endif
}


