/**********************************************************
* @ File name -> sdio_sdcard.c
* @ Version   -> V1.0
* @ Date      -> 02-09-2014
* @ Brief     -> SDCard的SDIO驱动函数

 V1.
* @ Revise    -> 
**********************************************************/

#include "sdio_sdcard.h"

/* 为了数据对齐，下面的buffer是为读写disk准备的，当数据不是4K对齐的时候会用到，否则直接操作即可 */

__align(4) uint8_t SDIO_DATA_BUFFER[512];

/**********************************************************
                       定义相关的变量
**********************************************************/

static uint32_t CardType =  SDIO_STD_CAPACITY_SD_CARD_V1_1;	//默认卡的类型为标准V1.1卡
static uint32_t CSD_Tab[4], CID_Tab[4], RCA = 0;			//CID、CSD、RCA缓存
static uint8_t SDSTATUS_Tab[16];							//SD卡状态缓存

__IO SD_Error TransferError = SD_OK;
__IO uint32_t StopCondition = 0;							//是否发送停止命令操作标志，多块读写时候用到
__IO uint32_t TransferEnd = 0;								//传输结束标志
SD_CardInfo SDCardInfo;										//SD卡信息结构定义

SDIO_InitTypeDef SDIO_InitStructure;
SDIO_CmdInitTypeDef SDIO_CmdInitStructure;
SDIO_DataInitTypeDef SDIO_DataInitStructure;
GPIO_InitTypeDef GPIO_InitStructure;

/**********************************************************
                        功能函数
**********************************************************/

static SD_Error CmdError(void);
static SD_Error CmdResp1Error(uint8_t cmd);
static SD_Error CmdResp7Error(void);
static SD_Error CmdResp3Error(void);
static SD_Error CmdResp2Error(void);
static SD_Error CmdResp6Error(uint8_t cmd, uint16_t *prca);
static SD_Error SDEnWideBus(FunctionalState NewState);
static SD_Error IsCardProgramming(uint8_t *pstatus);
static SD_Error FindSCR(uint16_t rca, uint32_t *pscr);
uint8_t convert_from_bytes_to_power_of_two(uint16_t NumberOfBytes);

/**********************************************************
* 函数功能 ---> DMA2传输状态
* 入口参数 ---> none
* 返回数值 ---> 状态值
* 功能说明 ---> none
**********************************************************/
uint32_t SD_DMAEndOfTransferStatus(void)
{
	return (uint32_t)DMA_GetFlagStatus(DMA2_FLAG_TC4);   //Channel4 transfer complete flag
}
/**********************************************************
* 函数功能 ---> DMA2接收数据
* 入口参数 ---> *BufferSRC：接收数据缓存
*				BufferSize：接收多少个块数据
* 返回数值 ---> none
* 功能说明 ---> none
**********************************************************/
static void SD_DMA_RxConfig(uint32_t *BufferSRC, uint32_t BufferSize)
{
	DMA_InitTypeDef DMA_InitStructure;

	DMA_ClearFlag(DMA2_FLAG_TC4 | DMA2_FLAG_TE4 | DMA2_FLAG_HT4 | DMA2_FLAG_GL4);//清除DMA标志位
	/*!< DMA2 Channel4 disable */
	DMA_Cmd(DMA2_Channel4, DISABLE);	//关闭DMA4第四通道
	/*!< DMA2 Channel4 Config */
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&SDIO->FIFO;		//外设地址
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)BufferSRC;				//目标地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;						//传输方向，从外设读
	DMA_InitStructure.DMA_BufferSize = BufferSize / 4;						//数据传输数量，1/4缓存大小
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;		//外设地址不自增
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;					//使能存储目标地址自增
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;	//外设数据大小为字，32位
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;			//DMA2数据大小为字，32位
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;							//不循环
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;						//通道优先级高
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;							//非存储器至存储器模式
	DMA_Init(DMA2_Channel4, &DMA_InitStructure);
	/*!< DMA2 Channel4 enable */
	DMA_Cmd(DMA2_Channel4, ENABLE); 
}
/**********************************************************
* 函数功能 ---> DMA2发送数据
* 入口参数 ---> *BufferDST：发送数据缓存
*				BufferSize：发送多少个块数据
* 返回数值 ---> none
* 功能说明 ---> none
**********************************************************/
static void SD_DMA_TxConfig(uint32_t *BufferDST, uint32_t BufferSize)
{
	DMA_InitTypeDef DMA_InitStructure;
	
	DMA_ClearFlag(DMA2_FLAG_TC4 | DMA2_FLAG_TE4 | DMA2_FLAG_HT4 | DMA2_FLAG_GL4);
	/*!< DMA2 Channel4 disable */
	DMA_Cmd(DMA2_Channel4, DISABLE);	//关闭DMA4第四通道
	/*!< DMA2 Channel4 Config */
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&SDIO->FIFO;		//外设地址
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)BufferDST;				//目标地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;						//传输方向，从存储器读
	DMA_InitStructure.DMA_BufferSize = BufferSize / 4;						//数据传输数量，1/4缓存大小
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;		//外设地址不自增
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;					//使能存储目标地址自增
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;	//外设数据大小为字，32位
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;			//DMA2数据大小为字，32位
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;							//不循环
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;						//通道优先级高
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;							//非存储器至存储器模式
	DMA_Init(DMA2_Channel4, &DMA_InitStructure);
	/*!< DMA2 Channel4 enable */
	DMA_Cmd(DMA2_Channel4, ENABLE);  
}
/**********************************************************
* 函数功能 ---> SDIO初始化
* 入口参数 ---> none
* 返回数值 ---> none
* 功能说明 ---> none
**********************************************************/
void SDIO_GPIOInit(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);	//开启外设时钟
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_SDIO | RCC_AHBPeriph_DMA2, ENABLE);	//开启SDIO和DMA时钟
	
	//PC.8 --> SDIO_D0
	//PC.9 --> SDIO_D1
	//PC.10 --> SDIO_D2
	//PC.11 --> SDIO_D3
	//PC.12 --> SDIO_CLK
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_Init(GPIOC, &GPIO_InitStructure);	//初始化GPIO相关结构体

	//PD.2 --> SDIO_CMD
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_Init(GPIOD, &GPIO_InitStructure);	//初始化GPIO相关结构体
}	
/**********************************************************
* 函数功能 ---> SD卡初始化
* 入口参数 ---> none
* 返回数值 ---> 返回卡应答
* 功能说明 ---> none
**********************************************************/
SD_Error SD_Init(void)
{
	SD_Error errorstatus = SD_OK;
  
	SDIO_GPIOInit();	//初始化SDIO端口模式

	MY_NVIC_Init(0, 0, SDIO_IRQn, NVIC_PriorityGroup_2);	//SDIO中断配置

	SDIO_DeInit();	//复位SDIO寄存器值

	//卡的上电过程
	errorstatus = SD_PowerON();
	if(errorstatus != SD_OK)	return(errorstatus);	//操作失败
	//初始化卡到待机状态
	errorstatus = SD_InitializeCards();
	if(errorstatus != SD_OK)	return(errorstatus);	//操作失败
	/*!< Configure the SDIO peripheral */
	/*!< SDIOCLK = HCLK, SDIO_CK = HCLK/(2 + SDIO_TRANSFER_CLK_DIV) */
	/*!< on STM32F2xx devices, SDIOCLK is fixed to 48MHz */  
	SDIO_InitStructure.SDIO_ClockDiv = SDIO_TRANSFER_CLK_DIV;	//提高到传输速度 
	SDIO_InitStructure.SDIO_ClockEdge = SDIO_ClockEdge_Rising;	//上升沿产生SDIO_CLK
	SDIO_InitStructure.SDIO_ClockBypass = SDIO_ClockBypass_Disable;	//关闭旁路
	SDIO_InitStructure.SDIO_ClockPowerSave = SDIO_ClockPowerSave_Disable;	//关闭省电
	SDIO_InitStructure.SDIO_BusWide = SDIO_BusWide_1b;	//保留一路D0
	SDIO_InitStructure.SDIO_HardwareFlowControl = SDIO_HardwareFlowControl_Disable;	//关闭硬件流，只适用于MMC卡
	SDIO_Init(&SDIO_InitStructure);
	//将卡的相关信息保存到结构体中去。CID、CSD、RCA等
	if(errorstatus == SD_OK)	errorstatus = SD_GetCardInfo(&SDCardInfo);
	//选中卡
	if(errorstatus == SD_OK)	errorstatus = SD_SelectDeselect((uint32_t) (SDCardInfo.RCA << 16));
	//设置4b模式
	if(errorstatus == SD_OK)	errorstatus = SD_EnableWideBusOperation(SDIO_BusWide_4b);

	return(errorstatus);
}
/**********************************************************
* 函数功能 ---> 取得卡的传输状态
* 入口参数 ---> none
* 返回数值 ---> 返回卡应答
* 功能说明 ---> none
**********************************************************/
SDTransferState SD_GetStatus(void)
{
	SDCardState cardstate =  SD_CARD_TRANSFER;
	cardstate = SD_GetState();
  
	     if(cardstate == SD_CARD_TRANSFER)	return(SD_TRANSFER_OK);		//传输完成
	else if(cardstate == SD_CARD_ERROR)		return(SD_TRANSFER_ERROR);	//传输出错
  	else									return(SD_TRANSFER_BUSY);	//忙着呢
}
/**********************************************************
* 函数功能 ---> 取得卡的传输状态值
* 入口参数 ---> none
* 返回数值 ---> 返回卡应答
* 功能说明 ---> none
**********************************************************/
SDCardState SD_GetState(void)
{
	uint32_t resp1 = 0;

	if(SD_SendStatus(&resp1) != SD_OK)	return(SD_CARD_ERROR);
	else								return(SDCardState)((resp1 >> 9) & 0x0F);
}
/**********************************************************
* 函数功能 ---> 卡的上电过程
* 入口参数 ---> none
* 返回数值 ---> 返回卡应答
* 功能说明 ---> none
**********************************************************/
SD_Error SD_PowerON(void)
{
	SD_Error errorstatus = SD_OK;
	uint32_t response = 0, count = 0, validvoltage = 0;
	uint32_t SDType = SD_STD_CAPACITY;

	/*!< Power ON Sequence -----------------------------------------------------*/
	/*!< Configure the SDIO peripheral */
	/*!< SDIOCLK = HCLK, SDIO_CK = HCLK/(2 + SDIO_INIT_CLK_DIV) */
	/*!< on STM32F2xx devices, SDIOCLK is fixed to 48MHz */
	/*!< SDIO_CK for initialization should not exceed 400 KHz */  
	SDIO_InitStructure.SDIO_ClockDiv = SDIO_INIT_CLK_DIV;	//初始化速度
	SDIO_InitStructure.SDIO_ClockEdge = SDIO_ClockEdge_Rising;
	SDIO_InitStructure.SDIO_ClockBypass = SDIO_ClockBypass_Disable;
	SDIO_InitStructure.SDIO_ClockPowerSave = SDIO_ClockPowerSave_Disable;
	SDIO_InitStructure.SDIO_BusWide = SDIO_BusWide_1b;
	SDIO_InitStructure.SDIO_HardwareFlowControl = SDIO_HardwareFlowControl_Disable;
	SDIO_Init(&SDIO_InitStructure);

	/*!< Set Power State to ON */
	SDIO_SetPowerState(SDIO_PowerState_ON);

	/*!< Enable SDIO Clock */
	SDIO_ClockCmd(ENABLE);

	//发送至少74个时钟
	for(count = 0;count < 74;count++)
	{
		/*!< CMD0: GO_IDLE_STATE ---------------------------------------------------*/
		/*!< No CMD response required */
		SDIO_CmdInitStructure.SDIO_Argument = 0x0;
		SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_GO_IDLE_STATE;
		SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_No;
		SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
		SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
		SDIO_SendCommand(&SDIO_CmdInitStructure);

		errorstatus = CmdError();
		if(errorstatus != SD_OK)	return errorstatus;	//操作失败
		else	break;	//操作成功了退出
	}
	count = 0;
	/*!< CMD8: SEND_IF_COND ----------------------------------------------------*/
	/*!< Send CMD8 to verify SD card interface operating condition */
	/*!< Argument: - [31:12]: Reserved (shall be set to '0')
                   - [11:8]: Supply Voltage (VHS) 0x1 (Range: 2.7-3.6 V)
                   - [7:0]: Check Pattern (recommended 0xAA) */
	/*!< CMD Response: R7 */
	SDIO_CmdInitStructure.SDIO_Argument = SD_CHECK_PATTERN;
	SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_SEND_IF_COND;
	SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
	SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
	SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
	SDIO_SendCommand(&SDIO_CmdInitStructure);

	errorstatus = CmdResp7Error();
	//有反应说明是V2.0
	if(errorstatus == SD_OK)
	{
		CardType = SDIO_STD_CAPACITY_SD_CARD_V2_0; /*!< SD Card 2.0 */
		SDType = SD_HIGH_CAPACITY;
	}
	else
	{
		/*!< CMD55 */
		SDIO_CmdInitStructure.SDIO_Argument = 0x00;
		SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_APP_CMD;
		SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
		SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
		SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
		SDIO_SendCommand(&SDIO_CmdInitStructure);
		errorstatus = CmdResp1Error(SD_CMD_APP_CMD);
	}
	/*!< CMD55 */
	SDIO_CmdInitStructure.SDIO_Argument = 0x00;
	SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_APP_CMD;
	SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
	SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
	SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
	SDIO_SendCommand(&SDIO_CmdInitStructure);
	errorstatus = CmdResp1Error(SD_CMD_APP_CMD);

	/*!< If errorstatus is Command TimeOut, it is a MMC card */
	/*!< If errorstatus is SD_OK it is a SD card: SD card 2.0 (voltage range mismatch)
         or SD card 1.x */
	if(errorstatus == SD_OK)
	{
		/*!< SD CARD */
		/*!< Send ACMD41 SD_APP_OP_COND with Argument 0x80100000 */
		while((!validvoltage) && (count < SD_MAX_VOLT_TRIAL))
		{
			/*!< SEND CMD55 APP_CMD with RCA as 0 */
			SDIO_CmdInitStructure.SDIO_Argument = 0x00;
			SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_APP_CMD;
			SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
			SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
			SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
			SDIO_SendCommand(&SDIO_CmdInitStructure);

			errorstatus = CmdResp1Error(SD_CMD_APP_CMD);
			if(errorstatus != SD_OK)	return errorstatus;	//操作失败
			
			//发送ACMD41
			SDIO_CmdInitStructure.SDIO_Argument = SD_VOLTAGE_WINDOW_SD | SDType;
			SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SD_APP_OP_COND;
			SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
			SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
			SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
			SDIO_SendCommand(&SDIO_CmdInitStructure);

			errorstatus = CmdResp3Error();
			if(errorstatus != SD_OK)	return errorstatus;	//操作失败
			
			response = SDIO_GetResponse(SDIO_RESP1);
			validvoltage = (((response >> 31) == 1) ? 1 : 0);
			count++;
		}	//end while
		
		if(count >= SD_MAX_VOLT_TRIAL)
		{
			errorstatus = SD_INVALID_VOLTRANGE;
			return(errorstatus);
		}

		if(response &= SD_HIGH_CAPACITY)
		{
			CardType = SDIO_HIGH_CAPACITY_SD_CARD;
		}
	}	//enf if
	else	//MMC card
	{
		CardType=SDIO_MULTIMEDIA_CARD;	  
		//MMC卡,发送CMD0 SDIO_SEND_OP_COND,参数为:0x80FF8000 
		while((!validvoltage)&&(count<SD_MAX_VOLT_TRIAL))
		{	   										   				   
			//发送CMD1,短响应	  
			SDIO_CmdInitStructure.SDIO_Argument = SD_VOLTAGE_WINDOW_MMC;
			SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SEND_OP_COND;
			SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;//短响应
			SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
			SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
			SDIO_SendCommand(&SDIO_CmdInitStructure);

			errorstatus=CmdResp3Error(); 					//等待R3响应   
 			if(errorstatus != SD_OK)	return errorstatus;	//操作失败
			 
			response = SDIO_GetResponse(SDIO_RESP1);			 //得到响应
			validvoltage = (((response>>31)==1)?1:0);
			count++;
		}	//end while
		if(count>=SD_MAX_VOLT_TRIAL)
		{
			errorstatus=SD_INVALID_VOLTRANGE;
			return(errorstatus);
		}	 
	}	//end else
	return(errorstatus);
}
/**********************************************************
* 函数功能 ---> SD卡关电
* 入口参数 ---> none
* 返回数值 ---> 返回卡应答
* 功能说明 ---> none
**********************************************************/
SD_Error SD_PowerOFF(void)
{
	SD_Error errorstatus = SD_OK;

	/*!< Set Power State to OFF */
	SDIO_SetPowerState(SDIO_PowerState_OFF);

	return(errorstatus);
}
/**********************************************************
* 函数功能 ---> SD初始化到待机状态
* 入口参数 ---> none
* 返回数值 ---> 返回卡应答
* 功能说明 ---> none
**********************************************************/
SD_Error SD_InitializeCards(void)
{
	SD_Error errorstatus = SD_OK;
	uint16_t rca = 0x01;
	
	if(SDIO_GetPowerState() == SDIO_PowerState_OFF)	//确保SDIO电源时钟打开
	{
		errorstatus = SD_REQUEST_NOT_APPLICABLE;
		return(errorstatus);
	}

	if(SDIO_SECURE_DIGITAL_IO_CARD != CardType)	//IO卡另外操作
	{
		/*!< Send CMD2 ALL_SEND_CID */
		SDIO_CmdInitStructure.SDIO_Argument = 0x0;
		SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_ALL_SEND_CID;
		SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Long;
		SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
		SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
		SDIO_SendCommand(&SDIO_CmdInitStructure);

		errorstatus = CmdResp2Error();
		if(errorstatus != SD_OK)	return errorstatus;	//操作失败
		//得到CID寄存器信息
		CID_Tab[0] = SDIO_GetResponse(SDIO_RESP1);
		CID_Tab[1] = SDIO_GetResponse(SDIO_RESP2);
		CID_Tab[2] = SDIO_GetResponse(SDIO_RESP3);
		CID_Tab[3] = SDIO_GetResponse(SDIO_RESP4);
	}
	
	if((SDIO_STD_CAPACITY_SD_CARD_V1_1 == CardType) ||  (SDIO_STD_CAPACITY_SD_CARD_V2_0 == CardType) ||  (SDIO_SECURE_DIGITAL_IO_COMBO_CARD == CardType)
      ||  (SDIO_HIGH_CAPACITY_SD_CARD == CardType))
	{
		/*!< Send CMD3 SET_REL_ADDR with argument 0 */
		/*!< SD Card publishes its RCA. */
		SDIO_CmdInitStructure.SDIO_Argument = 0x00;
		SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SET_REL_ADDR;
		SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
		SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
		SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
		SDIO_SendCommand(&SDIO_CmdInitStructure);

		errorstatus = CmdResp6Error(SD_CMD_SET_REL_ADDR, &rca);	//得到卡的相对地址
		if(errorstatus != SD_OK)	return errorstatus;	//操作失败
	}
	
	if(SDIO_SECURE_DIGITAL_IO_CARD != CardType)	
	{
		RCA = rca;

		/*!< Send CMD9 SEND_CSD with argument as card's RCA */
		SDIO_CmdInitStructure.SDIO_Argument = (uint32_t)(rca << 16);
		SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SEND_CSD;
		SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Long;
		SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
		SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
		SDIO_SendCommand(&SDIO_CmdInitStructure);

		errorstatus = CmdResp2Error();
		if(errorstatus != SD_OK)	return errorstatus;	//操作失败
		//得到CSD寄存器信息
		CSD_Tab[0] = SDIO_GetResponse(SDIO_RESP1);
		CSD_Tab[1] = SDIO_GetResponse(SDIO_RESP2);
		CSD_Tab[2] = SDIO_GetResponse(SDIO_RESP3);
		CSD_Tab[3] = SDIO_GetResponse(SDIO_RESP4);
	}
	errorstatus = SD_OK;	//全部都操作成功了
	return(errorstatus);
}
/**********************************************************
* 函数功能 ---> 保存卡的信息到相关结构体中
* 入口参数 ---> none
* 返回数值 ---> 返回卡应答
* 功能说明 ---> none
**********************************************************/
SD_Error SD_GetCardInfo(SD_CardInfo *cardinfo)
{
	SD_Error errorstatus = SD_OK;
	uint8_t tmp = 0;

	cardinfo->CardType = (uint8_t)CardType;	//卡类型
	cardinfo->RCA = (uint16_t)RCA;			//卡相对地址

	/******************************************************
	                 首先读取CSD寄存器数值
	******************************************************/

	/* Byte 0 [ 127 ~ 120 ] */

	tmp = (uint8_t)((CSD_Tab[0] & 0xff000000) >> 24);
	cardinfo->SD_csd.CSDStruct = (tmp & 0xc0) >> 6;		//CSD寄存器结构 ----> 2bits
	cardinfo->SD_csd.SysSpecVersion = (tmp & 0x3c) >> 2;
	cardinfo->SD_csd.Reserved1 = tmp & 0x03;			//RESERVED      ----> 6bits

	/* Byte 1 [ 119 ~ 112 ] */

	tmp = (uint8_t)((CSD_Tab[0] & 0x00ff0000) >> 16);	//data read access-time-1
	cardinfo->SD_csd.TAAC = tmp;						//TAAC[Binary and MLC] ----> 8bits

	/* Byte 2 [ 111 ~ 104 ] */

	tmp = (uint8_t)((CSD_Tab[0] & 0x0000ff00) >> 8);	//data read access-time-2
	cardinfo->SD_csd.NSAC = tmp;						//NSAC                 ----> 8bits

	/* Byte 3 [ 103 ~ 96 ] */

	tmp = (uint8_t)(CSD_Tab[0] & 0x000000ff);
	cardinfo->SD_csd.MaxBusClkFrec = tmp;				//TRAN SPEED ----> 8bits

	/* Byte 4 [ 95 ~ 88 ] */

	tmp = (uint8_t)((CSD_Tab[1] & 0xff000000) >> 24);
	cardinfo->SD_csd.CardComdClasses = tmp << 4;

	/* Byte 5 [ 87 ~ 80 ] */

	tmp = (uint8_t)((CSD_Tab[1] & 0x00ff0000) >> 16);
	cardinfo->SD_csd.CardComdClasses |= (tmp & 0xf0) >> 4;	//CCC               ----> 12bits
	cardinfo->SD_csd.RdBlockLen = tmp & 0x0f;				//----->READ_BL_LEN ----> 4bits

	/* Byte 6 [ 79 ~ 72 ] */

	tmp = (uint8_t)((CSD_Tab[1] & 0x0000ff00) >> 8);
	cardinfo->SD_csd.PartBlockRead = (tmp & 0x80) >> 7;		//READ_BL_PARTIAL      ----> 1bit
	cardinfo->SD_csd.WrBlockMisalign = (tmp & 0x40) >> 6;	//WRITE_BLOCK_MISALIGN ----> 1bit
	cardinfo->SD_csd.RdBlockMisalign = (tmp & 0x20) >> 5;	//READ_BLOCK_MISALIGN  ----> 1bit
	cardinfo->SD_csd.DSRImpl = (tmp & 0x10) >> 4;			//DSR_IMP              ----> 1bit
	cardinfo->SD_csd.Reserved2 = 0;							//RESERVED             ----> 2bits

	/******************************************************
	          下面根据不同类型的卡进行容量计算
	******************************************************/

	/* standard V1.1、V2.0 and MMC card */
	if((CardType==SDIO_STD_CAPACITY_SD_CARD_V1_1)||(CardType==SDIO_STD_CAPACITY_SD_CARD_V2_0)||(SDIO_MULTIMEDIA_CARD==CardType))
	{
		cardinfo->SD_csd.DeviceSize = (tmp & 0x03) << 10;

		/* Byte 7 [ 71 ~ 64 ] */
		
		tmp = (uint8_t)(CSD_Tab[1] & 0x000000ff);
		cardinfo->SD_csd.DeviceSize |= (tmp) << 2;
		
		/* Byte 8 [ 63 ~ 56 ] */
		
		tmp = (uint8_t)((CSD_Tab[2] & 0xff000000) >> 24);	
		cardinfo->SD_csd.DeviceSize |= (tmp & 0xc0) >> 6;			//----->C_SIZE   ----> 12bits

		cardinfo->SD_csd.MaxRdCurrentVDDMin = (tmp & 0x38) >> 3;	//VDD_R_CURR_MIN ----> 3bits
		cardinfo->SD_csd.MaxRdCurrentVDDMax = (tmp & 0x07);			//VDD_R_CURR_MAX ----> 3bits

		/* Byte 9 [ 55 ~ 48 ] */

		tmp = (uint8_t)((CSD_Tab[2] &0x00ff0000) >> 16);
		cardinfo->SD_csd.MaxWrCurrentVDDMin = (tmp & 0xe0) >> 5;	//VDD_W_CURR_MIN ----> 3bits
		cardinfo->SD_csd.MaxWrCurrentVDDMax = (tmp & 0x1c) >> 2;	//VDD_W_CURR_MAX ----> 3bits

		cardinfo->SD_csd.DeviceSizeMul = (tmp & 0x03) << 1;

		/* Byte 10 [ 47 ~ 40 ] */

		tmp = (uint8_t)((CSD_Tab[2] & 0x0000ff00) >> 8);
		cardinfo->SD_csd.DeviceSizeMul |= (tmp & 0x80) >> 7;	//C_SIZE_MULT

		/* 卡容量计算 */
		//计算公式: memory capacity = BLOCKNR * BLOCK_LEN
        //                  BLOCKNR = (C_SIZE + 1) * MULT
        //                     MULT = 2 ^ (C_SIZE_MULT + 2) ---> C_SIZE_MULT < 8
        //                BLOCK_LEN = 2 ^ READ_BL_LEN  ---> READ_BL_LEN < 12

		cardinfo->CardCapacity = (cardinfo->SD_csd.DeviceSize + 1) ;
		cardinfo->CardCapacity *= (1 << (cardinfo->SD_csd.DeviceSizeMul + 2));
		cardinfo->CardBlockSize = 1 << (cardinfo->SD_csd.RdBlockLen);			//卡块大小
		cardinfo->CardCapacity *= cardinfo->CardBlockSize;						//卡容量
		cardinfo->CardCapacity = cardinfo->CardCapacity >> 20;	//单位MB

	}	//end standard V1.1、V2.0 and MMC card
	/* High Capacity Card */
	else if(CardType == SDIO_HIGH_CAPACITY_SD_CARD)
	{
		/* Byte 7 [ 71 ~ 64 ] */

		tmp = (uint8_t)(CSD_Tab[1] & 0x000000ff);
		cardinfo->SD_csd.DeviceSize = (tmp & 0x3f) << 16;

		/* Byte 8 [ 63 ~ 56 ] */

		tmp = (uint8_t)((CSD_Tab[2] & 0xff000000) >> 24);
		cardinfo->SD_csd.DeviceSize |= (tmp << 8);

		/* Byte 9 [ 55 ~ 48 ] */

		tmp = (uint8_t)((CSD_Tab[2] & 0x00ff0000) >> 16);
		cardinfo->SD_csd.DeviceSize |= tmp;					//C_SIZE ----> 24bits

		/* Byte 10 [ 47 ~ 40 ] */

		tmp = (uint8_t)((CSD_Tab[2] & 0x0000ff00) >> 8);

		/* 卡容量计算 */
		//计算公式: memory capacity = (C_SIZE + 1) * 512byte

		cardinfo->CardCapacity=(long long)(cardinfo->SD_csd.DeviceSize+1)*512*1024;	//卡容量
		cardinfo->CardBlockSize = 512;												//卡块大小固定为512byte
		cardinfo->CardCapacity = cardinfo->CardCapacity >> 20;	//单位MB

	}	//end High Capacity Card
	/******************************************************
	                    结束卡容量计算
	******************************************************/

	cardinfo->SD_csd.EraseGrSize = (tmp & 0x40) >> 6;	//ERASE_BLOCK_EN ----> 1bit
	cardinfo->SD_csd.EraseGrMul = (tmp & 0x3f) << 1;

	/* Byte 11 [ 39 ~ 32 ] */

	tmp = (uint8_t)(CSD_Tab[2] & 0x000000ff);
	cardinfo->SD_csd.EraseGrMul |= (tmp & 0x80) >> 7;	//ERASE_GROUP_MUL ----> 8bits
	cardinfo->SD_csd.WrProtectGrSize = (tmp & 0x7f);	//WP_GRP_SIZE     ----> 7bits

	/* Byte 12 [ 31 ~ 24 ] */

	tmp = (uint8_t)((CSD_Tab[3] & 0xff000000) >> 24);
	cardinfo->SD_csd.WrProtectGrEnable = (tmp & 0x80) >> 7;	//WP_GRP_ENABLE      ----> 1bit
	cardinfo->SD_csd.ManDeflECC = (tmp & 0x60) >> 5;		//RMCC               ----> 2bits
	cardinfo->SD_csd.WrSpeedFact = (tmp & 0x1c) >> 2;		//write speed factor ----> 3bits
	cardinfo->SD_csd.MaxWrBlockLen = (tmp & 0x03);

	/* Byte 13 [ 23 ~ 16 ] */

	tmp = (uint8_t)((CSD_Tab[3] & 0x00ff0000) >> 16);
	cardinfo->SD_csd.MaxWrBlockLen |= (tmp & 0xc0) >> 6;		//WRITE_BLOCK_LEN.MAX ----> 4bits
	cardinfo->SD_csd.WriteBlockPaPartial = (tmp & 0x20) >> 5;	//WRITE_BLOCK_PARTIAL ----> 1bit
	cardinfo->SD_csd.Reserved3 = 0;								//RESEVED             ----> 5bits
	cardinfo->SD_csd.ContentProtectAppli = (tmp & 0x01);

	/* Byte 14 [ 15 ~ 8 ] */

	tmp = (uint8_t)((CSD_Tab[3] & 0x0000ff00) >> 8);
	cardinfo->SD_csd.FileFormatGrouop = (tmp & 0x80) >> 7;	//FILE_FORMAT_GRP    ----> 1bit
	cardinfo->SD_csd.CopyFlag = (tmp & 0x40) >> 6;			//COPY_FLAG          ----> 1bit
	cardinfo->SD_csd.PermWrProtect = (tmp & 0x20) >> 5;		//PERM_WRITE_PROTECT ----> 1bit
	cardinfo->SD_csd.TempWrProtect = (tmp & 0x10) >> 4;		//TMP_WRITE_PROTECT  ----> 1bit
	cardinfo->SD_csd.FileFormat = (tmp & 0x0c) >> 2;		//FILE_FORMAT        ----> 2bits
	cardinfo->SD_csd.ECC = (tmp & 0x03);					//ECC                ----> 2bits

	/* Byte 15 [ 7 ~ 0 ] */

	tmp = (uint8_t)(CSD_Tab[3] & 0x000000ff);
	cardinfo->SD_csd.CSD_CRC = (tmp & 0xfe) >> 1;			//CSD_CRC  ----> 7bits
	cardinfo->SD_csd.Reserved4 = 1;							//always 1 ----> 1bit

	/******************************************************
	                 结束CSD寄存器数值获取
	******************************************************/

	/******************************************************
	                 其次读取CID寄存器数值
	******************************************************/

	/* Byte 0 [ 127 ~ 120 ] */
	
	tmp = (uint8_t)((CID_Tab[0] & 0xff000000) >> 24);
	cardinfo->SD_cid.ManufacturerID = tmp;				//MID ----> 8bits

	/* Byte 1 [ 119 ~ 112 ] */

	tmp = (uint8_t)((CID_Tab[0] & 0x00ff0000) >> 16);
	cardinfo->SD_cid.OEM_AppliID = tmp << 8;

	/* Byte 2 [ 111 ~ 104 ] */

	tmp = (uint8_t)((CID_Tab[0] & 0x0000ff00) >> 8);
	cardinfo->SD_cid.OEM_AppliID |= tmp;				//OEM/APPLICATION ID ----> 16bits

	/* Byte 3 [ 103 ~ 96 ] */

	tmp = (uint8_t)(CID_Tab[0] & 0x000000ff);
	cardinfo->SD_cid.ProdName1 = (tmp << 24);			//PRODUCT NAME = PRODUCT NAME1 + PRODUCT NAME2 ----> 40bits

	/* Byte 4 [ 95 ~ 88 ] */

	tmp = (uint8_t)((CID_Tab[1] & 0xff000000) >> 24);
	cardinfo->SD_cid.ProdName1 |= (tmp << 16);

	/* Byte 5 [ 87 ~ 80 ] */

	tmp = (uint8_t)((CID_Tab[1] & 0x00ff0000) >> 16);
	cardinfo->SD_cid.ProdName1 |= (tmp << 8);

	/* Byte 6 [ 79 ~ 72 ] */

	tmp = (uint8_t)((CID_Tab[1] & 0x0000ff00) >> 8);
	cardinfo->SD_cid.ProdName1 |= tmp;					//PRODUCT NAME1 ----> 32bits

	/* Byte 7 [ 71 ~ 64 ] */

	tmp = (uint8_t)(CID_Tab[1] & 0x000000ff);
	cardinfo->SD_cid.ProdName2 = tmp;					//PRODUCT NAME2 ----> 8bits

	/* Byte 8 [ 63 ~ 56 ] */

	tmp = (uint8_t)((CID_Tab[2] & 0xff000000) >> 24);
	cardinfo->SD_cid.ProdRev = tmp;						//PRODUCT REVISION ----> 8bits

	/* Byte 9 [ 55 ~ 48 ] */

	tmp = (uint8_t)((CID_Tab[2] & 0x00ff0000) >> 16);
	cardinfo->SD_cid.ProdSN = (tmp << 24);

	/* Byte 10 [ 47 ~ 40 ] */

	tmp = (uint8_t)((CID_Tab[2] & 0x0000ff00) >> 8);
	cardinfo->SD_cid.ProdSN |= (tmp << 16);

	/* Byte 11 [ 39 ~ 32 ] */

	tmp = (uint8_t)(CID_Tab[2] & 0x000000ff);
	cardinfo->SD_cid.ProdSN |= (tmp << 8);

	/* Byte 12 [ 31 ~ 24 ] */

	tmp = (uint8_t)((CID_Tab[3] & 0xff000000) >> 24);
	cardinfo->SD_cid.ProdSN |= tmp;						//PRODUCT SERIAL NUMBER ----> 32bits

	/* Byte 13 [ 23 ~ 16 ] */

	tmp = (uint8_t)((CID_Tab[3] & 0x00ff0000) >> 16);
	cardinfo->SD_cid.Reserved1 = (tmp & 0xf0) >> 4;		//RESEVED ----> 4bits
	cardinfo->SD_cid.ManufactDate = (tmp & 0x0f) << 8;

	/* Byte 14 [ 15 ~ 8 ] */

	tmp = (uint8_t)((CID_Tab[3] & 0x0000ff00) >> 8);
	cardinfo->SD_cid.ManufactDate |= tmp;				//MDT ----> 12bits

	/* Byte 15 [ 7 ~ 0 ] */

	tmp = (uint8_t)(CID_Tab[3] & 0x000000ff);
	cardinfo->SD_cid.CID_CRC = (tmp & 0xfe) >> 1;		//CID_CRC  ----> 7bits
	cardinfo->SD_cid.Reserved2 = 1;						//always 1 ----> 1bit

	/******************************************************
	                 结束CID寄存器数值获取
	******************************************************/

	return(errorstatus);	//返回应答信息供后面程序判断
}
/**********************************************************
* 函数功能 ---> 读取卡状态信息并保存到结构体中
* 入口参数 ---> none
* 返回数值 ---> 返回卡应答
* 功能说明 ---> none
**********************************************************/
SD_Error SD_GetCardStatus(SD_CardStatus *cardstatus)
{
	SD_Error errorstatus = SD_OK;
	uint8_t tmp = 0;

	errorstatus = SD_SendSDStatus((uint32_t *)SDSTATUS_Tab);
	if(errorstatus != SD_OK)	return errorstatus;	//操作失败

	/*!< Byte 0 */
	tmp = (uint8_t)((SDSTATUS_Tab[0] & 0xC0) >> 6);
	cardstatus->DAT_BUS_WIDTH = tmp;

	/*!< Byte 0 */
	tmp = (uint8_t)((SDSTATUS_Tab[0] & 0x20) >> 5);
	cardstatus->SECURED_MODE = tmp;

	/*!< Byte 2 */
	tmp = (uint8_t)((SDSTATUS_Tab[2] & 0xFF));
	cardstatus->SD_CARD_TYPE = tmp << 8;

	/*!< Byte 3 */
	tmp = (uint8_t)((SDSTATUS_Tab[3] & 0xFF));
	cardstatus->SD_CARD_TYPE |= tmp;

	/*!< Byte 4 */
	tmp = (uint8_t)(SDSTATUS_Tab[4] & 0xFF);
	cardstatus->SIZE_OF_PROTECTED_AREA = tmp << 24;

	/*!< Byte 5 */
	tmp = (uint8_t)(SDSTATUS_Tab[5] & 0xFF);
	cardstatus->SIZE_OF_PROTECTED_AREA |= tmp << 16;

	/*!< Byte 6 */
	tmp = (uint8_t)(SDSTATUS_Tab[6] & 0xFF);
	cardstatus->SIZE_OF_PROTECTED_AREA |= tmp << 8;

	/*!< Byte 7 */
	tmp = (uint8_t)(SDSTATUS_Tab[7] & 0xFF);
	cardstatus->SIZE_OF_PROTECTED_AREA |= tmp;

	/*!< Byte 8 */
	tmp = (uint8_t)((SDSTATUS_Tab[8] & 0xFF));
	cardstatus->SPEED_CLASS = tmp;

	/*!< Byte 9 */
	tmp = (uint8_t)((SDSTATUS_Tab[9] & 0xFF));
	cardstatus->PERFORMANCE_MOVE = tmp;

	/*!< Byte 10 */
	tmp = (uint8_t)((SDSTATUS_Tab[10] & 0xF0) >> 4);
	cardstatus->AU_SIZE = tmp;

	/*!< Byte 11 */
	tmp = (uint8_t)(SDSTATUS_Tab[11] & 0xFF);
	cardstatus->ERASE_SIZE = tmp << 8;

	/*!< Byte 12 */
	tmp = (uint8_t)(SDSTATUS_Tab[12] & 0xFF);
	cardstatus->ERASE_SIZE |= tmp;

	/*!< Byte 13 */
	tmp = (uint8_t)((SDSTATUS_Tab[13] & 0xFC) >> 2);
	cardstatus->ERASE_TIMEOUT = tmp;

	/*!< Byte 13 */
	tmp = (uint8_t)((SDSTATUS_Tab[13] & 0x3));
	cardstatus->ERASE_OFFSET = tmp;
 
	return(errorstatus);
}
/**********************************************************
* 函数功能 ---> 开启宽总线模式
* 入口参数 ---> WideMode：总线模式参数
*               SDIO_BusWide_8b: 8-bit data transfer (Only for MMC)
*               SDIO_BusWide_4b: 4-bit data transfer
*               SDIO_BusWide_1b: 1-bit data transfer
* 返回数值 ---> 返回卡应答
* 功能说明 ---> none
**********************************************************/
SD_Error SD_EnableWideBusOperation(uint32_t WideMode)
{
	SD_Error errorstatus = SD_OK;

	/*!< MMC Card doesn't support this feature */
	if(SDIO_MULTIMEDIA_CARD == CardType)
	{
		errorstatus = SD_UNSUPPORTED_FEATURE;
		return(errorstatus);
	}

	else if((SDIO_STD_CAPACITY_SD_CARD_V1_1 == CardType) || (SDIO_STD_CAPACITY_SD_CARD_V2_0 == CardType) || (SDIO_HIGH_CAPACITY_SD_CARD == CardType))
	{
		if(SDIO_BusWide_8b == WideMode)
		{
			errorstatus = SD_UNSUPPORTED_FEATURE;
			return(errorstatus);
		}
		else if(SDIO_BusWide_4b == WideMode)
		{
			errorstatus = SDEnWideBus(ENABLE);
			if(errorstatus == SD_OK)
			{
				/*!< Configure the SDIO peripheral */
				SDIO_InitStructure.SDIO_ClockDiv = SDIO_TRANSFER_CLK_DIV; 
				SDIO_InitStructure.SDIO_ClockEdge = SDIO_ClockEdge_Rising;
				SDIO_InitStructure.SDIO_ClockBypass = SDIO_ClockBypass_Disable;
				SDIO_InitStructure.SDIO_ClockPowerSave = SDIO_ClockPowerSave_Disable;
				SDIO_InitStructure.SDIO_BusWide = SDIO_BusWide_4b;
				SDIO_InitStructure.SDIO_HardwareFlowControl = SDIO_HardwareFlowControl_Disable;
				SDIO_Init(&SDIO_InitStructure);
			}
		}
		else	//提高速度，保留1b总线宽度
		{
			errorstatus = SDEnWideBus(DISABLE);
			if(errorstatus == SD_OK)
			{
				/*!< Configure the SDIO peripheral */
				SDIO_InitStructure.SDIO_ClockDiv = SDIO_TRANSFER_CLK_DIV; 
				SDIO_InitStructure.SDIO_ClockEdge = SDIO_ClockEdge_Rising;
				SDIO_InitStructure.SDIO_ClockBypass = SDIO_ClockBypass_Disable;
				SDIO_InitStructure.SDIO_ClockPowerSave = SDIO_ClockPowerSave_Disable;
				SDIO_InitStructure.SDIO_BusWide = SDIO_BusWide_1b;
				SDIO_InitStructure.SDIO_HardwareFlowControl = SDIO_HardwareFlowControl_Disable;
				SDIO_Init(&SDIO_InitStructure);
			}
		}
	}
	return(errorstatus);
}
/**********************************************************
* 函数功能 ---> 选中或者取消卡在线
* 入口参数 ---> addr：卡的相对地址
* 返回数值 ---> 返回卡应答
* 功能说明 ---> none
**********************************************************/
SD_Error SD_SelectDeselect(uint32_t addr)
{
	SD_Error errorstatus = SD_OK;

	/*!< Send CMD7 SDIO_SEL_DESEL_CARD */
	SDIO_CmdInitStructure.SDIO_Argument =  addr;
	SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SEL_DESEL_CARD;
	SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
	SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
	SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
	SDIO_SendCommand(&SDIO_CmdInitStructure);

	errorstatus = CmdResp1Error(SD_CMD_SEL_DESEL_CARD);

	return(errorstatus);
}
/**********************************************************
* 函数功能 ---> 读取一个block数据
* 入口参数 ---> *readbuff：读取到数据缓存
*               ReadAddr：读取扇区起始地址
*               BlockSize：读取block大小
* 返回数值 ---> 返回卡应答
* 功能说明 ---> none
**********************************************************/
SD_Error SD_ReadBlock(uint8_t *readbuff, uint32_t ReadAddr, uint16_t BlockSize)
{
	SD_Error errorstatus = SD_OK;
	uint8_t power = 0;

#if defined (SD_POLLING_MODE)
 
	uint32_t count = 0, *tempbuff = (uint32_t *)readbuff;	//查询模式用到定义

#endif

	TransferError = SD_OK;
	TransferEnd = 0;
	StopCondition = 0;		//单块读取不需要发送停止命令

	SDIO->DCTRL = 0x0;	//数据控制寄存器清零，关闭DMA

	if(CardType == SDIO_HIGH_CAPACITY_SD_CARD)
	{
		BlockSize = 512;
		ReadAddr /= 512;
	}
	//清除CPSM状态机
	SDIO_DataInitStructure.SDIO_DataTimeOut = SD_DATATIMEOUT;
	SDIO_DataInitStructure.SDIO_DataLength = 0;
	SDIO_DataInitStructure.SDIO_DataBlockSize = 0;
	SDIO_DataInitStructure.SDIO_TransferDir = SDIO_TransferDir_ToCard;
	SDIO_DataInitStructure.SDIO_TransferMode = SDIO_TransferMode_Block;
	SDIO_DataInitStructure.SDIO_DPSM = SDIO_DPSM_Enable;
	SDIO_DataConfig(&SDIO_DataInitStructure);

	if(SDIO->RESP1&SD_CARD_LOCKED)	return SD_LOCK_UNLOCK_FAILED;//卡锁了

	/* 设置卡的块大小 */
	/* 主要是发送CMD16命令去设置 */
	/* SDSC卡，可以设置块的大小 */
	/* SDHC卡，块的大小恒为512byte，不受CMD16影响 */
	if((BlockSize>0)&&(BlockSize<=2048)&&((BlockSize&(BlockSize-1))==0))
	{
		power = convert_from_bytes_to_power_of_two(BlockSize);	    	   
		//发送CMD16+设置数据长度为blksize,短响应 	 
		SDIO_CmdInitStructure.SDIO_Argument = (uint32_t) BlockSize;
		SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SET_BLOCKLEN;
		SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;   //r1
		SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
		SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
		SDIO_SendCommand(&SDIO_CmdInitStructure);
				
		errorstatus=CmdResp1Error(SD_CMD_SET_BLOCKLEN);	//等待R1响应   
		if(errorstatus != SD_OK)	return errorstatus;	//操作失败	 
	}
	else	return(SD_INVALID_PARAMETER);
	/******************************************************
	                   结束卡块大小设置
	******************************************************/

	//设置SDIO相关寄存器
	SDIO_DataInitStructure.SDIO_DataTimeOut = SD_DATATIMEOUT;
	SDIO_DataInitStructure.SDIO_DataLength = BlockSize;
	SDIO_DataInitStructure.SDIO_DataBlockSize = power << 4;	//(uint32_t) 9 << 4;
	SDIO_DataInitStructure.SDIO_TransferDir = SDIO_TransferDir_ToSDIO;	//卡 --> SDIO主机
	SDIO_DataInitStructure.SDIO_TransferMode = SDIO_TransferMode_Block;
	SDIO_DataInitStructure.SDIO_DPSM = SDIO_DPSM_Enable;
	SDIO_DataConfig(&SDIO_DataInitStructure);

	/*!< Send CMD17 READ_SINGLE_BLOCK */
	SDIO_CmdInitStructure.SDIO_Argument = (uint32_t)ReadAddr;
	SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_READ_SINGLE_BLOCK;
	SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
	SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
	SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
	SDIO_SendCommand(&SDIO_CmdInitStructure);

	errorstatus = CmdResp1Error(SD_CMD_READ_SINGLE_BLOCK);
	if(errorstatus != SD_OK)	return errorstatus;	//操作失败

#if defined (SD_POLLING_MODE)	//查询模式

	/*!< In case of single block transfer, no need of stop transfer at all.*/
	/*!< Polling mode */
	 while(!(SDIO->STA &(SDIO_FLAG_RXOVERR | SDIO_FLAG_DCRCFAIL | SDIO_FLAG_DTIMEOUT | SDIO_FLAG_DBCKEND | SDIO_FLAG_STBITERR)))
	{
		if(SDIO_GetFlagStatus(SDIO_FLAG_RXFIFOHF) != RESET)
		{
			for(count = 0; count < 8; count++)
			{
				*(tempbuff + count) = SDIO_ReadData();
			}
			tempbuff += 8;
		}
	}

	if(SDIO_GetFlagStatus(SDIO_FLAG_DTIMEOUT) != RESET)
	{
		SDIO_ClearFlag(SDIO_FLAG_DTIMEOUT);
		errorstatus = SD_DATA_TIMEOUT;
		return(errorstatus);
	}
	else if(SDIO_GetFlagStatus(SDIO_FLAG_DCRCFAIL) != RESET)
	{
		SDIO_ClearFlag(SDIO_FLAG_DCRCFAIL);
		errorstatus = SD_DATA_CRC_FAIL;
		return(errorstatus);
	}
	else if(SDIO_GetFlagStatus(SDIO_FLAG_RXOVERR) != RESET)
	{
		SDIO_ClearFlag(SDIO_FLAG_RXOVERR);
		errorstatus = SD_RX_OVERRUN;
		return(errorstatus);
	}
	else if(SDIO_GetFlagStatus(SDIO_FLAG_STBITERR) != RESET)
	{
		SDIO_ClearFlag(SDIO_FLAG_STBITERR);
		errorstatus = SD_START_BIT_ERR;
		return(errorstatus);
	}
	while(SDIO_GetFlagStatus(SDIO_FLAG_RXDAVL) != RESET)
	{
		*tempbuff = SDIO_ReadData();
		tempbuff++;
	}
	 
	/*!< Clear all the static flags */
	SDIO_ClearFlag(SDIO_STATIC_FLAGS);

#elif defined (SD_DMA_MODE)	//DMA传输模式

	SDIO_ITConfig(SDIO_IT_RXOVERR|SDIO_IT_DTIMEOUT|SDIO_IT_DCRCFAIL|SDIO_IT_DATAEND, ENABLE);
	SDIO_DMACmd(ENABLE);
	SD_DMA_RxConfig((uint32_t *)readbuff, BlockSize);

	SD_WaitReadOperation();//循环查询dma传输是否结束	
	while(SD_GetStatus() != SD_TRANSFER_OK);

#endif

	return(errorstatus);
}
/**********************************************************
* 函数功能 ---> 读取多个block数据
* 入口参数 ---> *readbuff：读取到数据缓存
*               ReadAddr：读取扇区起始地址
*               BlockSize：读取block大小
*               NumberOfBlocks：读取块的数目
* 返回数值 ---> 返回卡应答
* 功能说明 ---> none
**********************************************************/
SD_Error SD_ReadMultiBlocks(uint8_t *readbuff, uint32_t ReadAddr, uint16_t BlockSize, uint32_t NumberOfBlocks)
{
	SD_Error errorstatus = SD_OK;
	uint32_t timeout = 0;
	uint8_t power = 0;

	TransferError = SD_OK;
	TransferEnd = 0;
	StopCondition = 1;		//读取多个块需要发送停止命令

	SDIO->DCTRL = 0x0;	//数据控制寄存器清零，关闭DMA

	if(CardType == SDIO_HIGH_CAPACITY_SD_CARD)
	{
		BlockSize = 512;
		ReadAddr /= 512;
	}

	//清除CPSM状态机
	SDIO_DataInitStructure.SDIO_DataTimeOut = SD_DATATIMEOUT;
	SDIO_DataInitStructure.SDIO_DataLength = 0;
	SDIO_DataInitStructure.SDIO_DataBlockSize = 0;
	SDIO_DataInitStructure.SDIO_TransferDir = SDIO_TransferDir_ToCard;
	SDIO_DataInitStructure.SDIO_TransferMode = SDIO_TransferMode_Block;
	SDIO_DataInitStructure.SDIO_DPSM = SDIO_DPSM_Enable;
	SDIO_DataConfig(&SDIO_DataInitStructure);

	if(SDIO->RESP1&SD_CARD_LOCKED)return SD_LOCK_UNLOCK_FAILED;//卡锁了

	/*!< Set Block Size for Card */
	/* 设置卡的块大小 */
	/* 主要是发送CMD16命令去设置 */
	/* SDSC卡，可以设置块的大小 */
	/* SDHC卡，块的大小恒位512byte，不受CMD16影响 */
	if((BlockSize>0)&&(BlockSize<=2048)&&((BlockSize&(BlockSize-1))==0))
	{
		power = convert_from_bytes_to_power_of_two(BlockSize);	    	   
		//发送CMD16+设置数据长度为blksize,短响应 	 
		SDIO_CmdInitStructure.SDIO_Argument = (uint32_t) BlockSize;
		SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SET_BLOCKLEN;
		SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;   //r1
		SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
		SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
		SDIO_SendCommand(&SDIO_CmdInitStructure);
				
		errorstatus=CmdResp1Error(SD_CMD_SET_BLOCKLEN);	//等待R1响应   
		if(errorstatus != SD_OK)	return errorstatus;	//操作失败	 
	}
	else	return(SD_INVALID_PARAMETER);
	/******************************************************
	                   结束卡块大小设置
	******************************************************/

	/* 判断是否超过接收的最大缓冲 */
	if(NumberOfBlocks * BlockSize > SD_MAX_DATA_LENGTH)
	{
		errorstatus = SD_INVALID_PARAMETER;	//超过了，返回错误信息
		return errorstatus;
	}
	//没超过
	SDIO_DataInitStructure.SDIO_DataTimeOut = SD_DATATIMEOUT;
	SDIO_DataInitStructure.SDIO_DataLength = NumberOfBlocks * BlockSize;
	SDIO_DataInitStructure.SDIO_DataBlockSize = power<<4;	//(uint32_t) 9 << 4;
	SDIO_DataInitStructure.SDIO_TransferDir = SDIO_TransferDir_ToSDIO;	//卡 --> SDIO主机
	SDIO_DataInitStructure.SDIO_TransferMode = SDIO_TransferMode_Block;
	SDIO_DataInitStructure.SDIO_DPSM = SDIO_DPSM_Enable;
	SDIO_DataConfig(&SDIO_DataInitStructure);

	/*!< Send CMD18 READ_MULT_BLOCK with argument data address */
	SDIO_CmdInitStructure.SDIO_Argument = (uint32_t)ReadAddr;
	SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_READ_MULT_BLOCK;
	SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
	SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
	SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
	SDIO_SendCommand(&SDIO_CmdInitStructure);

	errorstatus = CmdResp1Error(SD_CMD_READ_MULT_BLOCK);
	if(errorstatus != SD_OK)	return errorstatus;	//操作失败

	//等待DMA操作完成
	SDIO_ITConfig(SDIO_IT_RXOVERR|SDIO_IT_DTIMEOUT|SDIO_IT_DCRCFAIL|SDIO_IT_DATAEND|SDIO_IT_STBITERR, ENABLE);
	SDIO_DMACmd(ENABLE);	//SDIO DMA使能 
	SD_DMA_RxConfig((uint32_t *)readbuff, (NumberOfBlocks * BlockSize));

	timeout = SDIO_DATATIMEOUT;
	while(((DMA2->ISR&0x2000)==RESET)&&timeout)timeout--;//等待传输完成 
	if(timeout==0)return SD_DATA_TIMEOUT;//超时
	while((TransferEnd==0)&&(TransferError==SD_OK)); 
	if(TransferError!=SD_OK)errorstatus=TransferError;

	return(errorstatus);
}
/**********************************************************
* 函数功能 ---> 写一个block数据
* 入口参数 ---> *writebuff：写入数据缓存
*               WriteAddr：写入扇区起始地址
*               BlockSize：写block大小
* 返回数值 ---> 返回卡应答
* 功能说明 ---> none
**********************************************************/
SD_Error SD_WriteBlock(uint8_t *writebuff, uint32_t WriteAddr, uint16_t BlockSize)
{
	SD_Error errorstatus = SD_OK;
	uint32_t timeout = 0;
	uint32_t cardstatus = 0;
	uint8_t cardstate = 0;
	uint8_t power = 0;
	
#if defined (SD_POLLING_MODE)

	uint32_t bytestransferred = 0, count = 0, restwords = 0;
	uint32_t *tempbuff = (uint32_t *)writebuff;
	uint32_t Datalen = BlockSize;	//总长度(字节)

#endif

	TransferError = SD_OK;
	TransferEnd = 0;
	StopCondition = 0;		//单块写不需要发送停止命令
	
	if(writebuff==NULL)	return SD_INVALID_PARAMETER;//参数错误 

	SDIO->DCTRL = 0x0;	//数据控制寄存器清零，关闭DMA
	
	//清除CPSM状态机
	SDIO_DataInitStructure.SDIO_DataTimeOut = SD_DATATIMEOUT;
	SDIO_DataInitStructure.SDIO_DataLength = 0;
	SDIO_DataInitStructure.SDIO_DataBlockSize = 0;
	SDIO_DataInitStructure.SDIO_TransferDir = SDIO_TransferDir_ToCard;
	SDIO_DataInitStructure.SDIO_TransferMode = SDIO_TransferMode_Block;
	SDIO_DataInitStructure.SDIO_DPSM = SDIO_DPSM_Enable;
	SDIO_DataConfig(&SDIO_DataInitStructure);

	if(SDIO->RESP1&SD_CARD_LOCKED)return SD_LOCK_UNLOCK_FAILED;//卡锁了
	
	if(CardType == SDIO_HIGH_CAPACITY_SD_CARD)
	{
		BlockSize = 512;
		WriteAddr /= 512;
	}
	
	/* 设置卡的块大小 */
	/* 主要是发送CMD16命令去设置 */
	/* SDSC卡，可以设置块的大小 */
	/* SDHC卡，块的大小恒位512byte，不受CMD16影响 */
	if((BlockSize>0)&&(BlockSize<=2048)&&((BlockSize&(BlockSize-1))==0))
	{
		power = convert_from_bytes_to_power_of_two(BlockSize);	    	   
		//发送CMD16+设置数据长度为blksize,短响应 	 
		SDIO_CmdInitStructure.SDIO_Argument = (uint32_t) BlockSize;
		SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SET_BLOCKLEN;
		SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;   //r1
		SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
		SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
		SDIO_SendCommand(&SDIO_CmdInitStructure);
				
		errorstatus=CmdResp1Error(SD_CMD_SET_BLOCKLEN);	//等待R1响应   
		if(errorstatus != SD_OK)	return errorstatus;	//操作失败	 
	}
	else	return(SD_INVALID_PARAMETER);
	/******************************************************
	                   结束卡块大小设置
	******************************************************/	
	
	/*发送CMD13查询卡的状态*/
	SDIO_CmdInitStructure.SDIO_Argument = (uint32_t)RCA<<16;
	SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SEND_STATUS;
	SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
	SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
	SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
	SDIO_SendCommand(&SDIO_CmdInitStructure);

	errorstatus = CmdResp1Error(SD_CMD_SEND_STATUS);
	if(errorstatus != SD_OK)	return errorstatus;	//操作失败
	
	cardstatus = SDIO->RESP1;													  
	timeout = SD_DATATIMEOUT;
   	while(((cardstatus&0x00000100)==0)&&(timeout>0)) 	//检查READY_FOR_DATA位是否置位
	{
		timeout--;
		//发送CMD13,查询卡的状态,短响应  
		SDIO_CmdInitStructure.SDIO_Argument = (uint32_t)RCA<<16;
		SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SEND_STATUS;
		SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
		SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
		SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
		SDIO_SendCommand(&SDIO_CmdInitStructure);
					
		errorstatus=CmdResp1Error(SD_CMD_SEND_STATUS);	//等待R1响应   		   
		if(errorstatus != SD_OK)	return errorstatus;	//操作失败				    
		cardstatus = SDIO->RESP1;													  
	}
	if(timeout==0)	return SD_ERROR;
	
	/*!< Send CMD24 WRITE_SINGLE_BLOCK */
	SDIO_CmdInitStructure.SDIO_Argument = WriteAddr;
	SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_WRITE_SINGLE_BLOCK;
	SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
	SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
	SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
	SDIO_SendCommand(&SDIO_CmdInitStructure);

	errorstatus = CmdResp1Error(SD_CMD_WRITE_SINGLE_BLOCK);
	if(errorstatus != SD_OK)	return errorstatus;	//操作失败
	
	StopCondition=0;		//单块写,不需要发送停止传输指令

	SDIO_DataInitStructure.SDIO_DataTimeOut = SD_DATATIMEOUT;
	SDIO_DataInitStructure.SDIO_DataLength = BlockSize;
	SDIO_DataInitStructure.SDIO_DataBlockSize = power<<4;	//(uint32_t) 9 << 4;
	SDIO_DataInitStructure.SDIO_TransferDir = SDIO_TransferDir_ToCard;	//SDIO主机 --> 卡
	SDIO_DataInitStructure.SDIO_TransferMode = SDIO_TransferMode_Block;
	SDIO_DataInitStructure.SDIO_DPSM = SDIO_DPSM_Enable;
	SDIO_DataConfig(&SDIO_DataInitStructure);	

	/*!< In case of single data block transfer no need of stop command at all */
#if defined (SD_POLLING_MODE)

	while(!(SDIO->STA & (SDIO_FLAG_DBCKEND | SDIO_FLAG_TXUNDERR | SDIO_FLAG_DCRCFAIL | SDIO_FLAG_DTIMEOUT | SDIO_FLAG_STBITERR)))
	{
		if(SDIO_GetFlagStatus(SDIO_FLAG_TXFIFOHE) != RESET)
		{
			if((Datalen - bytestransferred) < 32)
			{
				restwords = ((Datalen - bytestransferred) % 4 == 0) ? ((Datalen - bytestransferred) / 4) : ((Datalen -  bytestransferred) / 4 + 1);
				for(count = 0; count < restwords; count++, tempbuff++, bytestransferred += 4)
				{
					SDIO_WriteData(*tempbuff);
				}
			}
			else
			{
				for(count = 0; count < 8; count++)
				{
					SDIO_WriteData(*(tempbuff + count));
				}
				tempbuff += 8;
				bytestransferred += 32;
			}
		}
	}

	if(SDIO_GetFlagStatus(SDIO_FLAG_DTIMEOUT) != RESET)
	{
		SDIO_ClearFlag(SDIO_FLAG_DTIMEOUT);
		errorstatus = SD_DATA_TIMEOUT;
		return(errorstatus);
	}
	else if(SDIO_GetFlagStatus(SDIO_FLAG_DCRCFAIL) != RESET)
	{
		SDIO_ClearFlag(SDIO_FLAG_DCRCFAIL);
		errorstatus = SD_DATA_CRC_FAIL;
		return(errorstatus);
	}
	else if(SDIO_GetFlagStatus(SDIO_FLAG_TXUNDERR) != RESET)
	{
		SDIO_ClearFlag(SDIO_FLAG_TXUNDERR);
		errorstatus = SD_TX_UNDERRUN;
		return(errorstatus);
	}
	else if(SDIO_GetFlagStatus(SDIO_FLAG_STBITERR) != RESET)
	{
		SDIO_ClearFlag(SDIO_FLAG_STBITERR);
		errorstatus = SD_START_BIT_ERR;
		return(errorstatus);
	}

#elif defined (SD_DMA_MODE)

	TransferError = SD_OK;
	TransferEnd = 0;  		//传输结束标置位，在中断服务置1
	StopCondition = 0;  	//单块写,不需要发送停止传输指令 

	SDIO_ITConfig(SDIO_IT_TXUNDERR|SDIO_IT_DTIMEOUT|SDIO_IT_DCRCFAIL|SDIO_IT_DATAEND|SDIO_IT_STBITERR, ENABLE);
	SD_DMA_TxConfig((uint32_t*)writebuff, BlockSize);
	SDIO_DMACmd(ENABLE);
/*
	timeout = SDIO_DATATIMEOUT;
 	while(((DMA2->ISR&0x2000)==RESET)&&timeout)timeout--;//等待传输完成 
	if(timeout==0)
	{
  		SD_Init();	 					//重新初始化SD卡,可以解决写入死机的问题
		return SD_DATA_TIMEOUT;			//超时	 
 	}
	timeout = SDIO_DATATIMEOUT;
	while((TransferEnd==0)&&(TransferError==SD_OK)&&timeout)timeout--;
 	if(timeout==0)return SD_DATA_TIMEOUT;			//超时	 
	if(TransferError!=SD_OK)return TransferError;
*/
	SD_WaitWriteOperation();//循环查询dma传输是否结束	
	while(SD_GetStatus() != SD_TRANSFER_OK);//Wait until end of DMA transfer
	  
#endif

	SDIO_ClearFlag(SDIO_STATIC_FLAGS);	//清除所有标记
	errorstatus = IsCardProgramming(&cardstate);
	while((errorstatus==SD_OK)&&((cardstate==SD_CARD_PROGRAMMING)||(cardstate==SD_CARD_RECEIVING)))
	{
		errorstatus = IsCardProgramming(&cardstate);
	}

	return(errorstatus);
}
/**********************************************************
* 函数功能 ---> 写多个block数据
* 入口参数 ---> *writebuff：写入数据缓存
*               WriteAddr：写入扇区起始地址
*               BlockSize：写block大小
*               NumberOfBlocks：写入块数目
* 返回数值 ---> 返回卡应答
* 功能说明 ---> none
**********************************************************/
SD_Error SD_WriteMultiBlocks(uint8_t *writebuff, uint32_t WriteAddr, uint16_t BlockSize, uint32_t NumberOfBlocks)
{
	SD_Error errorstatus = SD_OK;
	__IO uint32_t count = 0;
	__IO uint32_t timeout = 0;
	uint8_t cardstate = 0;
	uint8_t power = 0;

	TransferError = SD_OK;
	TransferEnd = 0;
	StopCondition = 1;		//多个块写入操作需要发送停止命令

	SDIO->DCTRL = 0x0;	//数据控制寄存器清零，关闭DMA

	//清除CPSM状态机
	SDIO_DataInitStructure.SDIO_DataTimeOut = SD_DATATIMEOUT;
	SDIO_DataInitStructure.SDIO_DataLength = 0;
	SDIO_DataInitStructure.SDIO_DataBlockSize = 0;
	SDIO_DataInitStructure.SDIO_TransferDir = SDIO_TransferDir_ToCard;
	SDIO_DataInitStructure.SDIO_TransferMode = SDIO_TransferMode_Block;
	SDIO_DataInitStructure.SDIO_DPSM = SDIO_DPSM_Enable;
	SDIO_DataConfig(&SDIO_DataInitStructure);

	if(SDIO->RESP1&SD_CARD_LOCKED)return SD_LOCK_UNLOCK_FAILED;//卡锁了

	if(CardType == SDIO_HIGH_CAPACITY_SD_CARD)
	{
		BlockSize = 512;
		WriteAddr /= 512;
	}

	/* 设置卡的块大小 */
	/* 主要是发送CMD16命令去设置 */
	/* SDSC卡，可以设置块的大小 */
	/* SDHC卡，块的大小恒位512byte，不受CMD16影响 */
	if((BlockSize>0)&&(BlockSize<=2048)&&((BlockSize&(BlockSize-1))==0))
	{
		power = convert_from_bytes_to_power_of_two(BlockSize);	    	   
		//发送CMD16+设置数据长度为blksize,短响应 	 
		SDIO_CmdInitStructure.SDIO_Argument = (uint32_t) BlockSize;
		SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SET_BLOCKLEN;
		SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;   //r1
		SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
		SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
		SDIO_SendCommand(&SDIO_CmdInitStructure);
				
		errorstatus=CmdResp1Error(SD_CMD_SET_BLOCKLEN);	//等待R1响应   
		if(errorstatus != SD_OK)	return errorstatus;	//操作失败	 
	}
	else	return(SD_INVALID_PARAMETER);
	/******************************************************
	                   结束卡块大小设置
	******************************************************/

	/* 计算数据长度是否超出范围 */
	if(NumberOfBlocks * BlockSize > SD_MAX_DATA_LENGTH)
	{
		errorstatus = SD_INVALID_PARAMETER;	//超出范围了，返回错误信息
		return errorstatus;
	}

	/*!< To improve performance */
	SDIO_CmdInitStructure.SDIO_Argument = (uint32_t)(RCA << 16);
	SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_APP_CMD;	//CMD55
	SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
	SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
	SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
	SDIO_SendCommand(&SDIO_CmdInitStructure);

	errorstatus = CmdResp1Error(SD_CMD_APP_CMD);
	if(errorstatus != SD_OK)	return errorstatus;	//操作失败

	//发送CMD23设置块数目
	/*!< To improve performance */
	SDIO_CmdInitStructure.SDIO_Argument = (uint32_t)NumberOfBlocks;
	SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SET_BLOCK_COUNT;
	SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
	SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
	SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
	SDIO_SendCommand(&SDIO_CmdInitStructure);

	errorstatus = CmdResp1Error(SD_CMD_SET_BLOCK_COUNT);
	if(errorstatus != SD_OK)	return errorstatus;	//操作失败

	/*!< Send CMD25 WRITE_MULT_BLOCK with argument data address */
	SDIO_CmdInitStructure.SDIO_Argument = (uint32_t)WriteAddr;
	SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_WRITE_MULT_BLOCK;
	SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
	SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
	SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
	SDIO_SendCommand(&SDIO_CmdInitStructure);

	errorstatus = CmdResp1Error(SD_CMD_WRITE_MULT_BLOCK);
	if(errorstatus != SD_OK)	return errorstatus;	//操作失败

	SDIO_DataInitStructure.SDIO_DataTimeOut = SD_DATATIMEOUT;
	SDIO_DataInitStructure.SDIO_DataLength = NumberOfBlocks * BlockSize;
	SDIO_DataInitStructure.SDIO_DataBlockSize = power<<4;	//(uint32_t) 9 << 4;
	SDIO_DataInitStructure.SDIO_TransferDir = SDIO_TransferDir_ToCard;	//SDIO主机 --> 卡
	SDIO_DataInitStructure.SDIO_TransferMode = SDIO_TransferMode_Block;
	SDIO_DataInitStructure.SDIO_DPSM = SDIO_DPSM_Enable;
	SDIO_DataConfig(&SDIO_DataInitStructure);

	//等待DMA传输完成
	TransferError = SD_OK;
	TransferEnd = 0;
	StopCondition = 1;

	SDIO_ITConfig(SDIO_IT_TXUNDERR|SDIO_IT_DCRCFAIL|SDIO_IT_DTIMEOUT|SDIO_IT_DATAEND|SDIO_IT_STBITERR, ENABLE);
	SDIO_DMACmd(ENABLE);    
	SD_DMA_TxConfig((uint32_t *)writebuff, (NumberOfBlocks * BlockSize));
	
	timeout = SDIO_DATATIMEOUT;
	while(((DMA2->ISR&0x2000)==RESET)&&timeout)timeout--;//等待传输完成 
	if(timeout == 0)	 								//超时
	{									  
  		SD_Init();	 					//重新初始化SD卡,可以解决写入死机的问题
	 	return SD_DATA_TIMEOUT;			//超时	 
	}
	timeout = SDIO_DATATIMEOUT;
	while((TransferEnd==0)&&(TransferError==SD_OK)&&timeout)timeout--;
	if(timeout == 0)	return SD_DATA_TIMEOUT;			//超时	 
	if(TransferError != SD_OK)	return TransferError;
		 
	SDIO_ClearFlag(SDIO_STATIC_FLAGS);	//清除所有标记
	errorstatus = IsCardProgramming(&cardstate);
	while((errorstatus==SD_OK)&&((cardstate==SD_CARD_PROGRAMMING)||(cardstate==SD_CARD_RECEIVING)))
	{
		errorstatus = IsCardProgramming(&cardstate);
	} 

	return(errorstatus);
}
/**********************************************************
* 函数功能 ---> 等待读操作
* 入口参数 ---> none
* 返回数值 ---> 返回卡应答
* 功能说明 ---> none
**********************************************************/
SD_Error SD_WaitReadOperation(void)
{
	SD_Error errorstatus = SD_OK;

	while((SD_DMAEndOfTransferStatus() == RESET) && (TransferEnd == 0) && (TransferError == SD_OK))
	{}

	if(TransferError != SD_OK)	return(TransferError);

	return(errorstatus);
}
/**********************************************************
* 函数功能 ---> 等待写操作
* 入口参数 ---> none
* 返回数值 ---> 返回卡应答
* 功能说明 ---> none
**********************************************************/
SD_Error SD_WaitWriteOperation(void)
{
	SD_Error errorstatus = SD_OK;

	while((SD_DMAEndOfTransferStatus() == RESET) && (TransferEnd == 0) && (TransferError == SD_OK))
	{}

	if(TransferError != SD_OK)	return(TransferError);

	/*!< Clear all the static flags */
	SDIO_ClearFlag(SDIO_STATIC_FLAGS);

	return(errorstatus);
}
/**********************************************************
* 函数功能 ---> 获取传输状态
* 入口参数 ---> none
* 返回数值 ---> 返回卡应答
* 功能说明 ---> none
**********************************************************/
SDTransferState SD_GetTransferState(void)
{
	if(SDIO->STA & (SDIO_FLAG_TXACT | SDIO_FLAG_RXACT))	return(SD_TRANSFER_BUSY);
	else	return(SD_TRANSFER_OK);
}
/**********************************************************
* 函数功能 ---> 发送停止命令
* 入口参数 ---> none
* 返回数值 ---> 返回卡应答
* 功能说明 ---> none
**********************************************************/
SD_Error SD_StopTransfer(void)
{
	SD_Error errorstatus = SD_OK;

	/*!< Send CMD12 STOP_TRANSMISSION  */
	SDIO->ARG = 0x0;
	SDIO->CMD = 0x44C;
	errorstatus = CmdResp1Error(SD_CMD_STOP_TRANSMISSION);

	return(errorstatus);
}
/**********************************************************
* 函数功能 ---> 擦除操作
* 入口参数 ---> startaddr：起始地址
*               endaddr：结束地址
* 返回数值 ---> 返回卡应答
* 功能说明 ---> none
**********************************************************/
SD_Error SD_Erase(uint32_t startaddr, uint32_t endaddr)
{
	SD_Error errorstatus = SD_OK;
	uint32_t delay = 0;
	__IO uint32_t maxdelay = 0;
	uint8_t cardstate = 0;

	/*!< Check if the card coomnd class supports erase command */
	if(((CSD_Tab[1] >> 20) & SD_CCCC_ERASE) == 0)
	{
		errorstatus = SD_REQUEST_NOT_APPLICABLE;
		return(errorstatus);
	}

	maxdelay = 120000 / ((SDIO->CLKCR & 0xFF) + 2);

	if(SDIO_GetResponse(SDIO_RESP1) & SD_CARD_LOCKED)	//卡锁了
	{
		errorstatus = SD_LOCK_UNLOCK_FAILED;
		return(errorstatus);
	}

	if(CardType == SDIO_HIGH_CAPACITY_SD_CARD)
	{
		startaddr /= 512;
		endaddr /= 512;
	}

	/*!< According to sd-card spec 1.0 ERASE_GROUP_START (CMD32) and erase_group_end(CMD33) */
	if((SDIO_STD_CAPACITY_SD_CARD_V1_1 == CardType) || (SDIO_STD_CAPACITY_SD_CARD_V2_0 == CardType) || (SDIO_HIGH_CAPACITY_SD_CARD == CardType))
	{
		/*!< Send CMD32 SD_ERASE_GRP_START with argument as addr  */
		SDIO_CmdInitStructure.SDIO_Argument = startaddr;
		SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SD_ERASE_GRP_START;
		SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
		SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
		SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
		SDIO_SendCommand(&SDIO_CmdInitStructure);

		errorstatus = CmdResp1Error(SD_CMD_SD_ERASE_GRP_START);
		if(errorstatus != SD_OK)	return errorstatus;	//操作失败

		/*!< Send CMD33 SD_ERASE_GRP_END with argument as addr  */
		SDIO_CmdInitStructure.SDIO_Argument = endaddr;
		SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SD_ERASE_GRP_END;
		SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
		SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
		SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
		SDIO_SendCommand(&SDIO_CmdInitStructure);

		errorstatus = CmdResp1Error(SD_CMD_SD_ERASE_GRP_END);
		if(errorstatus != SD_OK)	return errorstatus;	//操作失败
	}
	/*!< Send CMD38 ERASE */
	SDIO_CmdInitStructure.SDIO_Argument = 0;
	SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_ERASE;
	SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
	SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
	SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
	SDIO_SendCommand(&SDIO_CmdInitStructure);

	errorstatus = CmdResp1Error(SD_CMD_ERASE);
	if(errorstatus != SD_OK)	return errorstatus;	//操作失败

	for (delay = 0; delay < maxdelay; delay++)	{}	//延时给卡进行擦除操作

	/*!< Wait till the card is in programming state */
	errorstatus = IsCardProgramming(&cardstate);

	while((errorstatus == SD_OK) && ((SD_CARD_PROGRAMMING == cardstate) || (SD_CARD_RECEIVING == cardstate)))
	{
		errorstatus = IsCardProgramming(&cardstate);
	}

	return(errorstatus);
}
/**********************************************************
* 函数功能 ---> 读取卡状态寄存器
* 入口参数 ---> *pcardstatus：返回状态值缓存
* 返回数值 ---> 返回卡应答
* 功能说明 ---> none
**********************************************************/
SD_Error SD_SendStatus(uint32_t *pcardstatus)
{
	SD_Error errorstatus = SD_OK;

	SDIO->ARG = (uint32_t) RCA << 16;
	SDIO->CMD = 0x44D;
  
	errorstatus = CmdResp1Error(SD_CMD_SEND_STATUS);

	if(errorstatus != SD_OK)	return(errorstatus);

	*pcardstatus = SDIO->RESP1;
	return(errorstatus);
}
/**********************************************************
* 函数功能 ---> 读取SD状态寄存器
* 入口参数 ---> *psdstatus：返回状态值缓存
* 返回数值 ---> 返回卡应答
* 功能说明 ---> none
**********************************************************/
SD_Error SD_SendSDStatus(uint32_t *psdstatus)
{
	SD_Error errorstatus = SD_OK;
	uint32_t count = 0;

	if(SDIO_GetResponse(SDIO_RESP1) & SD_CARD_LOCKED)	//卡锁了
	{
		errorstatus = SD_LOCK_UNLOCK_FAILED;
		return(errorstatus);
	}

	/*!< Set block size for card if it is not equal to current block size for card. */
	SDIO_CmdInitStructure.SDIO_Argument = 64;
	SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SET_BLOCKLEN;
	SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
	SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
	SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
	SDIO_SendCommand(&SDIO_CmdInitStructure);

	errorstatus = CmdResp1Error(SD_CMD_SET_BLOCKLEN);
	if(errorstatus != SD_OK)	return errorstatus;	//操作失败

	/*!< CMD55 */
	SDIO_CmdInitStructure.SDIO_Argument = (uint32_t) RCA << 16;
	SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_APP_CMD;
	SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
	SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
	SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
	SDIO_SendCommand(&SDIO_CmdInitStructure);

	errorstatus = CmdResp1Error(SD_CMD_APP_CMD);
	if(errorstatus != SD_OK)	return errorstatus;	//操作失败

	SDIO_DataInitStructure.SDIO_DataTimeOut = SD_DATATIMEOUT;
	SDIO_DataInitStructure.SDIO_DataLength = 64;
	SDIO_DataInitStructure.SDIO_DataBlockSize = SDIO_DataBlockSize_64b;
	SDIO_DataInitStructure.SDIO_TransferDir = SDIO_TransferDir_ToSDIO;	//卡 --> SDIO主机
	SDIO_DataInitStructure.SDIO_TransferMode = SDIO_TransferMode_Block;
	SDIO_DataInitStructure.SDIO_DPSM = SDIO_DPSM_Enable;
	SDIO_DataConfig(&SDIO_DataInitStructure);

	/*!< Send ACMD13 SD_APP_STAUS  with argument as card's RCA.*/
	SDIO_CmdInitStructure.SDIO_Argument = 0;
	SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SD_APP_STAUS;
	SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
	SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
	SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
	SDIO_SendCommand(&SDIO_CmdInitStructure);

	errorstatus = CmdResp1Error(SD_CMD_SD_APP_STAUS);
	if(errorstatus != SD_OK)	return errorstatus;	//操作失败

	//开始接收数据
	while(!(SDIO->STA &(SDIO_FLAG_RXOVERR | SDIO_FLAG_DCRCFAIL | SDIO_FLAG_DTIMEOUT | SDIO_FLAG_DBCKEND | SDIO_FLAG_STBITERR)))
	{
		if(SDIO_GetFlagStatus(SDIO_FLAG_RXFIFOHF) != RESET)
		{
			for(count = 0; count < 8; count++)
			{
				*(psdstatus + count) = SDIO_ReadData();
			}
			psdstatus += 8;
		}
	}

	if(SDIO_GetFlagStatus(SDIO_FLAG_DTIMEOUT) != RESET)
	{
		SDIO_ClearFlag(SDIO_FLAG_DTIMEOUT);
		errorstatus = SD_DATA_TIMEOUT;
		return(errorstatus);
	}
	else if(SDIO_GetFlagStatus(SDIO_FLAG_DCRCFAIL) != RESET)
	{
		SDIO_ClearFlag(SDIO_FLAG_DCRCFAIL);
		errorstatus = SD_DATA_CRC_FAIL;
		return(errorstatus);
	}
	else if(SDIO_GetFlagStatus(SDIO_FLAG_RXOVERR) != RESET)
	{
		SDIO_ClearFlag(SDIO_FLAG_RXOVERR);
		errorstatus = SD_RX_OVERRUN;
		return(errorstatus);
	}
	else if(SDIO_GetFlagStatus(SDIO_FLAG_STBITERR) != RESET)
	{
		SDIO_ClearFlag(SDIO_FLAG_STBITERR);
		errorstatus = SD_START_BIT_ERR;
		return(errorstatus);
	}

	while(SDIO_GetFlagStatus(SDIO_FLAG_RXDAVL) != RESET)
	{
		*psdstatus = SDIO_ReadData();
		psdstatus++;
	}

	/*!< Clear all the static status flags*/
	SDIO_ClearFlag(SDIO_STATIC_FLAGS);

	return(errorstatus);
}
/**********************************************************
* 函数功能 ---> 检测CMD0执行情况
* 入口参数 ---> none
* 返回数值 ---> 返回卡应答
* 功能说明 ---> none
**********************************************************/
static SD_Error CmdError(void)
{
	SD_Error errorstatus = SD_OK;
	uint32_t timeout;

	timeout = SDIO_CMD0TIMEOUT; /*!< 10000 */

	while((timeout > 0) && (SDIO_GetFlagStatus(SDIO_FLAG_CMDSENT) == RESET))	timeout--;

	if(timeout == 0)
	{
		errorstatus = SD_CMD_RSP_TIMEOUT;
		return(errorstatus);
	}

	/*!< Clear all the static flags */
	SDIO_ClearFlag(SDIO_STATIC_FLAGS);

	return(errorstatus);
}
/**********************************************************
* 函数功能 ---> 检测R7响应情况
* 入口参数 ---> none
* 返回数值 ---> 返回卡应答
* 功能说明 ---> none
**********************************************************/
static SD_Error CmdResp7Error(void)
{
	SD_Error errorstatus = SD_OK;
	uint32_t status;
	uint32_t timeout = SDIO_CMD0TIMEOUT;

	status = SDIO->STA;

	while(!(status & (SDIO_FLAG_CCRCFAIL | SDIO_FLAG_CMDREND | SDIO_FLAG_CTIMEOUT)) && (timeout > 0))
	{
		timeout--;
		status = SDIO->STA;
	}

	if((timeout == 0) || (status & SDIO_FLAG_CTIMEOUT))
	{
		/*!< Card is not V2.0 complient or card does not support the set voltage range */
		errorstatus = SD_CMD_RSP_TIMEOUT;
		SDIO_ClearFlag(SDIO_FLAG_CTIMEOUT);
		return(errorstatus);
	}

	if(status & SDIO_FLAG_CMDREND)
	{
		/*!< Card is SD V2.0 compliant */
		errorstatus = SD_OK;
		SDIO_ClearFlag(SDIO_FLAG_CMDREND);
		return(errorstatus);
	}
	return(errorstatus);
}
/**********************************************************
* 函数功能 ---> 检测R1响应情况
* 入口参数 ---> cmd：命令
* 返回数值 ---> 返回卡应答
* 功能说明 ---> none
**********************************************************/
static SD_Error CmdResp1Error(uint8_t cmd)
{
	while(!(SDIO->STA & (SDIO_FLAG_CCRCFAIL | SDIO_FLAG_CMDREND | SDIO_FLAG_CTIMEOUT)))	{}

	SDIO->ICR = SDIO_STATIC_FLAGS;

	return (SD_Error)(SDIO->RESP1 &  SD_OCR_ERRORBITS);
}
/**********************************************************
* 函数功能 ---> 检测R3响应情况
* 入口参数 ---> none
* 返回数值 ---> 返回卡应答
* 功能说明 ---> none
**********************************************************/
static SD_Error CmdResp3Error(void)
{
	SD_Error errorstatus = SD_OK;
	uint32_t status;

	status = SDIO->STA;

	while(!(status & (SDIO_FLAG_CCRCFAIL | SDIO_FLAG_CMDREND | SDIO_FLAG_CTIMEOUT)))	status = SDIO->STA;

	if(status & SDIO_FLAG_CTIMEOUT)
	{
		errorstatus = SD_CMD_RSP_TIMEOUT;
		SDIO_ClearFlag(SDIO_FLAG_CTIMEOUT);
		return(errorstatus);
	}
	/*!< Clear all the static flags */
	SDIO_ClearFlag(SDIO_STATIC_FLAGS);
	return(errorstatus);
}
/**********************************************************
* 函数功能 ---> 检测R2响应情况
* 入口参数 ---> none
* 返回数值 ---> 返回卡应答
* 功能说明 ---> none
**********************************************************/
static SD_Error CmdResp2Error(void)
{
	SD_Error errorstatus = SD_OK;
	uint32_t status;

	status = SDIO->STA;

	while(!(status & (SDIO_FLAG_CCRCFAIL | SDIO_FLAG_CTIMEOUT | SDIO_FLAG_CMDREND)))	status = SDIO->STA;

	if(status & SDIO_FLAG_CTIMEOUT)
	{
		errorstatus = SD_CMD_RSP_TIMEOUT;
		SDIO_ClearFlag(SDIO_FLAG_CTIMEOUT);
		return(errorstatus);
	}
	else if(status & SDIO_FLAG_CCRCFAIL)
	{
		errorstatus = SD_CMD_CRC_FAIL;
		SDIO_ClearFlag(SDIO_FLAG_CCRCFAIL);
		return(errorstatus);
	}

	/*!< Clear all the static flags */
	SDIO_ClearFlag(SDIO_STATIC_FLAGS);

	return(errorstatus);
}
/**********************************************************
* 函数功能 ---> 检测R6响应情况
* 入口参数 ---> cmd：命令
*               *prca：返回卡相对地址缓存
* 返回数值 ---> 返回卡应答
* 功能说明 ---> none
**********************************************************/
static SD_Error CmdResp6Error(uint8_t cmd, uint16_t *prca)
{
	SD_Error errorstatus = SD_OK;
	uint32_t status;
	uint32_t response_r1;

	status = SDIO->STA;

	while(!(status & (SDIO_FLAG_CCRCFAIL | SDIO_FLAG_CTIMEOUT | SDIO_FLAG_CMDREND)))	status = SDIO->STA;

	if(status & SDIO_FLAG_CTIMEOUT)
	{
		errorstatus = SD_CMD_RSP_TIMEOUT;
		SDIO_ClearFlag(SDIO_FLAG_CTIMEOUT);
		return(errorstatus);
	}
	else if(status & SDIO_FLAG_CCRCFAIL)
	{
		errorstatus = SD_CMD_CRC_FAIL;
		SDIO_ClearFlag(SDIO_FLAG_CCRCFAIL);
		return(errorstatus);
	}

	/*!< Check response received is of desired command */
	if(SDIO_GetCommandResponse() != cmd)
	{
		errorstatus = SD_ILLEGAL_CMD;
		return(errorstatus);
	}

	/*!< Clear all the static flags */
	SDIO_ClearFlag(SDIO_STATIC_FLAGS);

	/*!< We have received response, retrieve it.  */
	response_r1 = SDIO_GetResponse(SDIO_RESP1);

	if(SD_ALLZERO == (response_r1 & (SD_R6_GENERAL_UNKNOWN_ERROR | SD_R6_ILLEGAL_CMD | SD_R6_COM_CRC_FAILED)))
	{
		*prca = (uint16_t) (response_r1 >> 16);
		return(errorstatus);
	}

	if(response_r1 & SD_R6_GENERAL_UNKNOWN_ERROR)	return(SD_GENERAL_UNKNOWN_ERROR);
	if(response_r1 & SD_R6_ILLEGAL_CMD)				return(SD_ILLEGAL_CMD);
	if(response_r1 & SD_R6_COM_CRC_FAILED)			return(SD_COM_CRC_FAILED);

	return(errorstatus);
}
/**********************************************************
* 函数功能 ---> 获取SCR寄存器值
* 入口参数 ---> rca：卡相对地址
*               *pscr：SCR寄存器值缓存
* 返回数值 ---> 返回卡应答
* 功能说明 ---> none
**********************************************************/
static SD_Error FindSCR(uint16_t rca, uint32_t *pscr)
{
	SD_Error errorstatus = SD_OK;
	uint32_t index = 0;
	uint32_t tempscr[2] = {0, 0};

	/*!< Set Block Size To 8 Bytes */
	/*!< Send CMD55 APP_CMD with argument as card's RCA */
	SDIO_CmdInitStructure.SDIO_Argument = (uint32_t)8;
	SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SET_BLOCKLEN;	//CMD16
	SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
	SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
	SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
	SDIO_SendCommand(&SDIO_CmdInitStructure);

	errorstatus = CmdResp1Error(SD_CMD_SET_BLOCKLEN);
	if(errorstatus != SD_OK)	return errorstatus;	//操作失败

	/*!< Send CMD55 APP_CMD with argument as card's RCA */
	SDIO_CmdInitStructure.SDIO_Argument = (uint32_t) RCA << 16;
	SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_APP_CMD;
	SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
	SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
	SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
	SDIO_SendCommand(&SDIO_CmdInitStructure);

	errorstatus = CmdResp1Error(SD_CMD_APP_CMD);
	if(errorstatus != SD_OK)	return errorstatus;	//操作失败

	SDIO_DataInitStructure.SDIO_DataTimeOut = SD_DATATIMEOUT;
	SDIO_DataInitStructure.SDIO_DataLength = 8;
	SDIO_DataInitStructure.SDIO_DataBlockSize = SDIO_DataBlockSize_8b;
	SDIO_DataInitStructure.SDIO_TransferDir = SDIO_TransferDir_ToSDIO;	//卡 --> SDIO主机
	SDIO_DataInitStructure.SDIO_TransferMode = SDIO_TransferMode_Block;
	SDIO_DataInitStructure.SDIO_DPSM = SDIO_DPSM_Enable;
	SDIO_DataConfig(&SDIO_DataInitStructure);

	/******************************************************
	             延时一定时间给CPU处理数据
	******************************************************/

	for(index = 0;index < 20;index++)	{}

	index = 0;

	/*****************************************************/

	/*!< Send ACMD51 SD_APP_SEND_SCR with argument as 0 */
	SDIO_CmdInitStructure.SDIO_Argument = 0x0;
	SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SD_APP_SEND_SCR;
	SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
	SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
	SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
	SDIO_SendCommand(&SDIO_CmdInitStructure);

	errorstatus = CmdResp1Error(SD_CMD_SD_APP_SEND_SCR);
	if(errorstatus != SD_OK)	return errorstatus;	//操作失败

	while(!(SDIO->STA & (SDIO_FLAG_RXOVERR | SDIO_FLAG_DCRCFAIL | SDIO_FLAG_DTIMEOUT | SDIO_FLAG_DBCKEND | SDIO_FLAG_STBITERR)))
	{
		if(SDIO_GetFlagStatus(SDIO_FLAG_RXDAVL) != RESET)
		{
			*(tempscr + index) = SDIO_ReadData();
			index++;
			if(index == 2)	break;
		}
	}

	if(SDIO_GetFlagStatus(SDIO_FLAG_DTIMEOUT) != RESET)
	{
		SDIO_ClearFlag(SDIO_FLAG_DTIMEOUT);
		errorstatus = SD_DATA_TIMEOUT;
		return(errorstatus);
	}
	else if(SDIO_GetFlagStatus(SDIO_FLAG_DCRCFAIL) != RESET)
	{
		SDIO_ClearFlag(SDIO_FLAG_DCRCFAIL);
		errorstatus = SD_DATA_CRC_FAIL;
		return(errorstatus);
	}
	else if(SDIO_GetFlagStatus(SDIO_FLAG_RXOVERR) != RESET)
	{
		SDIO_ClearFlag(SDIO_FLAG_RXOVERR);
		errorstatus = SD_RX_OVERRUN;
		return(errorstatus);
	}
	else if(SDIO_GetFlagStatus(SDIO_FLAG_STBITERR) != RESET)
	{
		SDIO_ClearFlag(SDIO_FLAG_STBITERR);
		errorstatus = SD_START_BIT_ERR;
		return(errorstatus);
	}

	/*!< Clear all the static flags */
	SDIO_ClearFlag(SDIO_STATIC_FLAGS);

	*(pscr + 1) = ((tempscr[0] & SD_0TO7BITS) << 24) | ((tempscr[0] & SD_8TO15BITS) << 8) | ((tempscr[0] & SD_16TO23BITS) >> 8) | ((tempscr[0] & SD_24TO31BITS) >> 24);
	*(pscr) = ((tempscr[1] & SD_0TO7BITS) << 24) | ((tempscr[1] & SD_8TO15BITS) << 8) | ((tempscr[1] & SD_16TO23BITS) >> 8) | ((tempscr[1] & SD_24TO31BITS) >> 24);

	return(errorstatus);
}
/**********************************************************
* 函数功能 ---> 设置总线宽度
* 入口参数 ---> NewState：状态开关量
* 返回数值 ---> 返回卡应答
* 功能说明 ---> none
**********************************************************/
static SD_Error SDEnWideBus(FunctionalState NewState)
{
	SD_Error errorstatus = SD_OK;
	uint32_t scr[2] = {0, 0};

	if(SDIO_GetResponse(SDIO_RESP1) & SD_CARD_LOCKED)	//卡锁了
	{
		errorstatus = SD_LOCK_UNLOCK_FAILED;
		return(errorstatus);
	}

	/*!< Get SCR Register */
	errorstatus = FindSCR(RCA, scr);
	if(errorstatus != SD_OK)	return errorstatus;	//操作失败

	/*!< If wide bus operation to be enabled */
	if(NewState == ENABLE)
	{
		/*!< If requested card supports wide bus operation */
		if((scr[1] & SD_WIDE_BUS_SUPPORT) != SD_ALLZERO)
		{
			/*!< Send CMD55 APP_CMD with argument as card's RCA.*/
			SDIO_CmdInitStructure.SDIO_Argument = (uint32_t) RCA << 16;
			SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_APP_CMD;
			SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
			SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
			SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
			SDIO_SendCommand(&SDIO_CmdInitStructure);

			errorstatus = CmdResp1Error(SD_CMD_APP_CMD);
			if(errorstatus != SD_OK)	return errorstatus;	//操作失败

			/*!< Send ACMD6 APP_CMD with argument as 2 for wide bus mode */
			SDIO_CmdInitStructure.SDIO_Argument = 0x2;
			SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_APP_SD_SET_BUSWIDTH;
			SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
			SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
			SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
			SDIO_SendCommand(&SDIO_CmdInitStructure);

			errorstatus = CmdResp1Error(SD_CMD_APP_SD_SET_BUSWIDTH);
			if(errorstatus != SD_OK)	return errorstatus;	//操作失败

			return(errorstatus);
		}
		else
		{
			errorstatus = SD_REQUEST_NOT_APPLICABLE;
			return(errorstatus);
		}
	}   /*!< If wide bus operation to be disabled */
	else
	{
		/*!< If requested card supports 1 bit mode operation */
		if((scr[1] & SD_SINGLE_BUS_SUPPORT) != SD_ALLZERO)
		{
			/*!< Send CMD55 APP_CMD with argument as card's RCA.*/
			SDIO_CmdInitStructure.SDIO_Argument = (uint32_t) RCA << 16;
			SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_APP_CMD;
			SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
			SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
			SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
			SDIO_SendCommand(&SDIO_CmdInitStructure);

			errorstatus = CmdResp1Error(SD_CMD_APP_CMD);
			if(errorstatus != SD_OK)	return errorstatus;	//操作失败

			/*!< Send ACMD6 APP_CMD with argument as 2 for wide bus mode */
			SDIO_CmdInitStructure.SDIO_Argument = 0x00;
			SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_APP_SD_SET_BUSWIDTH;
			SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
			SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
			SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
			SDIO_SendCommand(&SDIO_CmdInitStructure);

			errorstatus = CmdResp1Error(SD_CMD_APP_SD_SET_BUSWIDTH);
			if(errorstatus != SD_OK)	return errorstatus;	//操作失败

			return(errorstatus);
		}
		else
		{
			errorstatus = SD_REQUEST_NOT_APPLICABLE;
			return(errorstatus);
		}
	}
}
/**********************************************************
* 函数功能 ---> 获取卡执行状态
* 入口参数 ---> *pstatus：返回状态值缓存
* 返回数值 ---> 返回卡应答
* 功能说明 ---> none
**********************************************************/
static SD_Error IsCardProgramming(uint8_t *pstatus)
{
	SD_Error errorstatus = SD_OK;
	__IO uint32_t respR1 = 0, status = 0;

	//发送CMD13命令
	SDIO_CmdInitStructure.SDIO_Argument = (uint32_t) RCA << 16;
	SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SEND_STATUS;
	SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
	SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
	SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
	SDIO_SendCommand(&SDIO_CmdInitStructure);

	status = SDIO->STA;
	while(!(status & (SDIO_FLAG_CCRCFAIL | SDIO_FLAG_CMDREND | SDIO_FLAG_CTIMEOUT)))	status = SDIO->STA;

	if(status & SDIO_FLAG_CTIMEOUT)
	{
		errorstatus = SD_CMD_RSP_TIMEOUT;
		SDIO_ClearFlag(SDIO_FLAG_CTIMEOUT);
		return(errorstatus);
	}
	else if(status & SDIO_FLAG_CCRCFAIL)
	{
		errorstatus = SD_CMD_CRC_FAIL;
		SDIO_ClearFlag(SDIO_FLAG_CCRCFAIL);
		return(errorstatus);
	}

	status = (uint32_t)SDIO_GetCommandResponse();

	/*!< Check response received is of desired command */
	if(status != SD_CMD_SEND_STATUS)
	{
		errorstatus = SD_ILLEGAL_CMD;
		return(errorstatus);
	}

	/*!< Clear all the static flags */
	SDIO_ClearFlag(SDIO_STATIC_FLAGS);

	/*!< We have received response, retrieve it for analysis  */
	respR1 = SDIO_GetResponse(SDIO_RESP1);

	/*!< Find out card status */
	*pstatus = (uint8_t) ((respR1 >> 9) & 0x0000000F);

	if((respR1 & SD_OCR_ERRORBITS) == SD_ALLZERO)	return(errorstatus);
	if(respR1 & SD_OCR_ADDR_OUT_OF_RANGE)			return(SD_ADDR_OUT_OF_RANGE);
	if(respR1 & SD_OCR_ADDR_MISALIGNED)				return(SD_ADDR_MISALIGNED);
	if(respR1 & SD_OCR_BLOCK_LEN_ERR)				return(SD_BLOCK_LEN_ERR);
	if(respR1 & SD_OCR_ERASE_SEQ_ERR)				return(SD_ERASE_SEQ_ERR);
	if(respR1 & SD_OCR_BAD_ERASE_PARAM)				return(SD_BAD_ERASE_PARAM);
	if(respR1 & SD_OCR_WRITE_PROT_VIOLATION)		return(SD_WRITE_PROT_VIOLATION);
	if(respR1 & SD_OCR_LOCK_UNLOCK_FAILED)			return(SD_LOCK_UNLOCK_FAILED);
	if(respR1 & SD_OCR_COM_CRC_FAILED)				return(SD_COM_CRC_FAILED);
	if(respR1 & SD_OCR_ILLEGAL_CMD)					return(SD_ILLEGAL_CMD);
	if(respR1 & SD_OCR_CARD_ECC_FAILED)				return(SD_CARD_ECC_FAILED);
	if(respR1 & SD_OCR_CC_ERROR)					return(SD_CC_ERROR);
	if(respR1 & SD_OCR_GENERAL_UNKNOWN_ERROR)		return(SD_GENERAL_UNKNOWN_ERROR);
	if(respR1 & SD_OCR_STREAM_READ_UNDERRUN)		return(SD_STREAM_READ_UNDERRUN);
	if(respR1 & SD_OCR_STREAM_WRITE_OVERRUN)		return(SD_STREAM_WRITE_OVERRUN);
	if(respR1 & SD_OCR_CID_CSD_OVERWRIETE)			return(SD_CID_CSD_OVERWRITE);
	if(respR1 & SD_OCR_WP_ERASE_SKIP)				return(SD_WP_ERASE_SKIP);
	if(respR1 & SD_OCR_CARD_ECC_DISABLED)			return(SD_CARD_ECC_DISABLED);
	if(respR1 & SD_OCR_ERASE_RESET)					return(SD_ERASE_RESET);
	if(respR1 & SD_OCR_AKE_SEQ_ERROR)				return(SD_AKE_SEQ_ERROR);

	return(errorstatus);
}
/**********************************************************
* 函数功能 ---> 得到NumberOfBytes以2为底的指数值
* 入口参数 ---> NumberOfBytes：2的指数值
* 返回数值 ---> 指数运算结果
* 功能说明 ---> none
**********************************************************/
uint8_t convert_from_bytes_to_power_of_two(uint16_t NumberOfBytes)
{
	uint8_t count = 0;

	while(NumberOfBytes != 1)
	{
		NumberOfBytes >>= 1;
		count++;
	}
	return(count);
}
/**********************************************************
* 函数功能 ---> 中断处理函数
* 入口参数 ---> *psdstatus：返回状态值缓存
* 返回数值 ---> 返回卡应答
* 功能说明 ---> none
**********************************************************/
SD_Error SD_ProcessIRQSrc(void)
{
	if(SDIO_GetITStatus(SDIO_IT_DATAEND) != RESET)//接收完成中断
	{	 
		if (StopCondition==1)
		{
			//发送CMD12,结束传输 	   
			SDIO_CmdInitStructure.SDIO_Argument = 0;
			SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_STOP_TRANSMISSION;
			SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
			SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
			SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
			SDIO_SendCommand(&SDIO_CmdInitStructure);
			TransferError=CmdResp1Error(SD_CMD_STOP_TRANSMISSION);
		}
		else	TransferError = SD_OK;	
		
		SDIO_ClearITPendingBit(SDIO_IT_DATAEND);//清除完成中断标记
		SDIO_ITConfig(SDIO_IT_DCRCFAIL | SDIO_IT_DTIMEOUT | SDIO_IT_DATAEND |
					  SDIO_IT_TXFIFOHE | SDIO_IT_RXFIFOHF | SDIO_IT_TXUNDERR |
					  SDIO_IT_RXOVERR | SDIO_IT_STBITERR, DISABLE);//关闭相关中断
		TransferEnd = 1;
		return(TransferError);
	}
	if(SDIO_GetITStatus(SDIO_IT_DCRCFAIL) != RESET)//数据CRC错误
	{
		SDIO_ClearITPendingBit(SDIO_IT_DCRCFAIL);//清除中断标记
		SDIO_ITConfig(SDIO_IT_DCRCFAIL | SDIO_IT_DTIMEOUT | SDIO_IT_DATAEND |
					  SDIO_IT_TXFIFOHE | SDIO_IT_RXFIFOHF | SDIO_IT_TXUNDERR |
 					  SDIO_IT_RXOVERR | SDIO_IT_STBITERR, DISABLE);//关闭相关中断  
		TransferError = SD_DATA_CRC_FAIL;
		return(SD_DATA_CRC_FAIL);
	}
	if(SDIO_GetITStatus(SDIO_IT_DTIMEOUT) != RESET)//数据超时错误
	{
		SDIO_ClearITPendingBit(SDIO_IT_DTIMEOUT);//清除中断标记
		SDIO_ITConfig(SDIO_IT_DCRCFAIL | SDIO_IT_DTIMEOUT | SDIO_IT_DATAEND |
					  SDIO_IT_TXFIFOHE | SDIO_IT_RXFIFOHF | SDIO_IT_TXUNDERR |
					  SDIO_IT_RXOVERR | SDIO_IT_STBITERR, DISABLE);//关闭相关中断
		TransferError = SD_DATA_TIMEOUT;
		return(SD_DATA_TIMEOUT);
	}
	if(SDIO_GetITStatus(SDIO_IT_RXOVERR) != RESET)//FIFO上溢错误
	{
		SDIO_ClearITPendingBit(SDIO_IT_RXOVERR);//清除中断标记
		SDIO_ITConfig(SDIO_IT_DCRCFAIL | SDIO_IT_DTIMEOUT | SDIO_IT_DATAEND |
					  SDIO_IT_TXFIFOHE | SDIO_IT_RXFIFOHF | SDIO_IT_TXUNDERR |
					  SDIO_IT_RXOVERR | SDIO_IT_STBITERR, DISABLE);//关闭相关中断  
		TransferError = SD_RX_OVERRUN;
		return(SD_RX_OVERRUN);
	}
	if(SDIO_GetITStatus(SDIO_IT_TXUNDERR) != RESET)//FIFO下溢错误
	{
		SDIO_ClearITPendingBit(SDIO_IT_TXUNDERR);//清除中断标记
		SDIO_ITConfig(SDIO_IT_DCRCFAIL | SDIO_IT_DTIMEOUT | SDIO_IT_DATAEND |
					  SDIO_IT_TXFIFOHE | SDIO_IT_RXFIFOHF | SDIO_IT_TXUNDERR |
					  SDIO_IT_RXOVERR | SDIO_IT_STBITERR, DISABLE);//关闭相关中断  
		TransferError = SD_TX_UNDERRUN;
		return(SD_TX_UNDERRUN);
	}
	if(SDIO_GetITStatus(SDIO_IT_STBITERR) != RESET)//起始位错误
	{
		SDIO_ClearITPendingBit(SDIO_IT_STBITERR);//清除中断标记
		SDIO_ITConfig(SDIO_IT_DCRCFAIL | SDIO_IT_DTIMEOUT | SDIO_IT_DATAEND |
					  SDIO_IT_TXFIFOHE | SDIO_IT_RXFIFOHF | SDIO_IT_TXUNDERR |
					  SDIO_IT_RXOVERR | SDIO_IT_STBITERR, DISABLE);//关闭相关中断  
		TransferError = SD_START_BIT_ERR;
		return(SD_START_BIT_ERR);
	}
	return(SD_OK);
}
/**********************************************************
* 函数功能 ---> SDIO中断服务函数
* 入口参数 ---> none
* 返回数值 ---> none
* 功能说明 ---> none
**********************************************************/
void SDIO_IRQHandler(void) 
{											
	SD_ProcessIRQSrc();	//处理所有SDIO相关中断
}
/**********************************************************
* 函数功能 ---> 读SD卡
* 入口参数 ---> *readbuff：读取数据缓存
*               sector：读取block地址
*               cnt：读取block数目
* 返回数值 ---> 返回卡应答
* 功能说明 ---> none
**********************************************************/			  				 
SD_Error SD_ReadDisk(uint8_t *readbuff, uint32_t sector, uint16_t cnt)
{
	SD_Error sta = SD_OK;
	uint16_t n;

	if(CardType!=SDIO_STD_CAPACITY_SD_CARD_V1_1)	sector<<=9;

	if((uint32_t)readbuff%4!=0)
	{
	 	for(n=0;n<cnt;n++)
		{
		 	sta = SD_ReadBlock(SDIO_DATA_BUFFER, sector, 512);    	//单个sector的读操作
			memcpy(readbuff, SDIO_DATA_BUFFER, 512);
			readbuff+=512;
		} 
	}
	else
	{
		if(cnt==1)	sta = SD_ReadBlock(readbuff, sector, 512);    	//单个sector的读操作
		else	sta = SD_ReadMultiBlocks(readbuff, sector, 512, cnt);//多个sector  
	}
	return sta;
}
/**********************************************************
* 函数功能 ---> 写SD卡
* 入口参数 ---> *writebuff：读取数据缓存
*               sector：读取block地址
*               cnt：读取block数目
* 返回数值 ---> 返回卡应答
* 功能说明 ---> none
**********************************************************/	
SD_Error SD_WriteDisk(uint8_t *writebuff, uint32_t sector, uint16_t cnt)
{
	SD_Error sta = SD_OK;
	uint16_t n;

	if(CardType!=SDIO_STD_CAPACITY_SD_CARD_V1_1)	sector<<=9;

	if((uint32_t)writebuff%4!=0)
	{
	 	for(n=0;n<cnt;n++)
		{
			memcpy(SDIO_DATA_BUFFER, writebuff, 512);
		 	sta = SD_WriteBlock(SDIO_DATA_BUFFER, sector, 512);    	//单个sector的写操作
			writebuff+=512;
		} 
	}
	else
	{
		if(cnt==1)	sta = SD_WriteBlock(writebuff, sector, 512);    	//单个sector的写操作
		else	sta = SD_WriteMultiBlocks(writebuff, sector, 512, cnt);	//多个sector  
	}
	return sta;
}

