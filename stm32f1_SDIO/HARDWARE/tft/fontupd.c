/**********************************************************
* @ File name -> fontupd.c
* @ Version   -> V1.0
* @ Date      -> 13-02-2014
* @ Brief     -> �ֿ���غ���

 V1.
* @ Revise    ->
**********************************************************/

#include "fontupd.h"

/* ͷ�ļ����ض��� */
#if ((_USER_SD_FON==1)||(_USER_BufforFLASH==2))	//ʹ��SD���ֿ����FLASH�ļ�ϵͳ�ֿ�Ļ�Ҫ����ļ�ϵͳ����

	#include "ff.h"

#endif

//=========================================================
#if !_USER_SD_FON	//�ڲ�ʹ��SD����Ϊ�ֿ��ʱ���ʹ��SPI-FLASH�������鷽ʽ
//=========================================================
#if _USER_BufforFLASH==1	//ʹ��SPI-FLASH�ֿ�ʱ���õ�ͷ�ļ�

	#include "w25q64.h"
	#include "refreshfont.h"	//��⡢�����ֿ⺯��

#endif	//end _USER_BufforFLASH
//=========================================================

/* û�ж���SD����FLASH�Ļ���ʹ�����鷽ʽ��ȱ����̫ռ��MCU����ռ� */

//#include "HzLib_65k.h"	//GB2312���������ֿ�

/* ����ͷ�ļ����ض��� */

#endif	//end _USER_SD_FON

/**********************************************************
                        �����ֿ�·��
**********************************************************/

#if _USER_SD_FON	//ʹ��SD���ֿ�

//��SD���е�·��
const uint8_t *GBK12_SDUPD="0:/SYSTEM/FONT/GBK12.FON";	//GBK12.FON�ֿ���λ��
const uint8_t *GBK16_SDUPD="0:/SYSTEM/FONT/GBK16.FON";	//GBK16.FON�ֿ���λ��
const uint8_t *UNIGBK_SDUPD="0:/SYSTEM/FONT/UNIGBK.BIN";	//UNIGBK.BIN�ֿ���λ��
const uint8_t *Hz65k_SDUPD="0:/SYSTEM/FONT/Hz65k.o";	//����GB2312�ֿ�

#else	//ʹ��FLASH����ˢ��MCU�ڲ����ֿ�����

#if _USER_BufforFLASH==2	//ʹ��SPI-FLASH�ļ�ϵͳ�ڵ��ֿ�

//��SPI-FLASH�е�·��
const uint8_t *GBK12_FLASHUPD="1:/SYSTEM/FONT/GBK12.FON";	//GBK12.FON�ֿ���λ��
const uint8_t *GBK16_FLASHUPD="1:/SYSTEM/FONT/GBK16.FON";	//GBK16.FON�ֿ���λ��
const uint8_t *UNIGBK_FLASHUPD="1:/SYSTEM/FONT/UNIGBK.BIN";	//UNIGBK.BIN�ֿ���λ��
const uint8_t *Hz65k_FLASHUPD="1:/SYSTEM/FONT/Hz65k.o";	//����GB2312�ֿ�

#endif	//end _USER_BufforFLASH
#endif	//end _USER_SD_FON


//=========================================================
#if _USER_SD_FON	//��������ˣ���ʹ��SD����Ϊ�����ֿ�ʹ��
//=========================================================
/**********************************************************
* �������� ---> ȡ�������ֿ�����
* ��ڲ��� ---> *code���ִ���ʼ��ַ
*               *c_HanZibuff����ȡ������ģ���ݻ���
*               size�������С
* ������ֵ ---> none
* ����˵�� ---> ���ô˺���ǰ������ע��FatFs������������SD��
**********************************************************/
void Get_Hzlib(u8 *ch, u8 *c_HanZibuff,u8 size)
{
	FIL filefont;	//�ļ�
	UINT brf;		//��д����
	FRESULT res;	//��������Ӧ��

//	unsigned char i;
//	unsigned char qh,ql;					  
	unsigned long foffset;	//�������ֿ���׵�ַ
	 
//	qh = *ch;
//	ql = *(++ch);
//	if(qh<0x81||ql<0x40||ql==0xff||qh==0xff)//�� ���ú���
//	{   		    
//	    for(i=0;i<(size*2);i++)	*c_HanZibuff ++= 0x00;//�������
//	    return; //��������
//	}
//	//���㹫ʽ
//	//�� GBKL<0X7F ʱ��Hp=((GBKH-0x81)*190+GBKL-0X40)*(size*2)
//	//�� GBKL>0X80 ʱ��Hp=((GBKH-0x81)*190+GBKL-0X41)*(size*2)          
//	if(ql<0x7f)	ql -= 0x40;//ע��!
//	else	ql -= 0x41;
//	qh -= 0x81;
//	//���㺺�ֵ�ַƫ��   
//	foffset = ((unsigned long)190*qh+ql)*(size*2);//�õ��ֿ��е��ֽ�ƫ����

	foffset = (((*ch) - 176)*94 + ((*(ch+1)) - 161))*32;	//����õ��������ֿ��е�ƫ�Ƶ�ַ
	
//	res = f_open (&filefont, (const TCHAR*)GBK16_SDUPD, FA_OPEN_EXISTING | FA_READ);	//�����Խ���д����
//	res = f_open (&filefont, "0:/SYSTEM/FONT/GBK16.fon", FA_OPEN_EXISTING | FA_READ);	//�����Խ���д����
	res = f_open (&filefont, (const TCHAR*)Hz65k_SDUPD, FA_OPEN_EXISTING | FA_READ);	//�����Խ���д����
	if(res != FR_OK)	//��ʧ��
	{	printf("open SD FON res:%d\r\n", res);
		f_close(&filefont);	//�ر��ļ�
		return;	//��������
	}
	else	//���ļ��ɹ�
	{
//		memset(c_HanZibuff, 0, 32);	//��������黺��
		res = f_lseek(&filefont, foffset+0x112);	//�õ��ļ�ָ��ƫ����
		res = f_read(&filefont, c_HanZibuff, 32, &brf);	//��ȡ32�ֽ�����	
		res = f_close(&filefont);	//�ر��ļ�
	}
}

//=========================================================
#else	//����ֱ��ʹ�ö��������
//=========================================================

/**********************************************************
* �������� ---> ȡ�������ֿ�����
* ��ڲ��� ---> *code���ִ���ʼ��ַ
*               *c_HanZibuff����ȡ������ģ���ݻ���
*               size�������С
* ������ֵ ---> none
* ����˵�� ---> none
**********************************************************/
void Get_Hzlib(u8 *ch, u8 *c_HanZibuff,u8 size)
{
#if _USER_BufforFLASH==0	//ʹ��ˢ��MCU�ڲ����ֿ�����

	unsigned char i;
	u32 address=0;
	if(size == 16)
	{
		address = ( ((*ch)-176)*94 + ((*(ch+1))-161) )*32;
		for(i = 0;i < address;i++)
		{
			c_HanZibuff[i] = HzLib[i];
		}
	}

#elif _USER_BufforFLASH==1	//����ʹ���ⲿSPI-FLASH���ֿ�оƬ

	unsigned char i;
	unsigned char qh,ql;					  
	unsigned long foffset;
	 
	qh = *ch;
	ql = *(++ch);
	if(qh<0x81||ql<0x40||ql==0xff||qh==0xff)//�� ���ú���
	{   		    
	    for(i=0;i<(size*2);i++)	*c_HanZibuff++=0x00;//�������
	    return; //��������
	}
	//���㹫ʽ
	//�� GBKL<0X7F ʱ��Hp=((GBKH-0x81)*190+GBKL-0X40)*(size*2)
	//�� GBKL>0X80 ʱ��Hp=((GBKH-0x81)*190+GBKL-0X41)*(size*2)
	          
	if(ql<0x7f)	ql -= 0x40;//ע��!
	else	ql -= 0x41;
	qh -= 0x81;   
	foffset=((unsigned long)190*qh+ql)*(size*2);//�õ��ֿ��е��ֽ�ƫ����  		  

	if(size==16)SPI_Flash_Read(c_HanZibuff,foffset+ftinfo.f16addr,32);
	else SPI_Flash_Read(c_HanZibuff,foffset+ftinfo.f12addr,24);

#elif _USER_BufforFLASH==2	//ʹ��SPI-FLASH�ļ�ϵͳ���ֿ�

	FIL filefont;	//�ļ�
	UINT brf;		//��д����
	FRESULT res;	//��������Ӧ��

//	unsigned char i;
//	unsigned char qh,ql;					  
	unsigned long foffset;	//�������ֿ���׵�ַ
	 
//	qh = *ch;
//	ql = *(++ch);
//	if(qh<0x81||ql<0x40||ql==0xff||qh==0xff)//�� ���ú���
//	{   		    
//	    for(i=0;i<(size*2);i++)	*c_HanZibuff ++= 0x00;//�������
//	    return; //��������
//	}
//	//���㹫ʽ
//	//�� GBKL<0X7F ʱ��Hp=((GBKH-0x81)*190+GBKL-0X40)*(size*2)
//	//�� GBKL>0X80 ʱ��Hp=((GBKH-0x81)*190+GBKL-0X41)*(size*2)          
//	if(ql<0x7f)	ql -= 0x40;//ע��!
//	else	ql -= 0x41;
//	qh -= 0x81;
//	//���㺺�ֵ�ַƫ��   
//	foffset = ((unsigned long)190*qh+ql)*(size*2);//�õ��ֿ��е��ֽ�ƫ����

	foffset = (((*ch) - 176)*94 + ((*(ch+1)) - 161))*32;	//����õ��������ֿ��е�ƫ�Ƶ�ַ
	
	res = f_open (&filefont, (const TCHAR*)Hz65k_FLASHUPD, FA_OPEN_EXISTING | FA_READ);	//�����Խ���д����
	if(res != FR_OK)	//��ʧ��
	{	printf("open SD FON res:%d\r\n", res);
		f_close(&filefont);	//�ر��ļ�
		return;	//��������
	}
	else	//���ļ��ɹ�
	{
//		memset(c_HanZibuff, 0, 32);	//��������黺��
		res = f_lseek(&filefont, foffset+0x112);	//�õ��ļ�ָ��ƫ����
		res = f_read(&filefont, c_HanZibuff, 32, &brf);	//��ȡ32�ֽ�����	
		res = f_close(&filefont);	//�ر��ļ�
	}

#endif	//end _USER_BufforFLASH
	
}

//=========================================================
#endif	//end _USER_SD_FON
//========================================================= 

