/**********************************************************
* @ File name -> lcd.c
* @ Version   -> V1.0.2
* @ Date      -> 02-25-2014
* @ Brief     -> TFT彩屏上层应用函数

 V1.0.1
* @ Revise    -> A、将LCD_Display_String_BK1函数进行修改
*                B、修改为带位数显示选择，默认不叠加显示
*                C、将函数名修改为LCD_Display_String_Num

 V1.0.2
* @ Revise    -> 修正数字串显示函数小bug，原来是从高位开始数的显示N位数字字符
*                修正后从最低位开始数的显示N位数字字符
*                比如显示00336.85这个数，显示8位就全部显示（高位为0也显示），显示5为就是36.85（小数点都算进显示位数）
**********************************************************/

#include "lcd.h"

/**********************************************************
                        添加字库
**********************************************************/

#include "fonts.h"	//ASCII字符型字库

/**********************************************************
                     定义相关变量参数
**********************************************************/

lcd_dev LCD_Manage;	//定义LCD参数全局变量

u16 Point_color = RED;	//默认点的颜色
u16 Back_color = WHITH;	//默认背景颜色

/**********************************************************
* 函数功能 ---> 读取LCD需要的延时
* 入口参数 ---> i：需要延时数值
* 返回数值 ---> none
* 功能说明 ---> none
**********************************************************/
void LCD_Delay(u8 i)
{
	while(i--);
}
/**********************************************************
* 函数功能 ---> LCD开启显示
* 入口参数 ---> none
* 返回数值 ---> none
* 功能说明 ---> none
**********************************************************/
void LCD_Display_ON(void)
{
      if(LCD_Manage.ID == 0x9341)   LCD_Write_Register(Display_ON);	//关闭显示
      else  LCD_WriteRegisterData(0x07, 0x0173);
}
/**********************************************************
* 函数功能 ---> LCD关闭显示
* 入口参数 ---> none
* 返回数值 ---> none
* 功能说明 ---> none
**********************************************************/
void LCD_Display_OFF(void)
{
      if(LCD_Manage.ID == 0x9341)   LCD_Write_Register(Display_OFF);	//关闭显示
      else  LCD_WriteRegisterData(0x07, 0x0);
}
/**********************************************************
* 函数功能 ---> LCD进入睡眠
* 入口参数 ---> none
* 返回数值 ---> none
* 功能说明 ---> none
**********************************************************/
void LCD_Enter_Sleep(void)
{
	if(LCD_Manage.ID == 0x9341)   LCD_Write_Register(LCD_EnterSleep);	//进入睡眠
}
/**********************************************************
* 函数功能 ---> LCD退出睡眠
* 入口参数 ---> none
* 返回数值 ---> none
* 功能说明 ---> none
**********************************************************/
void LCD_Exit_Sleep(void)
{
	if(LCD_Manage.ID == 0x9341)   LCD_Write_Register(LCD_ExitSleep);	//退出睡眠
}
/**********************************************************
* 函数功能 ---> LCD写入GRAM（命令）
* 入口参数 ---> none
* 返回数值 ---> none
* 功能说明 ---> none
**********************************************************/
void LCD_WriteGRAM(void)
{
      LCD_Write_Register(LCD_Manage.wgramcmd);
}
/**********************************************************
* 函数功能 ---> 设置LCD显示位置
* 入口参数 ---> (x,y): 坐标原点
* 返回数值 ---> none
* 功能说明 ---> none
**********************************************************/
void LCD_Set_xy(u16 x,u16 y)
{
      if((x > LCD_Manage.width) || (y > LCD_Manage.height)) return;     //超出范围，直接退出
      //没超出范围则设置

      if(LCD_Manage.ID == 0x9341)
      {
            LCD_Write_Register(LCD_Manage.setxcmd);	//设置x 
            LCD_Write_Data(x >> 8); 
            LCD_Write_Data(x & 0xff);	 
            LCD_Write_Register(LCD_Manage.setycmd);	//设置y
            LCD_Write_Data(y >> 8); 
            LCD_Write_Data(y & 0xff);
      }
      else
      {
            if(LCD_Manage.dir == 1) //横屏时
            {
                  x = LCD_Manage.width - 1 - x; //调转x，y
            }
            LCD_WriteRegisterData(LCD_Manage.setxcmd, x);   //设置x坐标值
            LCD_WriteRegisterData(LCD_Manage.setycmd, y);   //设置y坐标值
      }
/*      
      if(LCD_Manage.dir == 1) //横屏显示
      {
            x = LCD_Manage.width - x - 1;      //调转x,y
            
            LCD_WriteRegisterData(LCD_Manage.setxcmd, x);   //设置x坐标值
            LCD_WriteRegisterData(LCD_Manage.setycmd, y);   //设置y坐标值
      }
      else  //竖屏显示
      {
            if(LCD_Manage.ID == 0x9341)
            {
                  LCD_Write_Register(LCD_Manage.setxcmd);	//设置x 
                  LCD_Write_Data(x >> 8); 
                  LCD_Write_Data(x & 0xff);	 
                  LCD_Write_Register(LCD_Manage.setycmd);	//设置y
                  LCD_Write_Data(y >> 8); 
                  LCD_Write_Data(y & 0xff);
            }
            LCD_WriteRegisterData(LCD_Manage.setxcmd, x);   //设置x坐标值
            LCD_WriteRegisterData(LCD_Manage.setycmd, y);   //设置y坐标值
      }
*/
}
/**********************************************************
* 函数功能 ---> 设定LCD显示的扫描方式
* 入口参数 ---> dir：0：竖屏显示。1：横屏显示
* 返回数值 ---> none
* 功能说明 ---> none
**********************************************************/
void LCD_Scan_DIR(u8 dire)
{
      u8 dir_reg=0;    //扫描方式寄存器
      u16 dir_val=0;    //扫描寄存器数值
      u16 temp=0; //计算缓存

      if(LCD_Manage.dir == 1) //横屏
      {
            switch(dire)
            {
                  //顺时针旋转至横屏
                  case L2R_U2D:     //竖屏时--->从左到右，从上到下方向
                              dire = D2U_L2R;   //下到上，左到右
                              break;

                  case L2R_D2U:     //竖屏时--->从左到右，从下到上
                              dire = D2U_R2L;   //下到上，右到左
                              break;

                  case R2L_U2D:     //竖屏时--->从右到左，从上到下
                              dire = U2D_L2R;   //上到下，左到右
                              break;

                  case R2L_D2U:     //竖屏时--->从右到左，从下到上
                              dire = U2D_R2L;   //上到下，右到左
                              break;

                  //逆时针旋转会之前的竖屏状态

                  case U2D_L2R:     //横屏时--->从上到下，从左到右
                              dire = L2R_D2U;   //左到右，下到上
                              break;

                  case U2D_R2L:     //横屏时--->从上到下，从右到左
                              dire = L2R_U2D;   //左到右，上到下
                              break;

                  case D2U_L2R:     //横屏时--->从下到上，从左到右
                              dire = R2L_D2U;   //右到左，下到上
                              break;

                  case D2U_R2L:     //横屏时--->从下到上，从右到左
                              dire = R2L_U2D;   //右到左，上到下
                              break;
            }
      }

      if(LCD_Manage.ID == 0x9341)
      {
            switch(dire)
            {	//            I/D1       I/D0       AM
			case L2R_U2D:	//从左到右，从上到下
					dir_val |= (0<<7) | (0<<6) | (0<<5); 
					break;
			case L2R_D2U:	//从左到右,从下到上
					dir_val |= (1<<7) | (0<<6) | (0<<5); 
					break;
			case R2L_U2D:	//从右到左,从上到下
					dir_val |= (0<<7) | (1<<6) | (0<<5); 
					break;
			case R2L_D2U:	//从右到左,从下到上
					dir_val |= (1<<7) | (1<<6) | (0<<5); 
					break;	 
			case U2D_L2R:	//从上到下,从左到右
				      dir_val |= (0<<7) | (0<<6) | (1<<5); 
				      break;
			case U2D_R2L:	//从上到下,从右到左
					dir_val |= (0<<7) | (1<<6) | (1<<5); 
					break;
			case D2U_L2R:	//从下到上,从左到右
					dir_val |= (1<<7) | (0<<6) | (1<<5); 
					break;
			case D2U_R2L:	//从下到上,从右到左
					dir_val |= (1<<7) | (1<<6) | (1<<5); 
					break;	 
		}
            
		dir_reg = 0x36;
 		dir_val |= 0x08;//BGR   	   
		LCD_WriteRegisterData(dir_reg,dir_val);
 		if(dir_val & 0x20)
		{
			if(LCD_Manage.width < LCD_Manage.height)//交换X,Y
			{
				temp = LCD_Manage.width;
				LCD_Manage.width = LCD_Manage.height;
				LCD_Manage.height = temp;
 			}
		}
            else  
      	{
      		if(LCD_Manage.width > LCD_Manage.height)//交换X,Y
      		{
      			temp = LCD_Manage.width;
      			LCD_Manage.width = LCD_Manage.height;
      			LCD_Manage.height = temp;
       		}
      	}
		LCD_Write_Register(LCD_Manage.setxcmd); 
      	LCD_Write_Data(0);LCD_Write_Data(0);	//横坐标起点
      	LCD_Write_Data((LCD_Manage.width-1)>>8);LCD_Write_Data((LCD_Manage.width-1)&0XFF);	//横坐标终点
      	LCD_Write_Register(LCD_Manage.setycmd); 
      	LCD_Write_Data(0);LCD_Write_Data(0);	//纵坐标起点
      	LCD_Write_Data((LCD_Manage.height-1)>>8);LCD_Write_Data((LCD_Manage.height-1)&0XFF);	//纵坐标终点  
      }
      else  //if(LCD_Manage.ID == 0x9325)
      {
            switch(dire)
            {                //            I/D1       I/D0       AM
                  case L2R_U2D:
                              dir_val = (1<<5) | (1<<4) | (0<<3);
                              break;

                  case L2R_D2U:
                              dir_val = (0<<5) | (1<<4) | (0<<3);
                              break;

                  case R2L_U2D:
                              dir_val = (1<<5) | (0<<4) | (0<<3);
                              break;

                  case R2L_D2U:
                              dir_val = (0<<5) | (0<<4) | (0<<3);
                              break;

                  case U2D_L2R:
                              dir_val = (1<<5) | (1<<4) | (1<<3);
                              break;

                  case U2D_R2L:
                              dir_val = (1<<5) | (0<<4) | (1<<3);
                              break;

                  case D2U_L2R:
                              dir_val = (0<<5) | (1<<4) | (1<<3);
                              break;

                  case D2U_R2L:
                              dir_val = (0<<5) | (0<<4) | (1<<3);
                              break;
            }
            
            dir_reg = Entry_Mode;   //屏幕扫描方式寄存器地址
            dir_val |= 0x1000;      //要填入的数值
            LCD_WriteRegisterData(dir_reg, dir_val);
      }
}
/**********************************************************
* 函数功能 ---> 设定LCD显示方向
* 入口参数 ---> dir：0：竖屏显示。1：横屏显示
* 返回数值 ---> none
* 功能说明 ---> none
**********************************************************/
void LCD_Display_DIR(u8 dir)
{
	if(dir == 0)	//竖屏显示
	{
		LCD_Manage.dir = 0;	//竖屏显示
		LCD_Manage.width = 240;	//横坐标点数
		LCD_Manage.height = 320;	//纵坐标点数

            if(LCD_Manage.ID == 0x9341)
            {
      		LCD_Manage.wgramcmd = 0x2c;	//GRAM开始写指令
      		LCD_Manage.setxcmd = 0x2a;	//设置x坐标
      		LCD_Manage.setycmd = 0x2b;	//设置y坐标
            }
            else  //9320、9325
            {
                  LCD_Manage.wgramcmd = Write_Data_to_GRAM; //写数据到GRAM
                  LCD_Manage.setxcmd = Horizontal_GRAM_Address_Set;     //水平方向
                  LCD_Manage.setycmd = Vertical_GRAM_Address_Set; //垂直方向
            }
	}
	else  //横屏显示
	{
		LCD_Manage.dir = 1;	//横屏显示
		LCD_Manage.width = 320;	//横坐标点数
		LCD_Manage.height = 240;	//纵坐标点数

		if(LCD_Manage.ID == 0x9341)
            {
      		LCD_Manage.wgramcmd = 0x2c;	//GRAM开始写指令
      		LCD_Manage.setxcmd = 0x2a;	//设置x坐标
      		LCD_Manage.setycmd = 0x2b;	//设置y坐标
            }
            else
            {
                  LCD_Manage.wgramcmd = Write_Data_to_GRAM; //写数据到GRAM
                  LCD_Manage.setxcmd = Horizontal_GRAM_Address_Set;     //水平方向
                  LCD_Manage.setycmd = Vertical_GRAM_Address_Set; //垂直方向
            }
	}
	LCD_Scan_DIR(DEF_Scan_DIR);	//默认扫描方向
}
/**********************************************************
* 函数功能 ---> LCD初始化
* 入口参数 ---> none
* 返回数值 ---> none
* 功能说明 ---> none
**********************************************************/
void LCD_Init()
{
	LCD_RST_BL_Init();      //初始化背光灯控制和复位管脚控制

    /******************************************************
	                   控制方式选择
	******************************************************/  
	#if _USER_GPIO_OR_FSMC      //定义了则使用FSMC控制

		STM32_FSMC_Init();      //初始化FSMC

	#else //否则使用GPIO控制

		LCD_GPIO_Init();  //初始化通讯、控制GPIO

	#endif
	/*******************我是美丽的分割线******************/

	/******************************************************
	                   复位方式选择
	******************************************************/
	#if LCD_RESET_Soft   //定义了则使用软件控制复位

		LCD_RST = 0;
		delay_ms(8);
		LCD_RST = 1;
		delay_ms(8);

	#endif
	/*******************我是美丽的分割线******************/

	delay_ms(50);     //等待相关寄存器完成初始化
	LCD_WriteRegisterData(0x00, 0x01);    //软件复位
	delay_ms(50);

	LCD_Manage.ID = LCD_ReadRegisterData(0x0000);      //读取LCD型号

	if((LCD_Manage.ID < 0xff) || (LCD_Manage.ID == 0xffff) || (LCD_Manage.ID == 0x9300))
	{	//读取ID不正确
		//尝试读取9341
		LCD_Write_Register(0xd3);     //重新读取
		LCD_Read_Data();  //假读一次
		LCD_Read_Data();
		LCD_Manage.ID = LCD_Read_Data();    //读取到93
		LCD_Manage.ID <<= 8;
		LCD_Manage.ID |= LCD_Read_Data();   //读取41

//		if(LCD_Manage.ID != 0x9341)   LCD_Manage.ID = 0x9341;
	}

	printf("\r\nLCD Drive ID is: %x\r\n",LCD_Manage.ID);      //打印ID到串口

	if(LCD_Manage.ID == 0x9341)
	{
//		LCD_Write_Register(LCD_ExitSleep);	//退出睡眠模式，这句可放最后
//		delay_ms(120);
		
		LCD_Write_Register(0xcf);
		LCD_Write_Data(0x00);
		LCD_Write_Data(0xc1);	//参数可改为81
		LCD_Write_Data(0x30);
		
		LCD_Write_Register(0xed);
		LCD_Write_Data(0x64);
		LCD_Write_Data(0x03);
		LCD_Write_Data(0x12);
		LCD_Write_Data(0x81);

		LCD_Write_Register(0xe8);
		LCD_Write_Data(0x85);
		LCD_Write_Data(0x10);	//参数可改为00
		LCD_Write_Data(0x7a);	//参数可改为79
		
		LCD_Write_Register(0xcb);
		LCD_Write_Data(0x39);
		LCD_Write_Data(0x2c);
		LCD_Write_Data(0x00);
		LCD_Write_Data(0x34);
		LCD_Write_Data(0x02);

		LCD_Write_Register(0xf7);
		LCD_Write_Data(0x20);
		
		LCD_Write_Register(0xea);
		LCD_Write_Data(0x00);
		LCD_Write_Data(0x00);
		
		LCD_Write_Register(0xc0);	//power control
		LCD_Write_Data(0x1b);	//VRH[5:0]
	//	LCD_Write_Data(0x2e);	//参数可改为21

		LCD_Write_Register(0xc1);	//power control
		LCD_Write_Data(0x01);	//SAP[2:0],BT[3:0](11)
	//	LCD_Write_Data(0x12);	//参数可改为13
		
		LCD_Write_Register(0xc5);	//VCM control
		LCD_Write_Data(0x30);
		LCD_Write_Data(0x30);
	//	LCD_Write_Data(0x50);	//参数可改为3f
	//	LCD_Write_Data(0x19);	//参数可改为3c
		
		LCD_Write_Register(0xc7);	//VCM control2
		LCD_Write_Data(0xb7);
	//	LCD_Write_Data(0x90);	//参数可改为b3/90

		///////////////////////////////////////////////////

            //新增以下两命令
		LCD_Write_Register(0x2a);	//命令1
		LCD_Write_Data(0x00);
		LCD_Write_Data(0x00);
		LCD_Write_Data(0x00);
	//	LCD_Write_Data(0x01);
	//	LCD_Write_Data(0x3f);

		LCD_Write_Register(0x2b);	//命令2
		LCD_Write_Data(0x00);
		LCD_Write_Data(0x00);
		LCD_Write_Data(0x01);
	//	LCD_Write_Data(0xef);
		///////////////////////////////////////////////////

		LCD_Write_Register(0x36);	//Memory Access Control
		LCD_Write_Data(0x48);
	//	LCD_Write_Data(0xa8);

		///////////////////////////////////////////////////
		//新增
		LCD_Write_Register(0x3a);	//命令3
		LCD_Write_Data(0x55);	
		///////////////////////////////////////////////////
		
		LCD_Write_Register(0xb1);
		LCD_Write_Data(0x00);
		LCD_Write_Data(0x1a);
	//	LCD_Write_Data(0x14);	//参数可改为1b、29、14

		LCD_Write_Register(0xb6);	//Display Function Control
		LCD_Write_Data(0x0a);
		LCD_Write_Data(0xa2);
		
		LCD_Write_Register(0xf2);	//3Gamma Function Disable
		LCD_Write_Data(0x00);
		
		LCD_Write_Register(0x26);	//Gamma curve selected
		LCD_Write_Data(0x01);

		LCD_Write_Register(0xe0);	//Set Gamma
		LCD_Write_Data(0x0f);
		LCD_Write_Data(0x2a);
		LCD_Write_Data(0x28);
		LCD_Write_Data(0x08);
		LCD_Write_Data(0x0e);
		LCD_Write_Data(0x08);
		LCD_Write_Data(0x54);
		LCD_Write_Data(0xa9);
		LCD_Write_Data(0x43);
		LCD_Write_Data(0x0a);
		LCD_Write_Data(0x0f);
		LCD_Write_Data(0x00);
		LCD_Write_Data(0x00);
		LCD_Write_Data(0x00);
		LCD_Write_Data(0x00);

		LCD_Write_Register(0xe1);	//Set Gamma
		LCD_Write_Data(0x00);
		LCD_Write_Data(0x15);
		LCD_Write_Data(0x17);
		LCD_Write_Data(0x07);
		LCD_Write_Data(0x11);
		LCD_Write_Data(0x06);
		LCD_Write_Data(0x2b);
		LCD_Write_Data(0x56);
		LCD_Write_Data(0x3c);
		LCD_Write_Data(0x05);
		LCD_Write_Data(0x10);
		LCD_Write_Data(0x0f);
		LCD_Write_Data(0x3f);
		LCD_Write_Data(0x3f);
		LCD_Write_Data(0x0f);

		LCD_Exit_Sleep();	//退出睡眠，打开显示
		delay_ms(120);
		LCD_Display_ON(); //打开显示

	/*	LCD_Write_Register(LCD_ExitSleep);	//退出睡眠模式，这句也可以放初始化之前
		delay_ms(120);
	*/
	//	LCD_Write_Register(0x29);	//打开显示
//		delay_ms(10);

//		LCD_Write_Register(0x2c);	//GRAM开始写指令
	}

	else if(LCD_Manage.ID == 0x9325)
	{
		LCD_WriteRegisterData(0x0001,0x0100); 
		LCD_WriteRegisterData(0x0002,0x0700); 
		LCD_WriteRegisterData(0x0003,0x1030); 
		LCD_WriteRegisterData(0x0004,0x0000); 
		LCD_WriteRegisterData(0x0008,0x0207);  
		LCD_WriteRegisterData(0x0009,0x0000);
		LCD_WriteRegisterData(0x000A,0x0000); 
		LCD_WriteRegisterData(0x000C,0x0000); 
		LCD_WriteRegisterData(0x000D,0x0000);
		LCD_WriteRegisterData(0x000F,0x0000);

		//power on sequence VGHVGL
		LCD_WriteRegisterData(0x0010,0x0000);   
		LCD_WriteRegisterData(0x0011,0x0007);  
		LCD_WriteRegisterData(0x0012,0x0000);  
		LCD_WriteRegisterData(0x0013,0x0000);

		//vgh 
		LCD_WriteRegisterData(0x0010,0x1290);   
		LCD_WriteRegisterData(0x0011,0x0227);

		//vregiout 
		LCD_WriteRegisterData(0x0012,0x001d); //0x001b

		//vom amplitude
		LCD_WriteRegisterData(0x0013,0x1500);

		//vom H
		LCD_WriteRegisterData(0x0029,0x0018); 
		LCD_WriteRegisterData(0x002B,0x000D);

		//gamma
		LCD_WriteRegisterData(0x0030,0x0004);
		LCD_WriteRegisterData(0x0031,0x0307);
		LCD_WriteRegisterData(0x0032,0x0002);// 0006
		LCD_WriteRegisterData(0x0035,0x0206);
		LCD_WriteRegisterData(0x0036,0x0408);
		LCD_WriteRegisterData(0x0037,0x0507); 
		LCD_WriteRegisterData(0x0038,0x0204);//0200
		LCD_WriteRegisterData(0x0039,0x0707); 
		LCD_WriteRegisterData(0x003C,0x0405);// 0504
		LCD_WriteRegisterData(0x003D,0x0F02);
            
		//ram
		LCD_WriteRegisterData(0x0050,0x0000); 
		LCD_WriteRegisterData(0x0051,0x00EF);
		LCD_WriteRegisterData(0x0052,0x0000); 
		LCD_WriteRegisterData(0x0053,0x013F);  
		LCD_WriteRegisterData(0x0060,0xA700); 
		LCD_WriteRegisterData(0x0061,0x0001); 
		LCD_WriteRegisterData(0x006A,0x0000);
            
		//
		LCD_WriteRegisterData(0x0080,0x0000); 
		LCD_WriteRegisterData(0x0081,0x0000); 
		LCD_WriteRegisterData(0x0082,0x0000); 
		LCD_WriteRegisterData(0x0083,0x0000); 
		LCD_WriteRegisterData(0x0084,0x0000); 
		LCD_WriteRegisterData(0x0085,0x0000); 

		//
		LCD_WriteRegisterData(0x0090,0x0010); 
		LCD_WriteRegisterData(0x0092,0x0600); 
		LCD_WriteRegisterData(0x0093,0x0003); 
		LCD_WriteRegisterData(0x0095,0x0110); 
		LCD_WriteRegisterData(0x0097,0x0000); 
		LCD_WriteRegisterData(0x0098,0x0000);
		LCD_WriteRegisterData(0x0007,0x0133);
	}

	LCD_Display_DIR(0);	//设置显示方向
	LCD_BLControl = 1;	//打开背光
	
	LCD_Clear(Back_color);	//设置默认背景颜色
}
/**********************************************************
* 函数功能 ---> LCD清屏函数
* 入口参数 ---> color：填充颜色
* 返回数值 ---> none
* 功能说明 ---> 填充指定的color，清除显示
**********************************************************/	
void LCD_Clear(u16 color)
{
	u32 index;
	u32 total_point;

	total_point = LCD_Manage.width * LCD_Manage.height;	//计算显示屏面积，总点数

	LCD_Set_xy(0x0000,0x0000);	//光标定在坐标原点[0:0]
	LCD_WriteGRAM();	//GRAM start writing
	for(index = 0;index < total_point;index++)	LCD_WR_Data(color);	//开始填充颜色
}
/**********************************************************
* 函数功能 ---> LCD画点函数
* 入口参数 ---> (x,y)光标坐标数值
* 返回数值 ---> none
* 功能说明 ---> 点的颜色在前面的画笔颜色设置
**********************************************************/
void LCD_DrawPoint(u16 x,u16 y)
{
	LCD_Set_xy(x,y);	//设定光标坐标
	LCD_WriteGRAM();	//GRAM start writing
	LCD_WR_Data(Point_color);	//写入点的颜色值
}      
/**********************************************************
* 函数功能 ---> LCD画点函数，颜色值随设定写入
* 入口参数 ---> (x,y)光标坐标数值
*               color：要写入的颜色值
* 返回数值 ---> none
* 功能说明 ---> none
**********************************************************/					
void LCD_DrawPoint_Color(u16 x,u16 y,u16 color)
{
	LCD_Set_xy(x,y);	//设定光标坐标
	LCD_WriteGRAM();	//GRAM start writing
	LCD_WR_Data(color);	//写入点的颜色值
}
/**********************************************************
* 函数功能 ---> LCD画线函数
* 入口参数 ---> (x1,y1)起点坐标数值
*               (x2,y2)终点坐标数值
* 返回数值 ---> none
* 功能说明 ---> 线的颜色在前面的画笔颜色设置
**********************************************************/
void LCD_Draw_Line(u16 x1,u16 y1,u16 x2,u16 y2)
{
	u16 i;	//循环画线变量
	int x_err,y_err;
	int diff_x,	/*	x坐标增量	*/
		diff_y,	/*	y坐标增量	*/
		distance;	/*	画线距离	*/

	int inc_x,	/*	x轴画线方向	*/
		inc_y;	/*	y轴画线方向	*/

	int brush_x,	/*	画笔起点坐标	*/
		brush_y;

	brush_x = x1;	//设置画笔起点坐标
	brush_y = y1;

	diff_x = x2 - x1;	//计算x变化量
	diff_y = y2 - y1;	//计算y变化量

	if(diff_x > 0)	inc_x = 1;	//x轴方向，向正方向
	else	if(diff_x == 0)	inc_x = 0;	//x轴方向不变，画垂直线
	else
	{	inc_x = -1;diff_x = -diff_x;	}	//向x轴反方向

	if(diff_y > 0)	inc_y = 1;	//y轴方向，向正方向
	else	if(diff_y == 0)	inc_y = 0;	//y轴方向不变，画水平线
	else
	{	inc_y = -1;diff_y = -diff_y;	}	//向y轴反方向

      //画线判断（象限法判断）
	if(diff_x > diff_y)	distance = diff_x;	//画大于315°小于455°、大于135°小于225°直线
	else	distance = diff_y;	//画大于45°小于135°、大于225°小于315°直线

	for(i = 0;i < distance+1;i++)	//画线开始
	{
		LCD_DrawPoint(brush_x,brush_y);	//打点开始
		x_err += diff_x;
		y_err += diff_y;
		
		if(x_err > distance)	//x轴方向画笔控制
		{
			x_err -= distance;
			brush_x += inc_x;
		}

		if(y_err > distance)	//y轴方向画笔控制
		{
			y_err -= distance;
			brush_y += inc_y;
		}
	}
}
/**********************************************************
* 函数功能 ---> LCD画矩形函数
* 入口参数 ---> (x1,y1)起角坐标数值
*               (x2,y2)对角坐标数值
* 返回数值 ---> none
* 功能说明 ---> 线的颜色在前面的画笔颜色设置
**********************************************************/
void LCD_Draw_Quad(u16 x1,u16 y1,u16 x2,u16 y2)
{
	LCD_Draw_Line(x1,y1,x2,y1);	//以y1为轴画线
	LCD_Draw_Line(x2,y1,x2,y2);	//以x2为轴画线
	LCD_Draw_Line(x2,y2,x1,y2);	//以y2为轴画线
	LCD_Draw_Line(x1,y2,x1,y1);	//以x1为轴画线
}
/**********************************************************
* 函数功能 ---> LCD指定区域填充颜色函数
* 入口参数 ---> (x1,y1)起角坐标数值
*               (x2,y2)对角坐标数值
*               color：填充颜色值
* 返回数值 ---> none
* 功能说明 ---> 在屏幕上画一个四边形并填充相应的颜色
*               块区域大小（面积） = (x2 - x1) * (y2 - y1)
**********************************************************/
void LCD_Area_Color(u16 x1,u16 y1,u16 x2,u16 y2,u16 color)
{
	u16 i,j;
	u16 x_len;
	
	x_len = x2 - x1 + 1;	//计算X轴方向增量
	
	for(i = y1;i < y2;i++)	//y轴方向跨越宽度
	{
		LCD_Set_xy(x1,i);	//设定光标起点
		LCD_WriteGRAM();	//GRAM start writing
		for(j = 0;j < x_len;j++)	LCD_Write_Data(color);	//更新光标位置，写入颜色
	}
}
/**********************************************************
* 函数功能 ---> LCD中点法画圆函数
* 入口参数 ---> (x,y)起角坐标数值
*               r：圆半径
* 返回数值 ---> none
* 功能说明 ---> 圆的颜色在前面的画笔颜色设置
**********************************************************/	
void LCD_Draw_Circle(u16 x0,u16 y0,u16 r)
{
	u8 x,y;	//轴方向变量
	float d;
	x = 0;
	y = r;
	d = 5.0/4 - r;	//逆时针画圆

	while(x <= y)
	{
		LCD_DrawPoint(x0 + x,y0 + y);	//270°
		LCD_DrawPoint(x0 + x,y0 - y);	//90°
		LCD_DrawPoint(x0 - x,y0 + y);
		LCD_DrawPoint(x0 - x,y0 - y);
		LCD_DrawPoint(x0 + y,y0 + x);	//0°
		LCD_DrawPoint(x0 + y,y0 - x);
		LCD_DrawPoint(x0 - y,y0 + x);	//180°
		LCD_DrawPoint(x0 - y,y0 - x);
		
		if(d < 0)	//y轴衰减慢
		{	d += x * 2.0 + 3;	}
		else		//x轴衰减慢
		{
			d += 2.0 * (x - y) + 5;
			y--;
		}
		x++;
	}
}
/**********************************************************
* 函数功能 ---> LCD指定位置显示一个字符函数
* 入口参数 ---> (x,y)坐标数值
*               ch：要显示的字符
*               mode：0：非叠加方式显示。1：叠加方式显示
* 返回数值 ---> none
* 功能说明 ---> 主要显示一些需要变化之类的数字显示
**********************************************************/
void LCD_Draw_Char(u16 x,u16 y,u8 ch,u8 mode)
{
	u8 temp;
	u8 i,j;
	u16 x0=x;
	u16 colortemp=Point_color;

	u32 address = (ch - 0x20) * 16;

	if((x > LCD_Manage.width) || (y > LCD_Manage.height))		return;	//输入的坐标超出范围，直接退出
	if((ch < 0x20)&&(ch >0x80))	return;	//显示的字符超出英文字符范围，直接退出 

	if(!mode) //非叠加方式
	{
		for(i = 0;i < 16;i++)	//行控制
		{
			temp = ascii_8x16[address + i]; //调用1608字体 		 

			for(j = 0;j < 8;j++)	//列控制
			{                 
				if((temp >> 7 - j) & 0x01 == 0x01) Point_color = colortemp;	//字符颜色	
				else	Point_color = Back_color;	//背景颜色

				LCD_DrawPoint(x+j,y+i);	//开始打点

				if((x+j) >= LCD_Manage.width)	return;	//超出液晶宽度，直接退出
				if((y+i) >= LCD_Manage.height)	return;	//超出液晶高度，直接退出
			}     //end for j
			x=x0;
		}     //end for i	
	}
	else//叠加方式
	{
		for(i = 0;i < 16;i++)	//行控制
		{
			temp = ascii_8x16[address + i]; //调用1608字体		 

			for(j = 0;j < 8;j++)	//列控制
			{ 
				if((temp >> 7 - j) & 0x01 == 0x01) LCD_DrawPoint(x+j,y+i);//画一个点
				
				if((x+j) >= LCD_Manage.width)	return;	//超出液晶宽度，直接退出
				if((y+i) >= LCD_Manage.height)	return;	//超出液晶高度，直接退出  
			}     //end for j
		}     //end for i
	}
	Point_color = colortemp;	//恢复画笔颜色	    	   	 	  
}
/**********************************************************
* 函数功能 ---> LCD指定位置显示一个字符，字符、背景颜色随机设定
* 入口参数 ---> (x,y)坐标数值
*               ch：要显示的字符
*               charcolor：字符颜色
*               bkcolor：字符背景颜色
* 返回数值 ---> none
* 功能说明 ---> 主要显示一些固定不需要变化的字符或者数字
*               在(x,y)坐标上显示一个8x16的英文字符或者数字并指定其颜色及背景颜色
**********************************************************/
void LCD_Draw_Char_BK(u16 x,u16 y,u8 ch,u8 size,u16 charcolor,u16 bkcolor)
{
	u8 i,j;
	u8 temp_char=0;
	u32 address = (ch - 0x20) * size;

	if((x > LCD_Manage.width) || (y > LCD_Manage.height))		return;	//输入的坐标超出范围，直接退出
	
	if((ch < 0x20)&&(ch >0x80))	return;	//显示的字符超出英文字符范围，直接退出

	for(i = 0;i < 16;i++)	//行控制
	{
		temp_char = ascii_8x16[address + i];

		for(j = 0;j < 8;j++)	//列控制
		{
			if((temp_char >> 7 - j) & 0x01 == 0x01)	LCD_DrawPoint_Color(x+j,y+i,charcolor); // 字符颜色
			else	LCD_DrawPoint_Color(x+j,y+i,bkcolor); // 背景颜色
		}     //end for j
	}     //end for i
}
/**********************************************************
* 函数功能 ---> LCD显示一个汉字，背景颜色随机设定
* 入口参数 ---> (x,y)起角坐标数值
*               *ch：要显示的字符
*               fontsize：字体大小
*               charcolor：字符颜色
*               bkcolor：字符背景颜色
* 返回数值 ---> none
* 功能说明 ---> 在(x,y)坐标上显示一个16 x 16的中文汉字并指定其颜色及背景颜色
**********************************************************/
void LCD_Draw_GB_BK(u16 x,u16 y,u8 *ch,u8 fontsize,u16 charcolor,u16 bkcolor)
{
	static u8 FONTbuff[32];	//汉子字模数据缓存
	u8 i,j; 
	u8 temp_char=0;
	int num=0;

	if((x > LCD_Manage.width) || (y > LCD_Manage.height))		return;	//输入的坐标超出范围，直接退出
	
	Get_Hzlib(ch, FONTbuff, fontsize);	//读取字模数据
    
	for(i = 0;i < 16;i++)	//显示一个16 * 16的汉字
	{
		temp_char = FONTbuff[num];

		for(j = 0;j < 8;j++)	//显示汉字的左半边，8 * 16
		{
			if((temp_char >> 7 - j) & 0x01 == 0x01)	LCD_DrawPoint_Color(x+j,y+i,charcolor); // 字符颜色
			else	LCD_DrawPoint_Color(x+j,y+i,bkcolor); // 背景颜色
		}     //end for j
		num++;

		temp_char = FONTbuff[num];

		for(j = 0;j < 8;j++)	//显示汉字的右半边，8 * 16
		{
			if((temp_char >> 7 - j) & 0x01 == 0x01)	LCD_DrawPoint_Color(x+j+8,y+i,charcolor); // 字符颜色
			else	LCD_DrawPoint_Color(x+j+8,y+i,bkcolor); // 背景颜色
		}     //end for j
		num++;
	} 
}
/**********************************************************
* 函数功能 ---> LCD显示汉字或字符、背景颜色随机设定
* 入口参数 ---> (x,y)起角坐标数值
*               ch[2]：要显示的字符
*               charcolor：字符颜色
*               bkcolor：字符背景颜色
* 返回数值 ---> none
* 功能说明 ---> 在(x,y)坐标上显示字符串并指定其颜色，判断中文显示还是英文显示
**********************************************************/	
void LCD_Display_String_BK(u16 x,u16 y,u8* ch,u8 size,u16 charcolor,u16 bkcolor)
{
	for(;*ch != '\0';ch++)
	{
		if((*ch & 0x80) == 0)	//显示英文
		{
			LCD_Draw_Char_BK(x,y,*ch,size,charcolor,bkcolor);
			x += 8;
		}
		else	//显示中文
		{
			LCD_Draw_GB_BK(x,y,ch,size,charcolor,bkcolor);
			x += 16;
			ch++;
		}
	}
}
/**********************************************************
* 函数功能 ---> 一元多次方程计算
* 入口参数 ---> x：底数
*               n：指数
* 返回参数 ---> none 
* 功能说明 ---> x的n次方
**********************************************************/
u32 Equation_Calculate(u8 x,u8 n)
{
	u32 dat=1;
	while(n--)	dat *= x;	//n个x相乘
	return dat;
}
/**********************************************************
* 函数功能 ---> LCD显示数字，高位为0时不显示
* 入口参数 ---> (x,y)显示起始坐标
*               arr：要显示的数字，范围0 ~ 4294967295
*               sum：显示数字位数，数字低位都会显示，设置该值只是屏蔽高位
*                    比如显示数字65535。显示3位的话结果就是535
*               mode：显示效果设置。0：不叠加。1：叠加
* 返回参数 ---> none 
* 功能说明 ---> 在(x,y)坐标上显示数字并指定其颜色
**********************************************************/
void LCD_Display_Array_No0(u16 x,u16 y,u32 arr,u8 sum,u8 mode)
{
	u8 m,temp=0;
	u16 arr_temp;

	for(m = 0;m < sum;m++)
	{
		arr_temp = (arr / (Equation_Calculate(10,(sum - m - 1)))) % 10;	//取得最低位数字

		if((temp == 0) && (m < (sum -1)))	//是否最后一位数字
		{
			if(arr_temp == 0)	//取得的数字为0
			{
				LCD_Draw_Char(x+8*m,y,0x20,mode);	//取得的数字高位是0则显示空格
				continue;	//不显示最高位
			}
			else	temp = 1;
		}
		if(x + 8*m > LCD_Manage.width)	//超出显示屏宽度了
		{
			if(y+16 > LCD_Manage.height)	//超出显示屏显示范围
			{	break;	}	//退出，剩下内容不显示了

			LCD_Draw_Char(x+8*m,y+16,(arr_temp + 0x30),mode);	//在下一行显示
		}
		else	LCD_Draw_Char(x+8*m,y,(arr_temp + 0x30),mode);	//没超出范围，在本行显示
	}     //end for m
}
/**********************************************************
* 函数功能 ---> LCD显示数字，高位为0时显示0
* 入口参数 ---> (x,y)显示起始坐标
*               arr：要显示的数字，范围0 ~ 4294967295
*               sum：显示数字位数
*               mode：显示效果设置。0：不叠加。1：叠加
* 返回参数 ---> none 
* 功能说明 ---> 在(x,y)坐标上显示数字并指定其颜色
**********************************************************/
void LCD_Display_Array_Yes0(u16 x,u16 y,u32 arr,u8 sum,u8 mode)
{
	u8 m,temp=0;
	u16 arr_temp;

	for(m = 0;m < sum;m++)
	{
		arr_temp = (arr / (Equation_Calculate(10,(sum - m - 1)))) % 10;	//取得最高位数字

		if((temp == 0) && (m < (sum -1)))	//是否最后一位数字
		{
			if(arr_temp == 0)	//取得的数字为0
			{
				LCD_Draw_Char(x+8*m,y,(arr_temp + 0x30),mode);	//高位显示0
				continue;
			}
			else	temp = 1;
		}
		if(x + 8*m > LCD_Manage.width)	//超出显示屏宽度了
		{
			if(y+16 > LCD_Manage.height)	//超出显示屏显示范围
			{	break;	}	//退出，剩下内容不显示了

			LCD_Draw_Char(x+8*m,y+16,(arr_temp + 0x30),mode);	//在下一行显示	 
		}
		else	LCD_Draw_Char(x+8*m,y,(arr_temp + 0x30),mode);	//没超出范围，在本行显示
	}     //end for m
}


/**********************************************************
* 函数功能 ---> LCD显示一个数字串函数，显示字体、背景颜色随机设定
* 入口参数 ---> (x,y)起角坐标数值
*               ch[2]：要显示的字符
*               sum: 显示位数，数字低位都会显示，设置该值只是屏蔽高位
*                    比如显示数字00655.35。显示4位的话结果就是5.35
* 返回数值 ---> none
* 功能说明 ---> A、主要显示一个数字串等，非叠加方式显示
*               B、显示小数时候那个显示位数包括小数点
**********************************************************/
void LCD_Display_String_Num(u16 x,u16 y,u8 *ch,u8 sum)
{
	for(;sum > 0;sum--)
	{
		LCD_Draw_Char(x,y,*ch,0);	//输出字符，从高位开始
		x += 8;
		ch++;
	}
}

/**********************************************************
* 函数功能 ---> LCD显示一个数字串函数，显示位数、字体、背景颜色随机设定
* 入口参数 ---> (x,y)起角坐标数值
*               ch[2]：要显示的字符
*               fontsize：字体大小
*               chlen：显示数字字符串总长度，长度值必须由sizeof()-1函数获得或者直接数出来的
*               sum: 显示位数，数字低位都会显示，设置该值只是屏蔽高位
*                    比如显示数字00655.35。显示4位的话结果就是5.35
*               charcolor：字体颜色
*               bkcolor：显示字体背景颜色
* 返回数值 ---> none
* 功能说明 ---> A、主要显示一个数字串等，非叠加方式显示
*               B、显示小数时候那个显示位数包括小数点
**********************************************************/
void LCD_Display_String_Num1(u16 x,u16 y,u8 *ch,u8 fontsize,u16 chlen,u16 sum,u16 charcolor,u16 bkcolor)
{
	u16 i;
	
	if(chlen == sum)	//全部显示
	{
		for(;sum > 0;sum--)
		{
//			LCD_Draw_Char(x,y,*ch,0);	//输出字符
			LCD_Draw_Char_BK(x, y, *ch, fontsize, charcolor, bkcolor);
			x += 8;
			ch++;
		}
	}
	else	//只显示部分，从最低位开始数起sum位要显示
	{
		for(i = chlen-sum;i < chlen;i++)	//从倒数的第sum个字符开始显示
		{
//			LCD_Draw_Char(x,y,ch[i],0);	//输出字符
			LCD_Draw_Char_BK(x, y, ch[i], fontsize, charcolor, bkcolor);
			x += 8;
		}
	}
}

