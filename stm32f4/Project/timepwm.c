
#include "stm32f4xx.h"
#include "led.h"
#include "delay.h"
#include "key.h"
#include "usart.h"
#include "gpio.h" 
#include "mbi5153.h"

#if(TWO_TIMER_PULSE==1)
/*��ʱ��1��ģʽ*/
void TIM1_config(u32 Cycle)
{

    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 , ENABLE); 
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE); 	//��ʼ��GPIOGʱ��

    GPIO_PinAFConfig(GPIOE,GPIO_PinSource9,GPIO_AF_TIM1); //GPIOF9??????14

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;					//����
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;				//�������
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;					//����
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    TIM_TimeBaseStructure.TIM_Period = Cycle-1;
    TIM_TimeBaseStructure.TIM_Prescaler =21;                    //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ                                    
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;     //����ʱ�ӷָ�:TDTS= Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM���ϼ���
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;            //�ظ�������һ��Ҫ=0!!!
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);                                       

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;          //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ1    
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
    TIM_OCInitStructure.TIM_Pulse = Cycle/2-1;                    //���ô�װ�벶��Ĵ���������ֵ                    
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;      //�������
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);

    TIM_SelectMasterSlaveMode(TIM1, TIM_MasterSlaveMode_Enable);
    TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_Update);

    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM1, ENABLE);
}

/***???2???***/
void TIM2_config(u32 PulseNum)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure; 
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    TIM_TimeBaseStructure.TIM_Period = PulseNum-1;
    TIM_TimeBaseStructure.TIM_Prescaler =0;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    TIM_SelectInputTrigger(TIM2, TIM_TS_ITR0);
    //TIM_InternalClockConfig(TIM3);
    TIM2->SMCR|=0x07;                                  //???????? 
    //TIM_ITRxExternalClockConfig(TIM2, TIM_TS_ITR0);

    //TIM_ARRPreloadConfig(TIM3, ENABLE);
    TIM_ITConfig(TIM2,TIM_IT_Update,DISABLE);

   // NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void Pulse_output(u32 Cycle,u32 PulseNum)
{
	printf("Pulse_output Cycle %d,PulseNum %d\r\n",Cycle,PulseNum);
#if 1
    TIM2_config(PulseNum);
    TIM_Cmd(TIM2, ENABLE);
    TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
    TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
#endif
    TIM1_config(Cycle);
    TIM_Cmd(TIM1, ENABLE);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);   //??????????,?????
}

int8_t pre_vsync=0,gclk_num=5;

void TIM2_IRQHandler(void)
{

    printf("TIM2_IRQHandler gclk_num %d\r\n",gclk_num);

    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)     // TIM_IT_CC1
    {
        printf("TIM2_IRQHandler SET\r\n");
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update); // ??????? 
        TIM_CtrlPWMOutputs(TIM1, DISABLE);  //?????
        TIM_Cmd(TIM1, DISABLE); // ????? 
        TIM_Cmd(TIM2, DISABLE); // ????? 
        TIM_ITConfig(TIM2, TIM_IT_Update, DISABLE);

        gclk_num++;

        if(pre_vsync == 0 && gclk_num < GCLKNUM-1)
        {
            //Pulse_output(100,10);
            TIM_Cmd(TIM2, ENABLE);
            TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
            TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
            IM_Cmd(TIM1, ENABLE);
            TIM_CtrlPWMOutputs(TIM1, ENABLE);
        }
    }
}
#else

#if 0
//TIM14 PWM????? 
//PWM?????
//arr:?????
//psc:??????
void TIM14_PWM_Init(u32 arr,u32 psc)
{
	//????????IO???
	
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14,ENABLE);  	//TIM14????
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE); 	//??PORTF??	

	GPIO_PinAFConfig(GPIOF,GPIO_PinSource9,GPIO_AF_TIM14); //GPIOF9??????14

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;           //GPIOF9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //????
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//??100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //??????
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //??
	GPIO_Init(GPIOF,&GPIO_InitStructure);              //???PF9

	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //��ʱ����Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���
	TIM_TimeBaseStructure.TIM_Period=arr;   //�Զ�����
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM14,&TIM_TimeBaseStructure);//??????14
	//???TIM14 Channel1 PWM??
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //???????:TIM????????2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //??????
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //????:TIM???????
	TIM_OC1Init(TIM14, &TIM_OCInitStructure);  //??T??????????TIM1 4OC1

	TIM_OC1PreloadConfig(TIM14, TIM_OCPreload_Enable);  //??TIM14?CCR1????????

	TIM_ARRPreloadConfig(TIM14,ENABLE);//ARPE??
	TIM_Cmd(TIM14, ENABLE);  //??TIM14
}

#else
void TIM1_PWM_Init(u32 arr,u32 psc)
{
	
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 , ENABLE); 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE); 	//??PORTF??	

	GPIO_PinAFConfig(GPIOE,GPIO_PinSource9,GPIO_AF_TIM1); //GPIOF9??????14

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;           //GPIOF9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //????
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//??100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //??????
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;        //??
	GPIO_Init(GPIOE,&GPIO_InitStructure);              //???PF9

	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //��ʱ����Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���
	TIM_TimeBaseStructure.TIM_Period=arr;   //�Զ�����
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseStructure);//??????14
	//???TIM14 Channel1 PWM??
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //???????:TIM????????2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //??????
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //????:TIM???????
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);  //??T??????????TIM1 4OC1

	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);  //??TIM14?CCR1????????

	TIM_ARRPreloadConfig(TIM1,ENABLE);//ARPE??
	TIM_Cmd(TIM1, ENABLE);  //??TIM14
}
#endif
#endif