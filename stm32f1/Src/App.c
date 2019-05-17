/**
  ******************************************************************************
  * File Name          : App.c
  * Description        : Application file for gesture control
  ******************************************************************************
  *Author:  JL
  *Date:    2019/1/18
  ******************************************************************************
  **/
  
#include "main.h"  
#include "stm32f1xx_hal.h"
#include "VCNL4035.h"
#include "I2C.h"
#include "App.h"
#include "stdio.h"
#include "arm_math.h"
#include "arm_const_structs.h"
//#include "math.h"

extern volatile uint16_t I2C_SDA_PIN;
extern volatile uint16_t I2C_SCL_PIN;

static uint32_t tickstart;
uint64_t sys_tick=0;                                //ϵͳtick����
gs_struct gs={0};
uint16_t led_counter=0;
//uint16_t led
uint8_t led_flag=0;

//������ʼ��
void App_Var_Init(void)
{
    gs.sample_counter=0;
    gs.sample_ready_flag=0;
    gs.sample_start_flag=0;
    gs.sample_frequency=SAMPLE_FREQUENCY_DEF;
    gs.sample_size=GES_SAMPLE_ARRAY_NUM_S;
    gs.sample_size_raw=GES_SAMPLE_RAW_ARRAY_NUM_S;
    for (uint8_t i=0;i<GES_CHN_NUM;i++)
    {
        gs.sample_down_limit[i]=SAMPLE_DOWN_LIMIT_DEF;
        gs.sample_up_limit[i]=SAMPLE_UP_LIMIT_DEF;
    }
}

//Ӧ�ó�ʼ��
void App_Init(void)
{
    //uint16_t read_val;
    //uint8_t res;

    App_Var_Init();

    tickstart = HAL_GetTick();
    while((HAL_GetTick() - tickstart) <= 200)//delay
        ;

#if(SENSOR3==1)
    I2C_SDA_PIN = SDA1_Pin;
    I2C_SCL_PIN = SCL1_Pin;
    I2C_init();
    DEMO_Init();
    I2C_SDA_PIN = SDA2_Pin;
    I2C_SCL_PIN = SCL2_Pin;
    I2C_init();
    DEMO_Init();
    I2C_SDA_PIN = SDA3_Pin;
    I2C_SCL_PIN = SCL3_Pin;
    I2C_init();
    DEMO_Init();
#else
    I2C_SDA_PIN = SDA3_Pin;
    I2C_SCL_PIN = SCL3_Pin;
    I2C_init();
    DEMO_Init();
#endif
    LOG_DEBUG("App_Init!\r\n");
    
    //res=VCNL4035_ReadData(VCNL4035_ID,&read_val);
    //LOG_DEBUG("Read res: %d, ID: 0x%04x\n",res,read_val);

    //VCNL4035_InitGesMode();
}

void LED_Tick_Check(void)
{
    if (led_counter)
    {
        led_counter--;
        if (!led_counter)
        {
            HAL_GPIO_WritePin(LED_LEFT_GPIO_Port, LED_LEFT_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(LED_RIGHT_GPIO_Port, LED_RIGHT_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(LED_UP_GPIO_Port, LED_UP_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(LED_DOWN_GPIO_Port, LED_DOWN_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(LED_DCLK_GPIO_Port, LED_DCLK_Pin, GPIO_PIN_SET);
        }
    }
}

//����һ�β���ֵ
//���룺     ps->��ȡ�Ĳ���ֵ����
//���أ�     0-->����δ������
//          1-->������������
uint8_t Ges_Add_Sample(uint16_t *ps)
{
    uint8_t i,j,k;
    uint32_t sum;
    uint16_t ges_sample_raw_cur_temp;

//  res=gs.ges_cur;
//  ges_flag=0;
    for (i=0;i<GES_CHN_NUM;i++)
    {   
        ges_sample_raw_cur_temp=gs.ges_sample_raw_cur;
        gs.ges_sample_raw_array[i][ges_sample_raw_cur_temp]=ps[i];
        ges_sample_raw_cur_temp++;
        //raw������������
        if (ges_sample_raw_cur_temp>=gs.sample_size_raw)
        {
            for (k=0;k<gs.sample_size;k++)
            {
                for (j=0,sum=0;j<GES_SAMPLE_AVE_NUM;j++)
                {
                    sum+=gs.ges_sample_raw_array[i][k*GES_SAMPLE_AVE_NUM+j];
                }
                gs.ges_sample_array[i][k]=(uint16_t)(sum/GES_SAMPLE_AVE_NUM);
            }
        }       
    }
    //��ÿһ֡�Ŀ�ʼ��¼ʱ��
    if (gs.ges_sample_raw_cur==0)
        gs.frame_start_tick=Systick_Get();
    
    gs.ges_sample_raw_cur++;
    if (gs.ges_sample_raw_cur>=gs.sample_size_raw)
    {
        gs.ges_sample_raw_cur=0;
        return 1;
    }
    else
        return 0;
}

//���Ʋ�������
void Ges_Tick_Check(void)
{
    gs.sample_counter++;
    if (gs.sample_counter>=gs.sample_frequency)
    {
        gs.sample_counter=0;
        gs.sample_start_flag=1;
    }
}

//�жϴ�����
void Ges_It_Handle(void)
{
    if (__HAL_GPIO_EXTI_GET_IT(INT_Pin)!=RESET)  
    {  
        gs.sample_ready_flag=1;
    }
}

#if (0)

//��ĳһ֡����Ѱ�ҷ�ֵ
//���룺 ps      --> ��ȡ�Ĳ���ֵ����
//      chn     --> Ѱ���ͨ����
//      pindex  --> ��ֵ��indexָ��
//���أ�             �ڴ�֡�о��е���������
uint8_t Ges_Peak_Find(uint16_t *ps,uint8_t chn,uint16_t *pindex)
{
    uint16_t i,res,max,min,max_index;
    res=GES_NULL;
    
    //�����ֵ����Сֵ�����ֵ����Сֵλ��
    for (i=0,max=0,min=0xFFFF,max_index=0;i<gs.sample_size;i++)
    {
        if (ps[i]>max)
        {
            max=ps[i];
            max_index=i;
        }
        if (ps[i]<min)
        {
            min=ps[i];
//          min_index=i;
        }
    }
    
    //������Ч��ֵ
//  if ((max>gs.sample_up_limit[chn])&&(min<=gs.sample_down_limit[chn]))
    if (max>gs.sample_up_limit[chn])        
    {
        //��ֵ�����һ������
        if (max_index==(gs.sample_size-1))
        {
            res=GES_LEVEL1_APPROACH;
            *pindex=max_index;          
        }
        //��ֵ�ǵ�һ������
        else if (max_index==0)
        {
            res=GES_LEVEL1_LEAVE;
            *pindex=max_index;          
        }
        //��ֵ���м�
        else
        {
            res=GES_LEVEL1_FULL;
            *pindex=max_index;          
        }
//      if ((ps[0]<=gs.sample_down_limit[chn])&&(ps[GES_SAMPLE_ARRAY_NUM-1]>gs.sample_up_limit[chn]))
//      {
//          res=GES_LEVEL1_APPROACH;
//          *pindex=max_index;
//      }
//      else if ((ps[0]>gs.sample_up_limit[chn])&&(ps[GES_SAMPLE_ARRAY_NUM-1]<=gs.sample_down_limit[chn]))
//      {
//          res=GES_LEVEL1_LEAVE;
//          *pindex=max_index;
//      }
//      else if ((ps[0]<=gs.sample_down_limit[chn])&&(ps[GES_SAMPLE_ARRAY_NUM-1]<=gs.sample_down_limit[chn]))
//      {
//          res=GES_LEVEL1_FULL;
//          *pindex=max_index;
//      }
    }
    return res;
}
#endif

//��ĳһ֡����Ѱ�ұ���
//���룺 ps      --> ��ȡ�Ĳ���ֵ����
//      chn     --> Ѱ���ص�ͨ����
//      pindex  --> �����س�����ֵ��indexָ��
//���أ�             �ڴ�֡�о��е���������
uint8_t Ges_Edge_Find(uint16_t *ps,uint8_t chn,uint16_t *pindex)
{
    uint16_t i,res,rise,fall,rise_index,fall_index;
    res=GES_NULL;
    
    //�ӳ�ʼֵ��ʼѰ��������
    if (ps[0]<gs.sample_down_limit[chn])
    {
        for (i=0,rise=0;i<gs.sample_size;i++)
        {
            if (ps[i]>gs.sample_up_limit[chn])
            {
                rise=1;
                rise_index=i;
                break;
            }
        }
    }
    if (rise)
    {
        //���ϴ�Ѱ�ҵط���ʼ
        for (i=rise_index;i<gs.sample_size;i++)
        {
            if (ps[i]<gs.sample_down_limit[chn])
            {
                fall=1;
                fall_index=i;
                break;
            }
        }
    }
    
    //��ĩβֵ��ʼѰ���½���
    if ((!fall)&&(ps[gs.sample_size-i-1]<gs.sample_down_limit[chn]))
    {
        for (i=0;i<gs.sample_size;i++)
        {
            if (ps[gs.sample_size-i-1]>gs.sample_down_limit[chn])
            {
                fall=1;
                fall_index=i+1;
                break;
            }
        }
    }
    
    if (rise&&fall)
    {
        res=GES_LEVEL1_FULL;
        *pindex=rise_index;
    }
    else if (rise)
    {
        res=GES_LEVEL1_APPROACH;
        *pindex=rise_index;
    }
    else if (fall)
    {
        res=GES_LEVEL1_LEAVE;
        *pindex=fall_index;
    }
    return res;
}


//��ĳһ֡���в�������Ѱ��
//���룺 chn     --> Ѱ���ص�ͨ����
//
//���أ�             �ڴ�֡�о��е���������   ���ӽ�/ԭ��
uint8_t Ges_Trend_Find(uint8_t chn)
{
    uint8_t i,j,num_per_section;
    uint32_t total[SECTION_NUM];
    num_per_section = GES_SAMPLE_ARRAY_NUM_L/SECTION_NUM;

    memset(total,0,sizeof(total));
    for(i=0;i<SECTION_NUM;i++)
    {
        for(j=0;j<num_per_section;j++)
        {
            total[i] += gs.ges_sample_array[chn][j+i*num_per_section];
        }
    }

    if(total[0]>total[1] && total[1]>total[2] && total[2]>total[3] &&
        total[3]>40)
    {
        LOG_DEBUG("Ges_Trend_Find {chn[%d] leave}\r\n",chn);
        return GES_LEVEL1_LEAVE;
    }
    if(total[0]<total[1] && total[1]<total[2] && total[2]<total[3]) //&& total[0]>40)
    {
        LOG_DEBUG("Ges_Trend_Find {chn[%d] apporach}\r\n",chn);
        return GES_LEVEL1_APPROACH;
    }

    return GES_NULL;
}

//��ĳһ֡���в���Ѱ��
//���룺 chn     --> Ѱ���ص�ͨ����
//      pindex  --> ��ֵindexָ��
//���أ�             �ڴ�֡�о��е���������
uint8_t Ges_Wave_Search(uint8_t chn,uint16_t *pindex)
{
    uint16_t i,res;
//  uint32_t sum;
    uint8_t hold_flag=1;
    res=GES_NULL;
    
    //��ʼ��
    gs.wave[chn].fall_index=GES_INVALID_INDEX;
    gs.wave[chn].rise_index=GES_INVALID_INDEX;
    gs.wave[chn].peak_index=GES_INVALID_INDEX;
    
    //���ֵ�������ء��½��ؼ���ֵ�������ء��½���λ�ú�ƽ��ֵ
    for (i=0,gs.wave[chn].peak_value=0;i<gs.sample_size;i++)
    {
        if ((i>=(gs.sample_size-SAMPLE_HOLD_END_SHIFT))&&(gs.ges_sample_array[chn][i]<SAMPLE_HOLD_LIMIT_DEF))
            hold_flag=0;

        if (gs.ges_sample_array[chn][i]>gs.wave[chn].peak_value)
        {
            gs.wave[chn].peak_value=gs.ges_sample_array[chn][i];
            gs.wave[chn].peak_index=i;
        }
        if (i<gs.sample_size-1)
        {
            if ((gs.ges_sample_array[chn][i]<gs.sample_down_limit[chn])&&
                (gs.ges_sample_array[chn][i+1]>=gs.sample_down_limit[chn]))
            {
                gs.wave[chn].rise_value=gs.ges_sample_array[chn][i+1];
                gs.wave[chn].rise_index=i+1;
            }
            if ((gs.ges_sample_array[chn][i]>=gs.sample_down_limit[chn])&&
                (gs.ges_sample_array[chn][i+1]<gs.sample_down_limit[chn]))
            {
                gs.wave[chn].fall_value=gs.ges_sample_array[chn][i];
                gs.wave[chn].fall_index=i;
            }
        }
    }
    if ((gs.sample_size==GES_SAMPLE_ARRAY_NUM_L)&&(hold_flag==1))
    {
        LOG_DEBUG("after a hold Ges.\r\n");
        res = Ges_Trend_Find(chn);
        
        if(GES_NULL==res)
            res=GES_LEVEL1_HOLD;
/*
        if (gs.wave[chn].peak_index==(gs.sample_size-1) &&
            gs.ges_sample_array[chn][gs.sample_size-1]-gs.ges_sample_array[chn][0]>200)
        {
            LOG_DEBUG("hold {apporach}\r\n");
            res=GES_LEVEL1_APPROACH;
        }
        else if (gs.wave[chn].peak_index==0 &&
            gs.ges_sample_array[chn][0]-gs.ges_sample_array[chn][gs.sample_size-1]>200)
        {
            LOG_DEBUG("hold {leave}\r\n");
            res=GES_LEVEL1_LEAVE;
        }
*/
        *pindex=GES_SAMPLE_ARRAY_NUM_S-1;
        return res;
    }
//  if ((gs.wave[chn].rise_index!=GES_INVALID_INDEX)&&(gs.wave[chn].fall_index!=GES_INVALID_INDEX))
//  {
//      res=GES_LEVEL1_FULL;
//  }
//  else if (gs.wave[chn].rise_index!=GES_INVALID_INDEX)
//  {
//      res=GES_LEVEL1_APPROACH;
//  }
//  else if (gs.wave[chn].fall_index!=GES_INVALID_INDEX)
//  {
//      res=GES_LEVEL1_LEAVE;
//  }
    if ((gs.wave[chn].peak_index!=GES_INVALID_INDEX)&&(gs.wave[chn].peak_value>gs.sample_up_limit[chn]))
    {

        if(GES_SAMPLE_ARRAY_NUM_L == gs.sample_size)
        {
            res = Ges_Trend_Find(chn);

            if(GES_NULL==res)
                res=GES_LEVEL1_FULL;
        }
        else
        {
            //��ֵ�����һ������
            if (gs.wave[chn].peak_index==(gs.sample_size-1))
            {
                res=GES_LEVEL1_APPROACH;
                //*pindex=gs.wave[chn].peak_index;
            }
            //��ֵ�ǵ�һ������
            else if (gs.wave[chn].peak_index==0)
            {
                res=GES_LEVEL1_LEAVE;
                //*pindex=gs.wave[chn].peak_index;
            }
            //��ֵ���м�
            else
            {
                res=GES_LEVEL1_FULL;
                //*pindex=gs.wave[chn].peak_index;
            }
        }
        *pindex=gs.wave[chn].peak_index;
    }
    return res;
}

//�ж��������η������Ⱥ��ϵ
//���룺 chn1    --> ��һ����
//      chn2    --> �ڶ�����
//���أ� chn1    --> ��һ��������
//      chn2    --> �ڶ���������
//      0xFF    --> �޷��ж�
uint8_t Ges_Wave_Lead(uint8_t chn1,uint8_t chn2)
{
    if ((gs.wave[chn1].peak_index!=GES_INVALID_INDEX)&&
        (gs.wave[chn2].peak_index!=GES_INVALID_INDEX))
    {
        if (gs.wave[chn1].peak_index<gs.wave[chn2].peak_index)
            return chn1;
        else if (gs.wave[chn1].peak_index>gs.wave[chn2].peak_index)
            return chn2;
    }
    if ((gs.wave[chn1].fall_index!=GES_INVALID_INDEX)&&
        (gs.wave[chn2].fall_index!=GES_INVALID_INDEX))
    {
        if (gs.wave[chn1].fall_index<gs.wave[chn2].fall_index)
            return chn1;
        else if (gs.wave[chn1].fall_index>gs.wave[chn2].fall_index)
            return chn2;
    }
    if ((gs.wave[chn1].rise_index!=GES_INVALID_INDEX)&&
        (gs.wave[chn2].rise_index!=GES_INVALID_INDEX))
    {
        if (gs.wave[chn1].rise_index<gs.wave[chn2].rise_index)
            return chn1;
        else if (gs.wave[chn1].rise_index>gs.wave[chn2].rise_index)
            return chn2;
    }
    if ((gs.wave[chn1].rise_index!=GES_INVALID_INDEX)&&
        ((gs.wave[chn2].fall_index!=GES_INVALID_INDEX)||
        (gs.wave[chn2].peak_index!=GES_INVALID_INDEX)))
        return chn2;
    
    if ((gs.wave[chn2].rise_index!=GES_INVALID_INDEX)&&
        ((gs.wave[chn1].fall_index!=GES_INVALID_INDEX)||
        (gs.wave[chn1].peak_index!=GES_INVALID_INDEX)))
        return chn1;
    
    return GES_CHN_INVALID;
}

//��ĳһ֡�������Ʒ���
//���أ�         �ڴ�֡�о��е���������
uint8_t Ges_Analysis(void)
{
    uint8_t i,basic_ges,res,chn,hold_flag;
    uint16_t index;
    
    res=GES_NULL;
    chn=0;
    hold_flag=0;

#if(!CALIB_OBO)
    //��׼����������
    Ges_Normalize();
#endif
    for (i=0;i<GES_CHN_NUM;i++)
    {
//      basic_ges=Ges_Peak_Find(gs.ges_sample_array[i],i,&index);
//      basic_ges=Ges_Edge_Find(gs.ges_sample_array[i],i,&index);
        basic_ges=Ges_Wave_Search(i,&index);
        if (basic_ges==GES_LEVEL1_APPROACH)
        {
            gs.ges_record[i].ges=basic_ges;
            //gs.ges_record[i].add_time=gs.frame_start_tick+(index*SAMPLE_FREQUENCY_DEF*GES_SAMPLE_AVE_NUM);
            chn|=(1<<i);
//          res=1;
#if (LOG_ENABLE)
            LOG_DEBUG("tick: %lld, CHN%d ges: approach!\r\n",gs.ges_record[i].add_time,i);
#endif
        }
        else if (basic_ges==GES_LEVEL1_LEAVE)
        {
            gs.ges_record[i].ges=basic_ges;
            //gs.ges_record[i].add_time=gs.frame_start_tick+(index*SAMPLE_FREQUENCY_DEF*GES_SAMPLE_AVE_NUM);
            chn|=(1<<i);
//          res=1;
#if (LOG_ENABLE)
            LOG_DEBUG("tick: %lld, CHN%d ges: leave!\r\n",gs.ges_record[i].add_time,i);
#endif
        }
        else if (basic_ges==GES_LEVEL1_FULL)
        {
            gs.ges_record[i].ges=basic_ges;
            //gs.ges_record[i].add_time=gs.frame_start_tick+(index*SAMPLE_FREQUENCY_DEF*GES_SAMPLE_AVE_NUM);
            chn|=(1<<i);
//          res=1;
#if (LOG_ENABLE)
            LOG_DEBUG("tick: %lld, CHN%d ges: full!\r\n",gs.ges_record[i].add_time,i);
#endif
        }
        else if (basic_ges==GES_LEVEL1_HOLD)
        {
            gs.ges_record[i].ges=basic_ges;
            //gs.ges_record[i].add_time=gs.frame_start_tick+(index*SAMPLE_FREQUENCY_DEF*GES_SAMPLE_AVE_NUM);
            chn|=(1<<i);
#if (LOG_ENABLE)
            LOG_DEBUG("tick: %lld, CHN%d ges: hold!\r\n",gs.ges_record[i].add_time,i);
#endif          
            hold_flag++;
//          res=GES_LEVEL2_HOLD;
//          return res;
        }
    }
    
    //���µ�����
    if (chn)
    {
        if (hold_flag)
        {
            res=GES_LEVEL2_HOLD;
            return res;
        }

        if ((chn&(1<<GES_CHN_DOWN_LEFT))&&(chn&(1<<GES_CHN_DOWN_RIGHT)))
        {
            if(gs.ges_record[GES_CHN_DOWN_LEFT].ges==GES_LEVEL1_APPROACH &&
                gs.ges_record[GES_CHN_DOWN_RIGHT].ges==GES_LEVEL1_APPROACH)
            {
                res=GES_LEVEL2_APPROACH;
                return res;
            }
            else if(gs.ges_record[GES_CHN_DOWN_LEFT].ges==GES_LEVEL1_LEAVE &&
                gs.ges_record[GES_CHN_DOWN_RIGHT].ges==GES_LEVEL1_LEAVE)
            {
                res=GES_LEVEL2_LEAVE;
                return res;
            }
        }

        if (chn&(1<<GES_CHN_UP))
            //����last_ges_record
            gs.last_ges_record[GES_CHN_UP]=gs.ges_record[GES_CHN_UP];
        if (chn&(1<<GES_CHN_DOWN_LEFT))
            //����last_ges_record
            gs.last_ges_record[GES_CHN_DOWN_LEFT]=gs.ges_record[GES_CHN_DOWN_LEFT];
        if (chn&(1<<GES_CHN_DOWN_RIGHT))
            //����last_ges_record
            gs.last_ges_record[GES_CHN_DOWN_RIGHT]=gs.ges_record[GES_CHN_DOWN_RIGHT];
        
        uint8_t left_up,left_right,right_up;
        //left_up=Ges_Wave_Lead(GES_CHN_DOWN_LEFT,GES_CHN_UP);
        left_right=Ges_Wave_Lead(GES_CHN_DOWN_LEFT,GES_CHN_DOWN_RIGHT);
        //right_up=Ges_Wave_Lead(GES_CHN_DOWN_RIGHT,GES_CHN_UP);
#if 0
        //������
        if ((left_up==GES_CHN_DOWN_LEFT)&&(right_up==GES_CHN_UP))
        {
            res=GES_LEVEL2_RIGHT;
            return res;
        }
        //���ҵ���
        if ((left_up==GES_CHN_UP)&&(right_up==GES_CHN_DOWN_RIGHT))
        {
            res=GES_LEVEL2_LEFT;
            return res;
        }
        //���ϵ���
        if ((left_up==GES_CHN_UP)||(right_up==GES_CHN_UP))
        {
            res=GES_LEVEL2_DOWN;
            return res;
        }
        //���µ���
        if ((left_up==GES_CHN_DOWN_LEFT)||(right_up==GES_CHN_DOWN_RIGHT))
        {
            res=GES_LEVEL2_UP;
            return res;
        }
#endif
        //������
        if (left_right==GES_CHN_DOWN_LEFT)
        {
            res=GES_LEVEL2_RIGHT;
            return res;
        }
        //���ҵ���
        if (left_right==GES_CHN_DOWN_RIGHT)
        {
            res=GES_LEVEL2_LEFT;
            return res;
        }

#if (0)     
        //�˴���UPλ��
        if (chn&(1<<GES_CHN_UP))
        {
            //������
            if ((gs.last_ges_record[GES_CHN_DOWN_LEFT].add_time<gs.ges_record[GES_CHN_UP].add_time)&&
                (gs.last_ges_record[GES_CHN_DOWN_RIGHT].add_time>gs.ges_record[GES_CHN_UP].add_time)&&
                (gs.last_ges_record[GES_CHN_DOWN_LEFT].add_time+GES_TIME_DIF_MAX>gs.ges_record[GES_CHN_UP].add_time)&&
                (gs.last_ges_record[GES_CHN_DOWN_RIGHT].add_time<gs.ges_record[GES_CHN_UP].add_time+GES_TIME_DIF_MAX))
            {
                res=GES_LEVEL2_RIGHT;
                return res;
            }
            //���ҵ���
            if ((gs.last_ges_record[GES_CHN_DOWN_RIGHT].add_time<gs.ges_record[GES_CHN_UP].add_time)&&
                (gs.last_ges_record[GES_CHN_DOWN_LEFT].add_time>gs.ges_record[GES_CHN_UP].add_time)&&
                (gs.last_ges_record[GES_CHN_DOWN_RIGHT].add_time+GES_TIME_DIF_MAX>gs.ges_record[GES_CHN_UP].add_time)&&
                (gs.last_ges_record[GES_CHN_DOWN_LEFT].add_time<gs.ges_record[GES_CHN_UP].add_time+GES_TIME_DIF_MAX))
            {
                res=GES_LEVEL2_LEFT;
                return res;
            }
            //���ϵ���
            if (((gs.last_ges_record[GES_CHN_DOWN_LEFT].add_time<gs.ges_record[GES_CHN_UP].add_time)&&
                (gs.last_ges_record[GES_CHN_DOWN_LEFT].add_time+GES_TIME_DIF_MAX>gs.ges_record[GES_CHN_UP].add_time))||
                ((gs.last_ges_record[GES_CHN_DOWN_RIGHT].add_time<gs.ges_record[GES_CHN_UP].add_time)&&
                (gs.last_ges_record[GES_CHN_DOWN_RIGHT].add_time+GES_TIME_DIF_MAX>gs.ges_record[GES_CHN_UP].add_time)))
            {
                res=GES_LEVEL2_UP;
                return res;
            }
            //���µ���
            if (((gs.last_ges_record[GES_CHN_DOWN_LEFT].add_time>gs.ges_record[GES_CHN_UP].add_time)&&
                (gs.last_ges_record[GES_CHN_DOWN_LEFT].add_time<gs.ges_record[GES_CHN_UP].add_time+GES_TIME_DIF_MAX))||
                ((gs.last_ges_record[GES_CHN_DOWN_RIGHT].add_time>gs.ges_record[GES_CHN_UP].add_time)&&
                (gs.last_ges_record[GES_CHN_DOWN_RIGHT].add_time<gs.ges_record[GES_CHN_UP].add_time+GES_TIME_DIF_MAX)))
            {
                res=GES_LEVEL2_DOWN;
                return res;
            }
        }
        //�˴���DOWN_LEFTλ��
        if (chn&(1<<GES_CHN_DOWN_LEFT))
        {
            //������
            if ((gs.last_ges_record[GES_CHN_UP].add_time>gs.ges_record[GES_CHN_DOWN_LEFT].add_time)&&
                (gs.last_ges_record[GES_CHN_DOWN_RIGHT].add_time>gs.ges_record[GES_CHN_DOWN_LEFT].add_time)&&
                (gs.last_ges_record[GES_CHN_UP].add_time<gs.ges_record[GES_CHN_UP].add_time+GES_TIME_DIF_MAX)&&
                (gs.last_ges_record[GES_CHN_DOWN_RIGHT].add_time<gs.ges_record[GES_CHN_DOWN_LEFT].add_time+GES_TIME_DIF_MAX))
            {
                res=GES_LEVEL2_RIGHT;
                return res;
            }
            //���ҵ���
            if ((gs.last_ges_record[GES_CHN_UP].add_time<gs.ges_record[GES_CHN_DOWN_LEFT].add_time)&&
                (gs.last_ges_record[GES_CHN_DOWN_RIGHT].add_time<gs.ges_record[GES_CHN_DOWN_LEFT].add_time)&&
                (gs.last_ges_record[GES_CHN_UP].add_time+GES_TIME_DIF_MAX>gs.ges_record[GES_CHN_DOWN_LEFT].add_time)&&
                (gs.last_ges_record[GES_CHN_DOWN_RIGHT].add_time+GES_TIME_DIF_MAX>gs.ges_record[GES_CHN_DOWN_LEFT].add_time))
            {
                res=GES_LEVEL2_LEFT;
                return res;
            }
            //���ϵ���
            if ((gs.last_ges_record[GES_CHN_UP].add_time<gs.ges_record[GES_CHN_DOWN_LEFT].add_time)&&
                (gs.last_ges_record[GES_CHN_UP].add_time+GES_TIME_DIF_MAX>gs.ges_record[GES_CHN_DOWN_LEFT].add_time))
            {
                res=GES_LEVEL2_UP;
                return res;
            }
            //���µ���
            if ((gs.last_ges_record[GES_CHN_UP].add_time>gs.ges_record[GES_CHN_DOWN_LEFT].add_time)&&
                (gs.last_ges_record[GES_CHN_UP].add_time<gs.ges_record[GES_CHN_DOWN_LEFT].add_time+GES_TIME_DIF_MAX))
            {
                res=GES_LEVEL2_DOWN;
                return res;
            }
        }
        //�˴���DOWN_RIGHTλ��
        if (chn&(1<<GES_CHN_DOWN_RIGHT))
        {
            //������
            if ((gs.last_ges_record[GES_CHN_UP].add_time<gs.ges_record[GES_CHN_DOWN_RIGHT].add_time)&&
                (gs.last_ges_record[GES_CHN_DOWN_LEFT].add_time<gs.ges_record[GES_CHN_DOWN_RIGHT].add_time)&&
                (gs.last_ges_record[GES_CHN_UP].add_time+GES_TIME_DIF_MAX>gs.ges_record[GES_CHN_DOWN_RIGHT].add_time)&&
                (gs.last_ges_record[GES_CHN_DOWN_LEFT].add_time+GES_TIME_DIF_MAX>gs.ges_record[GES_CHN_DOWN_RIGHT].add_time))
            {
                res=GES_LEVEL2_RIGHT;
                return res;
            }
            //���ҵ���
            if ((gs.last_ges_record[GES_CHN_UP].add_time>gs.ges_record[GES_CHN_DOWN_RIGHT].add_time)&&
                (gs.last_ges_record[GES_CHN_DOWN_LEFT].add_time>gs.ges_record[GES_CHN_DOWN_RIGHT].add_time)&&
                (gs.last_ges_record[GES_CHN_UP].add_time<gs.ges_record[GES_CHN_DOWN_RIGHT].add_time+GES_TIME_DIF_MAX)&&
                (gs.last_ges_record[GES_CHN_DOWN_LEFT].add_time<gs.ges_record[GES_CHN_DOWN_RIGHT].add_time+GES_TIME_DIF_MAX))
            {
                res=GES_LEVEL2_LEFT;
                return res;
            }
            //���ϵ���
            if ((gs.last_ges_record[GES_CHN_UP].add_time<gs.ges_record[GES_CHN_DOWN_RIGHT].add_time)&&
                (gs.last_ges_record[GES_CHN_UP].add_time+GES_TIME_DIF_MAX>gs.ges_record[GES_CHN_DOWN_RIGHT].add_time))
            {
                res=GES_LEVEL2_UP;
                return res;
            }
            //���µ���
            if ((gs.last_ges_record[GES_CHN_UP].add_time>gs.ges_record[GES_CHN_DOWN_RIGHT].add_time)&&
                (gs.last_ges_record[GES_CHN_UP].add_time<gs.ges_record[GES_CHN_DOWN_RIGHT].add_time+GES_TIME_DIF_MAX))
            {
                res=GES_LEVEL2_DOWN;
                return res;
            }
        }
#endif
    }
    
    return res;
}
#if(!CALIB_OBO)
//���ƻ�׼���꣬����ƽ��ֵ����
void Ges_Calib(uint8_t firstboot)
{
    uint8_t i;
    uint16_t j,delta;
    uint32_t sum;
#if 1
    if(gs.sample_size == GES_SAMPLE_ARRAY_NUM_S)
    {
        for (i=0;i<GES_CHN_NUM;i++)
        {
            sum=0;
            for (j=0;j<GES_SAMPLE_ARRAY_NUM_S;j++)
            {
                sum+=gs.ges_sample_array[i][j];
            }
            delta=(uint16_t)(sum/gs.sample_size);
            //delta=(uint16_t)(sum>>3);

            if(firstboot)
                gs.sample_base_last[i]=delta;
            else
                gs.sample_base_last[i] = gs.sample_base[i];

            gs.sample_base[i]=delta;
            //gs.sample_down_limit[i]=SAMPLE_DOWN_LIMIT_DEF+delta;
            //gs.sample_up_limit[i]=SAMPLE_UP_LIMIT_DEF+delta;
#if (LOG_ENABLE)
            //LOG_DEBUG("CHN %d-> base: %d,down_limit: %d,up limit: %d\r\n",i,gs.sample_base[i],gs.sample_down_limit[i],gs.sample_up_limit[i]);
            //LOG_DEBUG("CHN %d-> base: %d %d\r\n",i,gs.sample_base_last[i],gs.sample_base[i]);
#endif
        }
    }
#else
    for (i=0;i<GES_CHN_NUM;i++)
    {
        sum=0;
        for (j=0;j<gs.sample_size;j++)
        {
            sum+=gs.ges_sample_array[i][j];
        }
        if(gs.sample_size == GES_SAMPLE_ARRAY_NUM_L)
        {
            delta=(uint16_t)(sum>>5);
            //delta=(uint16_t)(sum>>5)+(((sum%GES_SAMPLE_ARRAY_NUM_L)>=(GES_SAMPLE_ARRAY_NUM_L/2))?1:0);
        }
        else
        {
            delta=(uint16_t)(sum>>3);
            //delta=(uint16_t)(sum>>3)+(((sum%GES_SAMPLE_ARRAY_NUM_S)>=(GES_SAMPLE_ARRAY_NUM_S/2))?1:0);
        }

        if(firstboot)
            gs.sample_base_last[i]=delta;
        else
            gs.sample_base_last[i] = gs.sample_base[i];
        
        gs.sample_base[i]=delta;

#if (LOG_ENABLE)
        //LOG_DEBUG("CHN %d-> base: %d,down_limit: %d,up limit: %d\r\n",i,gs.sample_base[i],gs.sample_down_limit[i],gs.sample_up_limit[i]);
        //LOG_DEBUG("CHN %d-> base: %d %d\r\n",i,gs.sample_base_last[i],gs.sample_base[i]);
#endif
    }
#endif
}

//��һ����������
void Ges_Normalize(void)
{
    uint8_t i,j;
    for (i=0;i<GES_CHN_NUM;i++)
    {
        for (j=0;j<gs.sample_size;j++)
        {
        
            if (gs.ges_sample_array[i][j]>=gs.sample_base_last[i]) {
                gs.ges_sample_array[i][j]=gs.ges_sample_array[i][j]-gs.sample_base_last[i];
            }
            else {
                gs.ges_sample_array[i][j]=0;
                //gs.ges_sample_array[i][j]=gs.sample_base_last[i]-gs.ges_sample_array[i][j];
            }
        }
    }

#if (LOG_ENABLE)
    for (j=0;j<gs.sample_size;j++)
    {
        LOG_DEBUG(" %5d %5d %5d\r\n",
            gs.ges_sample_array[1][j],gs.ges_sample_array[2][j],gs.ges_sample_array[0][j]);
    }
    LOG_DEBUG("[%5d %5d %5d]sample[%d]\r\n",gs.sample_base_last[1],gs.sample_base_last[2],gs.sample_base_last[0],gs.sample_size);
#endif
}
#endif

#if(CALIB_OBO)
//��һ����������
void Ges_Normalize_OneBOne(uint16_t *ps)
{
    uint8_t i;

#if (LOG_ENABLE)
    LOG_DEBUG("%5d  %5d  %5d  ", ps[1],ps[2],ps[0]);
#endif

    for (i=0;i<GES_CHN_NUM;i++)
    {
        if (ps[i]>=gs.sample_base_last[i]) {
            ps[i]=ps[i]-gs.sample_base_last[i];
        }
        else {
            ps[i]=0;
            //ps[i]=gs.sample_base_last[i]-ps[i];
        }
    }

#if (LOG_ENABLE)
    LOG_DEBUG(" %5d %5d %5d base:[%5d %5d %5d]\r\n",
                    ps[1],ps[2],ps[0],
                    gs.sample_base_last[1],gs.sample_base_last[2],gs.sample_base_last[0]);
                    //ps[1]<=gs.sample_base[1]?gs.sample_base[1]-ps[1]:ps[1]-gs.sample_base[1],
                    //ps[2]<=gs.sample_base[2]?gs.sample_base[2]-ps[2]:ps[2]-gs.sample_base[2],
                    //ps[0]<=gs.sample_base[0]?gs.sample_base[0]-ps[0]:ps[0]-gs.sample_base[0]);
#endif
}

//���ƻ�׼���꣬����ʵʱƽ��ֵ
uint8_t Ges_CalibOneBOne(uint16_t *ps)
{
    uint8_t i;
    uint16_t j,delta;
    uint32_t sum;
    uint16_t temp;
    static uint8_t arrayIndex = 0;

    if(gs.hold_counter!=0)
        goto NORMALIZE;

    if(arrayIndex==GES_SAMPLE_ARRAY_FULL)
    {
        for (i=0;i<GES_CHN_NUM;i++)
        {
            for (j=0;j<GES_SAMPLE_ARRAY_NUM_L-1;j++)
            {
                gs.ges_sample_Calib_array[i][j] = gs.ges_sample_Calib_array[i][j+1];
            }
            gs.ges_sample_Calib_array[i][j] = ps[i];
        }
    } else {
        for (i=0;i<GES_CHN_NUM;i++)
        {
            gs.ges_sample_Calib_array[i][arrayIndex] = ps[i];
        }
        arrayIndex++;
        //return 1;
    }

    for (i=0;i<GES_CHN_NUM;i++)
    {
        sum=0;
        for (j=0;j<GES_SAMPLE_ARRAY_NUM_L;j++)
        {
            //LOG_DEBUG("CHN %d-> base: [%d] %d\r\n",i,j,gs.ges_sample_Calib_array[i][j]);
            sum+=gs.ges_sample_Calib_array[i][j];
        }

        delta=(uint16_t)(sum>>5);

        gs.sample_base_last[i]=delta;
        gs.sample_base[i]=delta;
#if (LOG_ENABLE)
        //LOG_DEBUG("CHN %d-> base: %d,down_limit: %d,up limit: %d\r\n",i,gs.sample_base[i],gs.sample_down_limit[i],gs.sample_up_limit[i]);
        //LOG_DEBUG("CHN %d-> base: %d %d\r\n",i,gs.sample_base_last[i],gs.sample_base[i]);
#endif
    }

NORMALIZE:
    Ges_Normalize_OneBOne(ps);

    return 0;
}
#endif


////�ж��Ƿ���Ҫ�ı�sample_size���ı�
////����ֵ��    0-->    ����Ҫ�ı�
////            1-->    ��Ҫ�ı�
//uint8_t Ges_SampleSize_Switch(void)
//{
//  uint16_t i;
//  for (i=0;i<gs.sample_size;i++)
//  {
//      
//  }
//}

//��ӡ���������Ϣ
void Ges_Log(void)
{
#if (LOG_ENABLE)
    LOG_DEBUG("this ges-->up: %lld/%d, down_left: %lld/%d, down_right: %lld/%d!\r\n",
                gs.ges_record[GES_CHN_UP].add_time,gs.ges_record[GES_CHN_UP].ges,
                gs.ges_record[GES_CHN_DOWN_LEFT].add_time,gs.ges_record[GES_CHN_DOWN_LEFT].ges,
                gs.ges_record[GES_CHN_DOWN_RIGHT].add_time,gs.ges_record[GES_CHN_DOWN_RIGHT].ges);

    LOG_DEBUG("last ges-->up: %d.%d/%d, down_left: %d.%d/%d, down_right: %d.%d/%d!\r\n",
                (uint32_t)(gs.last_ges_record[GES_CHN_UP].add_time/1000),(uint32_t)(gs.last_ges_record[GES_CHN_UP].add_time%1000),gs.last_ges_record[GES_CHN_UP].ges,
                (uint32_t)(gs.last_ges_record[GES_CHN_DOWN_LEFT].add_time/1000),(uint32_t)(gs.last_ges_record[GES_CHN_DOWN_LEFT].add_time%1000),gs.last_ges_record[GES_CHN_DOWN_LEFT].ges,
                (uint32_t)(gs.last_ges_record[GES_CHN_DOWN_RIGHT].add_time/1000),(uint32_t)(gs.last_ges_record[GES_CHN_DOWN_RIGHT].add_time%1000),gs.last_ges_record[GES_CHN_DOWN_RIGHT].ges);  
    LOG_DEBUG("CHN DOWN LEFT  rise_index: %d,peak_index: %d,fall_index: %d\r\n",gs.wave[GES_CHN_DOWN_LEFT].rise_index,gs.wave[GES_CHN_DOWN_LEFT].peak_index,gs.wave[GES_CHN_DOWN_LEFT].fall_index);
    LOG_DEBUG("CHN UP         rise_index: %d,peak_index: %d,fall_index: %d\r\n",gs.wave[GES_CHN_UP].rise_index,gs.wave[GES_CHN_UP].peak_index,gs.wave[GES_CHN_UP].fall_index);
    LOG_DEBUG("CHN DOWN RIGHT rise_index: %d,peak_index: %d,fall_index: %d\r\n",gs.wave[GES_CHN_DOWN_RIGHT].rise_index,gs.wave[GES_CHN_DOWN_RIGHT].peak_index,gs.wave[GES_CHN_DOWN_RIGHT].fall_index);
    LOG_DEBUG("\n");

    uint8_t i,j;
    for (i=0;i<GES_CHN_NUM;i++)
    {
//          LOG_DEBUG("CHN%d raw array: ",i);
//          for (j=0;j<GES_SAMPLE_RAW_ARRAY_NUM;j++)
//          {
//              LOG_DEBUG("%d ",gs.ges_sample_raw_array[i][j]);
//          }
//          LOG_DEBUG("\n");
        if (i==GES_CHN_UP)
            LOG_DEBUG("UP    array:");
        else if (i==GES_CHN_DOWN_LEFT)
            LOG_DEBUG("LEFT  array:");
        else if (i==GES_CHN_DOWN_RIGHT)
            LOG_DEBUG("RIGHT array:");
        for (j=0;j<gs.sample_size;j++)
        {
            //��ӡȥ����׼ֵ�Ĺ�һ������
            if (gs.ges_sample_array[i][j]>=gs.sample_down_limit[i])
                LOG_DEBUG("%d ",gs.ges_sample_array[i][j]-gs.sample_down_limit[i]);
            else
                LOG_DEBUG("%d ",0);
//              LOG_DEBUG("%d ",gs.ges_sample_array[i][j]);
        }
        LOG_DEBUG("\r\n");
    }
#endif  
}

//������
void App_Task(void)
{
    //static uint8_t calib_num=0;

    static uint8_t firstboot=1;
    /*
    if (gs.sample_start_flag==1)
    {
        gs.sample_start_flag=0;
        VCNL4035_StartGesDetect();
//      LOG_DEBUG("ges detect!\n");
    }*/
    //if (gs.sample_ready_flag==1)
    {
        uint16_t ps[3];
        uint8_t res;

        memset(ps,0,sizeof(ps));
        gs.sample_ready_flag=0;
        //VCNL4035_ReadPS(ps);
        //VCNL4035_ClearInt();
        getSensorDataByHostout(ps);

        if(!ps[0]&&!ps[1]&&!ps[2])
        {
            LOG_DEBUG(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\r\n");
            return;
        }
#if(CALIB_OBO)
        Ges_CalibOneBOne(ps);
#endif
        res=Ges_Add_Sample(ps);
        //һ������������
        if (res==1)
        {
//#if (LOG_ENABLE)
#if(CALIB_OBO)
            LOG_DEBUG("sample[%d]\r\n",gs.sample_size);
#endif
//#endif
            //��һ�β���������Ϊ������
            //if (!calib_num)
            //if(GES_SAMPLE_ARRAY_NUM_S==gs.sample_size)
#if(!CALIB_OBO)
            if(0==gs.hold_counter)
            {
                Ges_Calib(firstboot);
                firstboot = 0;
                //calib_num++;
                //return;
            }
#endif
#if (!LOG_ENABLE)
            LOG_DEBUG("sample[%d]\r\n",gs.sample_size);
#endif
            res=Ges_Analysis();
            if (res)//else return to 10 samples
            {
                //��ǰ��short�������ı�sample_size��������
                if (gs.sample_size==GES_SAMPLE_ARRAY_NUM_S)
                {
                    gs.sample_size=GES_SAMPLE_ARRAY_NUM_L;
                    gs.sample_size_raw=GES_SAMPLE_RAW_ARRAY_NUM_L;
                    gs.ges_sample_raw_cur=GES_SAMPLE_ARRAY_NUM_S;
                    return;
                }

                uint64_t tick=Systick_Get();
                
                //�ϴ���HOLD����,�˴β���HOLD�Ҳ�Ϊ�գ������󴥷�,����hold֮��Ҫ����leave��approach
                if ((gs.last_ges==GES_LEVEL2_HOLD)&&(res!=GES_LEVEL2_HOLD)&&
                    (res!=GES_LEVEL2_APPROACH)&&(res!=GES_LEVEL2_LEAVE))
                {
                    LOG_DEBUG("######IGNORE [%x] after GES_LEVEL2_HOLD#######\r\n",res);
                    gs.last_ges=GES_NULL;
                    gs.last_ges_time=tick;
                    return;
                }

                //�ϴ���APPROACH or LEAVE����,�˴���LEFT or RIGHT�������󴥷�
                if (((gs.last_ges==GES_LEVEL2_APPROACH) || (gs.last_ges==GES_LEVEL2_LEAVE)) && 
                    ((res==GES_LEVEL2_LEFT)||(res==GES_LEVEL2_RIGHT)))
                {
                    LOG_DEBUG("######IGNORE [%x] after [%s]#######\r\n",res,(gs.last_ges==GES_LEVEL2_APPROACH)?"APPROACH":"LEAVE");
                    gs.last_ges=GES_NULL;
                    gs.last_ges_time=tick;
                    return;
                }
                    

                //��Ч���Ƹ���
                if (tick>gs.last_ges_time+GES_DISTANCE_MIN)
                {
                    //if(!gs.hold_saveself)
                        gs.last_ges=res;
                    gs.last_ges_time=tick;
                }
                else//��Ч���ƣ��������
                {
                    //���������ӽ������뿪����
                    if((gs.last_ges == GES_LEVEL2_LEAVE && res == GES_LEVEL2_LEAVE) ||
                        (gs.last_ges == GES_LEVEL2_APPROACH && res == GES_LEVEL2_APPROACH))
                    {
                        gs.last_ges=res;
                        gs.last_ges_time=tick;
                    }
                    else
                    {
                        if(!(gs.last_ges == GES_LEVEL2_HOLD && res == GES_LEVEL2_LEAVE))
                        {
                            LOG_DEBUG("tick time lease %d ms\r\n",GES_DISTANCE_MIN);
                            return;
                        }
                        else
                        {
                            gs.last_ges=res;
                            gs.last_ges_time=tick;
                        }
                    }
                }

                if (res==GES_LEVEL2_UP)
                {
                    HAL_GPIO_WritePin(LED_UP_GPIO_Port, LED_UP_Pin, GPIO_PIN_RESET);
                    led_counter=LED_FLASH_DELAY;
                    LOG_DEBUG("                         Ges: UP!\r\n");
                    Ges_Log();
                    //��SW2
                    //HAL_GPIO_WritePin(SW2_GPIO_Port, SW2_Pin, GPIO_PIN_SET);
                }
                else if (res==GES_LEVEL2_DOWN)
                {
                    HAL_GPIO_WritePin(LED_DOWN_GPIO_Port, LED_DOWN_Pin, GPIO_PIN_RESET);
                    led_counter=LED_FLASH_DELAY;
                    LOG_DEBUG("                         Ges: DOWN!\r\n");
                    Ges_Log();
                    //��SW2
                    //HAL_GPIO_WritePin(SW2_GPIO_Port, SW2_Pin, GPIO_PIN_RESET);
                }
                else if (res==GES_LEVEL2_LEFT)
                {
                    HAL_GPIO_WritePin(LED_LEFT_GPIO_Port, LED_LEFT_Pin, GPIO_PIN_RESET);
                    led_counter=LED_FLASH_DELAY;
                    LOG_DEBUG("                         Ges: LEFT!\r\n");
                    Ges_Log();
                    //��SW1
                    //HAL_GPIO_WritePin(SW1_GPIO_Port, SW1_Pin, GPIO_PIN_RESET);
                }
                else if (res==GES_LEVEL2_RIGHT)
                {
                    HAL_GPIO_WritePin(LED_RIGHT_GPIO_Port, LED_RIGHT_Pin, GPIO_PIN_RESET);
                    led_counter=LED_FLASH_DELAY;
                    LOG_DEBUG("                         Ges: RIGHT!\r\n");
                    Ges_Log();
                    //��SW1
                    //HAL_GPIO_WritePin(SW1_GPIO_Port, SW1_Pin, GPIO_PIN_SET);
                }
                else if (res==GES_LEVEL2_HOLD)
                {
                    static uint8_t sw_state=0;
                    
//                  gs.hold_counter++;
//                  if (gs.hold_counter==0)
//                  {
//                      HAL_GPIO_WritePin(LED_HOLD_GPIO_Port, LED_HOLD_Pin, GPIO_PIN_SET);
//                      led_counter=LED_FLASH_DELAY;
//                      LOG_DEBUG("Ges: HOLD!\n");
//                      Ges_Log();
//                      if (!sw_state)
//                      {
//                          HAL_GPIO_WritePin(SW1_GPIO_Port, SW1_Pin, GPIO_PIN_SET);
//                          HAL_GPIO_WritePin(SW2_GPIO_Port, SW2_Pin, GPIO_PIN_SET);
//                          sw_state=1;
//                      }
//                      else
//                      {
//                          HAL_GPIO_WritePin(SW1_GPIO_Port, SW1_Pin, GPIO_PIN_RESET);
//                          HAL_GPIO_WritePin(SW2_GPIO_Port, SW2_Pin, GPIO_PIN_RESET);
//                          sw_state=0;
//                      }
//                  }
                    //if (gs.hold_counter<SAMPLE_HOLD_COUNT)
                    //{
                        gs.hold_counter++;
                        if (gs.hold_counter==SAMPLE_HOLD_COUNT)
                        {
                            //HAL_GPIO_WritePin(LED_HOLD_GPIO_Port, LED_HOLD_Pin, GPIO_PIN_RESET);
                            HAL_GPIO_WritePin(LED_LEFT_GPIO_Port, LED_LEFT_Pin, GPIO_PIN_RESET);
                            HAL_GPIO_WritePin(LED_RIGHT_GPIO_Port, LED_RIGHT_Pin, GPIO_PIN_RESET);
                            HAL_GPIO_WritePin(LED_UP_GPIO_Port, LED_UP_Pin, GPIO_PIN_RESET);
                            HAL_GPIO_WritePin(LED_DOWN_GPIO_Port, LED_DOWN_Pin, GPIO_PIN_RESET);
                            led_counter=LED_FLASH_DELAY;
                            LOG_DEBUG("                         Ges: HOLD!\r\n");
                            Ges_Log();
                            if (!sw_state)
                            {
                                //HAL_GPIO_WritePin(SW1_GPIO_Port, SW1_Pin, GPIO_PIN_SET);
                                //HAL_GPIO_WritePin(SW2_GPIO_Port, SW2_Pin, GPIO_PIN_SET);
                                sw_state=1;
                            }
                            else
                            {
                                //HAL_GPIO_WritePin(SW1_GPIO_Port, SW1_Pin, GPIO_PIN_RESET);
                                //HAL_GPIO_WritePin(SW2_GPIO_Port, SW2_Pin, GPIO_PIN_RESET);
                                sw_state=0;
                            }
                        }
                    //}

                    if(gs.hold_counter>5)
                    {
                        if (gs.sample_size==GES_SAMPLE_ARRAY_NUM_L)
                        {
                            gs.sample_size=GES_SAMPLE_ARRAY_NUM_S;
                            gs.sample_size_raw=GES_SAMPLE_RAW_ARRAY_NUM_S;
                        }
                        gs.last_ges = GES_NULL;
                        gs.hold_counter=0;
                    }
                }
                else if (res==GES_LEVEL2_DCLK)
                {
                    HAL_GPIO_WritePin(LED_DCLK_GPIO_Port, LED_DCLK_Pin, GPIO_PIN_RESET);
                    led_counter=LED_FLASH_DELAY;
                    LOG_DEBUG("                         Ges: DCLK!\r\n");
                    Ges_Log();
                }
                else if (res==GES_LEVEL2_APPROACH)
                {
                    if(gs.approach_counter)
                    {
                        LOG_DEBUG("                         Ges: APPROACH!\r\n");
                        HAL_GPIO_WritePin(LED_UP_GPIO_Port, LED_UP_Pin, GPIO_PIN_RESET);
                        led_counter=100;
                        Ges_Log();
                    }
                    gs.approach_counter++;
                }
                else if (res==GES_LEVEL2_LEAVE)
                {
                    LOG_DEBUG("                         Ges: LEAVE!\r\n");
                    HAL_GPIO_WritePin(LED_DOWN_GPIO_Port, LED_DOWN_Pin, GPIO_PIN_RESET);
                    led_counter=100;
                    Ges_Log();
                }
            }
            else
            {
                //��ǰ��long�������ı�sample_size��������
                if (gs.sample_size==GES_SAMPLE_ARRAY_NUM_L)
                {
                    gs.sample_size=GES_SAMPLE_ARRAY_NUM_S;
                    gs.sample_size_raw=GES_SAMPLE_RAW_ARRAY_NUM_S;
                    return;
                }
                gs.last_ges = GES_NULL;
                gs.hold_counter=0;
                gs.approach_counter=0;
                //HAL_GPIO_WritePin(LED_HOLD_GPIO_Port, LED_HOLD_Pin, GPIO_PIN_RESET);
            }
#if (LOG_ENABLE)
            LOG_DEBUG("\r\n\n");
#endif
        }
    }
}

//ϵͳʱ���Լ�
void Systick_Inc(void)
{
    __disable_irq();
    sys_tick++;
    __enable_irq();
}

//���ϵͳʱ��
uint64_t Systick_Get(void)
{
    return sys_tick;
}
