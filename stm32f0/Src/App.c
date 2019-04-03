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
#include "stm32f0xx_hal.h"
//#include "VCNL4035.h"
//#include "I2C.h"
#include "App.h"
#include "stdio.h"
//#include "arm_math.h"
//#include "arm_const_structs.h"
//#include "math.h"

uint64_t sys_tick=0;                                //ϵͳtick����
gs_struct gs={0};
uint16_t led_counter=0;
//uint16_t led
//uint8_t led_flag=0;

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
    uint16_t read_val;
    uint8_t res;

    App_Var_Init();
    
    //I2C_init();
    LOG_DEBUG("Start!\n");
    
    //res=VCNL4035_ReadData(VCNL4035_ID,&read_val);
    LOG_DEBUG("Read res: %d, ID: 0x%04x\n",res,read_val);

    //VCNL4035_InitGesMode();
}
/*
void LED_Tick_Check(void)
{
    if (led_counter)
    {
        led_counter--;
        if (!led_counter)
        {
            HAL_GPIO_WritePin(LED_LEFT_GPIO_Port, LED_LEFT_Pin, GPIO_PIN_RESET);    
            HAL_GPIO_WritePin(LED_RIGHT_GPIO_Port, LED_RIGHT_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_UP_GPIO_Port, LED_UP_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_DOWN_GPIO_Port, LED_DOWN_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_DCLK_GPIO_Port, LED_DCLK_Pin, GPIO_PIN_RESET);
        }
    }
}
*/
//����һ�β���ֵ
//���룺        ps->��ȡ�Ĳ���ֵ����
//���أ�        0-->����δ������
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
//���룺ps      --> ��ȡ�Ĳ���ֵ����
//      chn     --> Ѱ���ͨ����
//      pindex  --> ��ֵ��indexָ��
//���أ�            �ڴ�֡�о��е���������
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
//���룺    ps      --> ��ȡ�Ĳ���ֵ����
//      chn     --> Ѱ���ص�ͨ����
//      pindex  --> �����س�����ֵ��indexָ��
//���أ�                �ڴ�֡�о��е���������
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

//��ĳһ֡���в���Ѱ��
//���룺    chn     --> Ѱ���ص�ͨ����
//      pindex  --> ��ֵindexָ��
//���أ�                �ڴ�֡�о��е���������
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
        res=GES_LEVEL1_HOLD;
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
        //��ֵ�����һ������
        if (gs.wave[chn].peak_index==(gs.sample_size-1))
        {
            res=GES_LEVEL1_APPROACH;
            *pindex=gs.wave[chn].peak_index;
        }
        //��ֵ�ǵ�һ������
        else if (gs.wave[chn].peak_index==0)
        {
            res=GES_LEVEL1_LEAVE;
            *pindex=gs.wave[chn].peak_index;            
        }
        //��ֵ���м�
        else
        {
            res=GES_LEVEL1_FULL;
            *pindex=gs.wave[chn].peak_index;            
        }
    }
    return res;
}

//�ж��������η������Ⱥ��ϵ
//���룺chn1    --> ��һ����
//      chn2    --> �ڶ�����
//���أ�chn1    --> ��һ��������
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
//���أ��ڴ�֡�о��е���������
uint8_t Ges_Analysis(void)
{
    uint8_t i,basic_ges,res,chn,hold_flag;
    uint16_t index;
    
    res=GES_NULL;
    chn=0;
    hold_flag=0;
    
    //��׼����������
    Ges_Normalize();
    
    for (i=0;i<GES_CHN_NUM;i++)
    {
//      basic_ges=Ges_Peak_Find(gs.ges_sample_array[i],i,&index);
//      basic_ges=Ges_Edge_Find(gs.ges_sample_array[i],i,&index);
        basic_ges=Ges_Wave_Search(i,&index);
        if (basic_ges==GES_LEVEL1_APPROACH)
        {
            gs.ges_record[i].ges=basic_ges;
            gs.ges_record[i].add_time=gs.frame_start_tick+(index*SAMPLE_FREQUENCY_DEF*GES_SAMPLE_AVE_NUM);
            chn|=(1<<i);
//          res=1;
#if (LOG_ENABLE)
            LOG_DEBUG("tick: %lld, CHN%d ges: approach!\n",gs.ges_record[i].add_time,i);
#endif
        }
        else if (basic_ges==GES_LEVEL1_LEAVE)
        {
            gs.ges_record[i].ges=basic_ges;
            gs.ges_record[i].add_time=gs.frame_start_tick+(index*SAMPLE_FREQUENCY_DEF*GES_SAMPLE_AVE_NUM);
            chn|=(1<<i);
//          res=1;
#if (LOG_ENABLE)
            LOG_DEBUG("tick: %lld, CHN%d ges: leave!\n",gs.ges_record[i].add_time,i);
#endif
        }
        else if (basic_ges==GES_LEVEL1_FULL)
        {
            gs.ges_record[i].ges=basic_ges;
            gs.ges_record[i].add_time=gs.frame_start_tick+(index*SAMPLE_FREQUENCY_DEF*GES_SAMPLE_AVE_NUM);
            chn|=(1<<i);
//          res=1;
#if (LOG_ENABLE)
            LOG_DEBUG("tick: %lld, CHN%d ges: full!\n",gs.ges_record[i].add_time,i);    
#endif
        }
        else if (basic_ges==GES_LEVEL1_HOLD)
        {
            gs.ges_record[i].ges=basic_ges;
            gs.ges_record[i].add_time=gs.frame_start_tick+(index*SAMPLE_FREQUENCY_DEF*GES_SAMPLE_AVE_NUM);
            chn|=(1<<i);
#if (LOG_ENABLE)
            LOG_DEBUG("tick: %lld, CHN%d ges: hold!\n",gs.ges_record[i].add_time,i);
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
        left_up=Ges_Wave_Lead(GES_CHN_DOWN_LEFT,GES_CHN_UP);
        left_right=Ges_Wave_Lead(GES_CHN_DOWN_LEFT,GES_CHN_DOWN_RIGHT);
        right_up=Ges_Wave_Lead(GES_CHN_DOWN_RIGHT,GES_CHN_UP);
        
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

//���ƻ�׼���꣬����ƽ��ֵ����
void Ges_Calib(void)
{
    uint8_t i;
    uint16_t j,delta;
    uint32_t sum;
    for (i=0;i<GES_CHN_NUM;i++)
    {
        sum=0;
        for (j=0;j<gs.sample_size;j++)
        {
            sum+=gs.ges_sample_array[i][j];
        }
        delta=(uint16_t)(sum/gs.sample_size);
        gs.sample_base[i]=delta;
//      gs.sample_down_limit[i]=SAMPLE_DOWN_LIMIT_DEF+delta;
//      gs.sample_up_limit[i]=SAMPLE_UP_LIMIT_DEF+delta;
        LOG_DEBUG("CHN %d-> base: %d,down_limit: %d,up limit: %d\n",i,gs.sample_base[i],gs.sample_down_limit[i],gs.sample_up_limit[i]);
    }   
}

//��һ����������
void Ges_Normalize(void)
{
    uint8_t i,j;
    for (i=0;i<GES_CHN_NUM;i++)
    {
        for (j=0;j<gs.sample_size;j++)
        {
            if (gs.ges_sample_array[i][j]>=gs.sample_base[i])
                gs.ges_sample_array[i][j]=gs.ges_sample_array[i][j]-gs.sample_base[i];
            else
                gs.ges_sample_array[i][j]=0;
        }               
    }       
}

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
    LOG_DEBUG("this ges-->up: %lld/%d, down_left: %lld/%d, down_right: %lld/%d!\n",
                gs.ges_record[GES_CHN_UP].add_time,gs.ges_record[GES_CHN_UP].ges,
                gs.ges_record[GES_CHN_DOWN_LEFT].add_time,gs.ges_record[GES_CHN_DOWN_LEFT].ges,
                gs.ges_record[GES_CHN_DOWN_RIGHT].add_time,gs.ges_record[GES_CHN_DOWN_RIGHT].ges);
    
    LOG_DEBUG("last ges-->up: %d.%d/%d, down_left: %d.%d/%d, down_right: %d.%d/%d!\n",
                (uint32_t)(gs.last_ges_record[GES_CHN_UP].add_time/1000),(uint32_t)(gs.last_ges_record[GES_CHN_UP].add_time%1000),gs.last_ges_record[GES_CHN_UP].ges,
                (uint32_t)(gs.last_ges_record[GES_CHN_DOWN_LEFT].add_time/1000),(uint32_t)(gs.last_ges_record[GES_CHN_DOWN_LEFT].add_time%1000),gs.last_ges_record[GES_CHN_DOWN_LEFT].ges,
                (uint32_t)(gs.last_ges_record[GES_CHN_DOWN_RIGHT].add_time/1000),(uint32_t)(gs.last_ges_record[GES_CHN_DOWN_RIGHT].add_time%1000),gs.last_ges_record[GES_CHN_DOWN_RIGHT].ges);  
    LOG_DEBUG("CHN UP         rise_index: %d,peak_index: %d,fall_index: %d\n",gs.wave[GES_CHN_UP].rise_index,gs.wave[GES_CHN_UP].peak_index,gs.wave[GES_CHN_UP].fall_index);
    LOG_DEBUG("CHN DOWN LEFT  rise_index: %d,peak_index: %d,fall_index: %d\n",gs.wave[GES_CHN_DOWN_LEFT].rise_index,gs.wave[GES_CHN_DOWN_LEFT].peak_index,gs.wave[GES_CHN_DOWN_LEFT].fall_index);
    LOG_DEBUG("CHN DOWN RIGHT rise_index: %d,peak_index: %d,fall_index: %d\n",gs.wave[GES_CHN_DOWN_RIGHT].rise_index,gs.wave[GES_CHN_DOWN_RIGHT].peak_index,gs.wave[GES_CHN_DOWN_RIGHT].fall_index);
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
            LOG_DEBUG("CHN UP array:         ");
        else if (i==GES_CHN_DOWN_LEFT)
            LOG_DEBUG("CHN DOWN LEFT array:  ");        
        else if (i==GES_CHN_DOWN_RIGHT)
            LOG_DEBUG("CHN DOWN RIGHT array: ");            
        for (j=0;j<gs.sample_size;j++)
        {
            //��ӡȥ����׼ֵ�Ĺ�һ������
            if (gs.ges_sample_array[i][j]>=gs.sample_down_limit[i])
                LOG_DEBUG("%d ",gs.ges_sample_array[i][j]-gs.sample_down_limit[i]);
            else
                LOG_DEBUG("%d ",0);
//              LOG_DEBUG("%d ",gs.ges_sample_array[i][j]);
        }
        LOG_DEBUG("\n");                
    }
#endif
}

//������
void App_Task(void)
{
    static uint8_t calib_num=0;
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
        
        gs.sample_ready_flag=0;
        //VCNL4035_ReadPS(ps);
        //VCNL4035_ClearInt();
        getSensorDataByHostout(ps);
        
        res=Ges_Add_Sample(ps);
        //һ������������
        if (res==1)
        {
            LOG_DEBUG("sample[%d] %d",gs.sample_size,res);

            //��һ�β���������Ϊ������
            if (!calib_num)
            {
                Ges_Calib();
                calib_num++;
                return;
            }

            res=Ges_Analysis();
            if (res)
            {/*
                //��ǰ��short�������ı�sample_size��������
                if (gs.sample_size==GES_SAMPLE_ARRAY_NUM_S)
                {
                    gs.sample_size=GES_SAMPLE_ARRAY_NUM_L;
                    gs.sample_size_raw=GES_SAMPLE_RAW_ARRAY_NUM_L;
                    gs.ges_sample_raw_cur=GES_SAMPLE_ARRAY_NUM_S;
                    return;
                }*/
                
                uint64_t tick=Systick_Get();
                
                //�ϴ���HOLD����,�˴β���HOLD�Ҳ�Ϊ�գ������󴥷�
                if ((gs.last_ges==GES_LEVEL2_HOLD)&&(res!=GES_LEVEL2_HOLD))
                {
                    res=GES_NULL;
                    gs.last_ges=res;
                    gs.last_ges_time=tick;
                    return;
                }   
//              //�ϴ���HOLD���ƣ��˴�Ҳ��HOLD����
//              else if ((gs.last_ges==GES_LEVEL2_HOLD)&&(res==GES_LEVEL2_HOLD))
//              {
//                  gs.last_ges=res;
//                  gs.last_ges_time=tick;
////                    return;
//              }
                if (tick>gs.last_ges_time+GES_DISTANCE_MIN)
                {
                    gs.last_ges=res;
                    gs.last_ges_time=tick;
                }
                else
                    return;
                
                if (res==GES_LEVEL2_UP)
                {
                    //HAL_GPIO_WritePin(LED_UP_GPIO_Port, LED_UP_Pin, GPIO_PIN_SET);
                    led_counter=LED_FLASH_DELAY;
                    LOG_DEBUG("Ges: UP!\n");
                    Ges_Log();
                    //��SW2
                    //HAL_GPIO_WritePin(SW2_GPIO_Port, SW2_Pin, GPIO_PIN_SET);
                }
                else if (res==GES_LEVEL2_DOWN)
                {
                    //HAL_GPIO_WritePin(LED_DOWN_GPIO_Port, LED_DOWN_Pin, GPIO_PIN_SET);
                    led_counter=LED_FLASH_DELAY;
                    LOG_DEBUG("Ges: DOWN!\n");
                    Ges_Log();
                    //��SW2
                    //HAL_GPIO_WritePin(SW2_GPIO_Port, SW2_Pin, GPIO_PIN_RESET);
                }
                else if (res==GES_LEVEL2_LEFT)
                {
                    //HAL_GPIO_WritePin(LED_LEFT_GPIO_Port, LED_LEFT_Pin, GPIO_PIN_SET);
                    led_counter=LED_FLASH_DELAY;
                    LOG_DEBUG("Ges: LEFT!\n");
                    Ges_Log();
                    //��SW1
                    //HAL_GPIO_WritePin(SW1_GPIO_Port, SW1_Pin, GPIO_PIN_RESET);
                }
                else if (res==GES_LEVEL2_RIGHT)
                {
                    //HAL_GPIO_WritePin(LED_RIGHT_GPIO_Port, LED_RIGHT_Pin, GPIO_PIN_SET);
                    led_counter=LED_FLASH_DELAY;
                    LOG_DEBUG("Ges: RIGHT!\n");
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
                    if (gs.hold_counter<SAMPLE_HOLD_COUNT)
                    {
                        gs.hold_counter++;
                        if (gs.hold_counter==SAMPLE_HOLD_COUNT)
                        {
                            //HAL_GPIO_WritePin(LED_HOLD_GPIO_Port, LED_HOLD_Pin, GPIO_PIN_SET);
                            led_counter=LED_FLASH_DELAY;
                            LOG_DEBUG("Ges: HOLD!\n");
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
                    }
                }
                else if (res==GES_LEVEL2_DCLK)
                {
                    //HAL_GPIO_WritePin(LED_DCLK_GPIO_Port, LED_DCLK_Pin, GPIO_PIN_SET);
                    led_counter=LED_FLASH_DELAY;
                    LOG_DEBUG("Ges: DCLK!\n");
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
                gs.hold_counter=0;
                //HAL_GPIO_WritePin(LED_HOLD_GPIO_Port, LED_HOLD_Pin, GPIO_PIN_RESET);
            }
        }
    
//      VCNL4035_ReadData(VCNL4035_PS1_DATA,&ps1);
//      VCNL4035_ReadData(VCNL4035_PS2_DATA,&ps2);
//      VCNL4035_ReadData(VCNL4035_PS3_DATA,&ps3);
//      LOG_DEBUG("PS1: 0x%04x, PS2: 0x%04x, PS3: 0x%04x\n",ps1,ps2,ps3);
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
