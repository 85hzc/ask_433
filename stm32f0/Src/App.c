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

uint64_t sys_tick=0;                                //系统tick计数
gs_struct gs={0};
uint16_t led_counter=0;
//uint16_t led
//uint8_t led_flag=0;

//变量初始化
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

//应用初始化
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
//加入一次采样值
//输入：        ps->读取的采样值数组
//返回：        0-->数组未填充完毕
//          1-->数组已填充完毕
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
        //raw数组已填充完毕
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
    //在每一帧的开始记录时间
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

//手势采样技术
void Ges_Tick_Check(void)
{
    gs.sample_counter++;
    if (gs.sample_counter>=gs.sample_frequency)
    {
        gs.sample_counter=0;
        gs.sample_start_flag=1;
    }
}

//中断处理函数
void Ges_It_Handle(void)
{
    if (__HAL_GPIO_EXTI_GET_IT(INT_Pin)!=RESET)  
    {  
        gs.sample_ready_flag=1;
    }
}

#if (0)

//对某一帧进行寻找峰值
//输入：ps      --> 读取的采样值数组
//      chn     --> 寻峰的通道号
//      pindex  --> 峰值的index指针
//返回：            在此帧中具有的手势类型
uint8_t Ges_Peak_Find(uint16_t *ps,uint8_t chn,uint16_t *pindex)
{
    uint16_t i,res,max,min,max_index;
    res=GES_NULL;
    
    //求最大值、最小值及最大值、最小值位置
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
    
    //存在有效峰值
//  if ((max>gs.sample_up_limit[chn])&&(min<=gs.sample_down_limit[chn]))
    if (max>gs.sample_up_limit[chn])        
    {
        //峰值是最后一个数据
        if (max_index==(gs.sample_size-1))
        {
            res=GES_LEVEL1_APPROACH;
            *pindex=max_index;          
        }
        //峰值是第一个数据
        else if (max_index==0)
        {
            res=GES_LEVEL1_LEAVE;
            *pindex=max_index;          
        }
        //峰值在中间
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

//对某一帧进行寻找边沿
//输入：    ps      --> 读取的采样值数组
//      chn     --> 寻边沿的通道号
//      pindex  --> 上升沿超过阈值的index指针
//返回：                在此帧中具有的手势类型
uint8_t Ges_Edge_Find(uint16_t *ps,uint8_t chn,uint16_t *pindex)
{
    uint16_t i,res,rise,fall,rise_index,fall_index;
    res=GES_NULL;
    
    //从初始值开始寻找上升沿
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
        //从上次寻找地方开始
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
    
    //从末尾值开始寻找下降沿
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

//对某一帧进行波形寻找
//输入：    chn     --> 寻边沿的通道号
//      pindex  --> 峰值index指针
//返回：                在此帧中具有的手势类型
uint8_t Ges_Wave_Search(uint8_t chn,uint16_t *pindex)
{
    uint16_t i,res;
//  uint32_t sum;
    uint8_t hold_flag=1;
    res=GES_NULL;
    
    //初始化
    gs.wave[chn].fall_index=GES_INVALID_INDEX;
    gs.wave[chn].rise_index=GES_INVALID_INDEX;
    gs.wave[chn].peak_index=GES_INVALID_INDEX;
    
    //求峰值、上升沿、下降沿及峰值、上升沿、下降沿位置和平均值
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
        //峰值是最后一个数据
        if (gs.wave[chn].peak_index==(gs.sample_size-1))
        {
            res=GES_LEVEL1_APPROACH;
            *pindex=gs.wave[chn].peak_index;
        }
        //峰值是第一个数据
        else if (gs.wave[chn].peak_index==0)
        {
            res=GES_LEVEL1_LEAVE;
            *pindex=gs.wave[chn].peak_index;            
        }
        //峰值在中间
        else
        {
            res=GES_LEVEL1_FULL;
            *pindex=gs.wave[chn].peak_index;            
        }
    }
    return res;
}

//判断两个波形发生的先后关系
//输入：chn1    --> 第一波形
//      chn2    --> 第二波形
//返回：chn1    --> 第一波形领先
//      chn2    --> 第二波形领先
//      0xFF    --> 无法判断
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

//对某一帧进行手势分析
//返回：在此帧中具有的手势类型
uint8_t Ges_Analysis(void)
{
    uint8_t i,basic_ges,res,chn,hold_flag;
    uint16_t index;
    
    res=GES_NULL;
    chn=0;
    hold_flag=0;
    
    //标准化手势数组
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
    
    //有新的手势
    if (chn)
    {
        if (hold_flag)
        {
            res=GES_LEVEL2_HOLD;
            return res;
        }
        if (chn&(1<<GES_CHN_UP))
            //重置last_ges_record
            gs.last_ges_record[GES_CHN_UP]=gs.ges_record[GES_CHN_UP];
        if (chn&(1<<GES_CHN_DOWN_LEFT))
            //重置last_ges_record
            gs.last_ges_record[GES_CHN_DOWN_LEFT]=gs.ges_record[GES_CHN_DOWN_LEFT];
        if (chn&(1<<GES_CHN_DOWN_RIGHT))
            //重置last_ges_record
            gs.last_ges_record[GES_CHN_DOWN_RIGHT]=gs.ges_record[GES_CHN_DOWN_RIGHT];
        
        uint8_t left_up,left_right,right_up;
        left_up=Ges_Wave_Lead(GES_CHN_DOWN_LEFT,GES_CHN_UP);
        left_right=Ges_Wave_Lead(GES_CHN_DOWN_LEFT,GES_CHN_DOWN_RIGHT);
        right_up=Ges_Wave_Lead(GES_CHN_DOWN_RIGHT,GES_CHN_UP);
        
        //从左到右
        if ((left_up==GES_CHN_DOWN_LEFT)&&(right_up==GES_CHN_UP))
        {
            res=GES_LEVEL2_RIGHT;
            return res;             
        }
        //从右到左
        if ((left_up==GES_CHN_UP)&&(right_up==GES_CHN_DOWN_RIGHT))
        {
            res=GES_LEVEL2_LEFT;
            return res;             
        }           
        //从上到下
        if ((left_up==GES_CHN_UP)||(right_up==GES_CHN_UP))
        {
            res=GES_LEVEL2_DOWN;
            return res;
        }
        //从下到上
        if ((left_up==GES_CHN_DOWN_LEFT)||(right_up==GES_CHN_DOWN_RIGHT))
        {
            res=GES_LEVEL2_UP;
            return res;
        }           
        //从左到右
        if (left_right==GES_CHN_DOWN_LEFT)
        {
            res=GES_LEVEL2_RIGHT;
            return res;         
        }
        //从右到左
        if (left_right==GES_CHN_DOWN_RIGHT)
        {
            res=GES_LEVEL2_LEFT;
            return res;         
        }               
        
#if (0)     
        //此次是UP位置
        if (chn&(1<<GES_CHN_UP))
        {
            //从左到右
            if ((gs.last_ges_record[GES_CHN_DOWN_LEFT].add_time<gs.ges_record[GES_CHN_UP].add_time)&&
                (gs.last_ges_record[GES_CHN_DOWN_RIGHT].add_time>gs.ges_record[GES_CHN_UP].add_time)&&
                (gs.last_ges_record[GES_CHN_DOWN_LEFT].add_time+GES_TIME_DIF_MAX>gs.ges_record[GES_CHN_UP].add_time)&&
                (gs.last_ges_record[GES_CHN_DOWN_RIGHT].add_time<gs.ges_record[GES_CHN_UP].add_time+GES_TIME_DIF_MAX))
            {
                res=GES_LEVEL2_RIGHT;
                return res;
            }
            //从右到左
            if ((gs.last_ges_record[GES_CHN_DOWN_RIGHT].add_time<gs.ges_record[GES_CHN_UP].add_time)&&
                (gs.last_ges_record[GES_CHN_DOWN_LEFT].add_time>gs.ges_record[GES_CHN_UP].add_time)&&
                (gs.last_ges_record[GES_CHN_DOWN_RIGHT].add_time+GES_TIME_DIF_MAX>gs.ges_record[GES_CHN_UP].add_time)&&
                (gs.last_ges_record[GES_CHN_DOWN_LEFT].add_time<gs.ges_record[GES_CHN_UP].add_time+GES_TIME_DIF_MAX))
            {
                res=GES_LEVEL2_LEFT;
                return res;
            }
            //从上到下
            if (((gs.last_ges_record[GES_CHN_DOWN_LEFT].add_time<gs.ges_record[GES_CHN_UP].add_time)&&
                (gs.last_ges_record[GES_CHN_DOWN_LEFT].add_time+GES_TIME_DIF_MAX>gs.ges_record[GES_CHN_UP].add_time))||
                ((gs.last_ges_record[GES_CHN_DOWN_RIGHT].add_time<gs.ges_record[GES_CHN_UP].add_time)&&
                (gs.last_ges_record[GES_CHN_DOWN_RIGHT].add_time+GES_TIME_DIF_MAX>gs.ges_record[GES_CHN_UP].add_time)))
            {
                res=GES_LEVEL2_UP;
                return res;
            }
            //从下到上
            if (((gs.last_ges_record[GES_CHN_DOWN_LEFT].add_time>gs.ges_record[GES_CHN_UP].add_time)&&
                (gs.last_ges_record[GES_CHN_DOWN_LEFT].add_time<gs.ges_record[GES_CHN_UP].add_time+GES_TIME_DIF_MAX))||
                ((gs.last_ges_record[GES_CHN_DOWN_RIGHT].add_time>gs.ges_record[GES_CHN_UP].add_time)&&
                (gs.last_ges_record[GES_CHN_DOWN_RIGHT].add_time<gs.ges_record[GES_CHN_UP].add_time+GES_TIME_DIF_MAX)))
            {
                res=GES_LEVEL2_DOWN;
                return res;
            }    
        }
        //此次是DOWN_LEFT位置
        if (chn&(1<<GES_CHN_DOWN_LEFT))
        {
            //从左到右
            if ((gs.last_ges_record[GES_CHN_UP].add_time>gs.ges_record[GES_CHN_DOWN_LEFT].add_time)&&
                (gs.last_ges_record[GES_CHN_DOWN_RIGHT].add_time>gs.ges_record[GES_CHN_DOWN_LEFT].add_time)&&
                (gs.last_ges_record[GES_CHN_UP].add_time<gs.ges_record[GES_CHN_UP].add_time+GES_TIME_DIF_MAX)&&
                (gs.last_ges_record[GES_CHN_DOWN_RIGHT].add_time<gs.ges_record[GES_CHN_DOWN_LEFT].add_time+GES_TIME_DIF_MAX))
            {
                res=GES_LEVEL2_RIGHT;
                return res;
            }
            //从右到左
            if ((gs.last_ges_record[GES_CHN_UP].add_time<gs.ges_record[GES_CHN_DOWN_LEFT].add_time)&&
                (gs.last_ges_record[GES_CHN_DOWN_RIGHT].add_time<gs.ges_record[GES_CHN_DOWN_LEFT].add_time)&&
                (gs.last_ges_record[GES_CHN_UP].add_time+GES_TIME_DIF_MAX>gs.ges_record[GES_CHN_DOWN_LEFT].add_time)&&
                (gs.last_ges_record[GES_CHN_DOWN_RIGHT].add_time+GES_TIME_DIF_MAX>gs.ges_record[GES_CHN_DOWN_LEFT].add_time))
            {
                res=GES_LEVEL2_LEFT;
                return res;
            }
            //从上到下
            if ((gs.last_ges_record[GES_CHN_UP].add_time<gs.ges_record[GES_CHN_DOWN_LEFT].add_time)&&
                (gs.last_ges_record[GES_CHN_UP].add_time+GES_TIME_DIF_MAX>gs.ges_record[GES_CHN_DOWN_LEFT].add_time))
            {
                res=GES_LEVEL2_UP;
                return res;
            }
            //从下到上
            if ((gs.last_ges_record[GES_CHN_UP].add_time>gs.ges_record[GES_CHN_DOWN_LEFT].add_time)&&
                (gs.last_ges_record[GES_CHN_UP].add_time<gs.ges_record[GES_CHN_DOWN_LEFT].add_time+GES_TIME_DIF_MAX))
            {
                res=GES_LEVEL2_DOWN;
                return res;
            }           
        }   
        //此次是DOWN_RIGHT位置
        if (chn&(1<<GES_CHN_DOWN_RIGHT))
        {
            //从左到右
            if ((gs.last_ges_record[GES_CHN_UP].add_time<gs.ges_record[GES_CHN_DOWN_RIGHT].add_time)&&
                (gs.last_ges_record[GES_CHN_DOWN_LEFT].add_time<gs.ges_record[GES_CHN_DOWN_RIGHT].add_time)&&
                (gs.last_ges_record[GES_CHN_UP].add_time+GES_TIME_DIF_MAX>gs.ges_record[GES_CHN_DOWN_RIGHT].add_time)&&
                (gs.last_ges_record[GES_CHN_DOWN_LEFT].add_time+GES_TIME_DIF_MAX>gs.ges_record[GES_CHN_DOWN_RIGHT].add_time))
            {
                res=GES_LEVEL2_RIGHT;
                return res;
            }
            //从右到左
            if ((gs.last_ges_record[GES_CHN_UP].add_time>gs.ges_record[GES_CHN_DOWN_RIGHT].add_time)&&
                (gs.last_ges_record[GES_CHN_DOWN_LEFT].add_time>gs.ges_record[GES_CHN_DOWN_RIGHT].add_time)&&
                (gs.last_ges_record[GES_CHN_UP].add_time<gs.ges_record[GES_CHN_DOWN_RIGHT].add_time+GES_TIME_DIF_MAX)&&
                (gs.last_ges_record[GES_CHN_DOWN_LEFT].add_time<gs.ges_record[GES_CHN_DOWN_RIGHT].add_time+GES_TIME_DIF_MAX))
            {
                res=GES_LEVEL2_LEFT;
                return res;
            }
            //从上到下
            if ((gs.last_ges_record[GES_CHN_UP].add_time<gs.ges_record[GES_CHN_DOWN_RIGHT].add_time)&&
                (gs.last_ges_record[GES_CHN_UP].add_time+GES_TIME_DIF_MAX>gs.ges_record[GES_CHN_DOWN_RIGHT].add_time))
            {
                res=GES_LEVEL2_UP;
                return res;
            }
            //从下到上
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

//手势基准定标，采用平均值定标
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

//归一化采样数组
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

////判断是否需要改变sample_size并改变
////返回值：    0-->    不需要改变
////            1-->    需要改变
//uint8_t Ges_SampleSize_Switch(void)
//{
//  uint16_t i;
//  for (i=0;i<gs.sample_size;i++)
//  {
//      
//  }
//}

//打印相关手势信息
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
            //打印去除基准值的归一化数组
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

//主任务
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
        //一次数组填充完毕
        if (res==1)
        {
            LOG_DEBUG("sample[%d] %d",gs.sample_size,res);

            //第一次采样数据作为定标用
            if (!calib_num)
            {
                Ges_Calib();
                calib_num++;
                return;
            }

            res=Ges_Analysis();
            if (res)
            {/*
                //当前是short采样，改变sample_size继续采样
                if (gs.sample_size==GES_SAMPLE_ARRAY_NUM_S)
                {
                    gs.sample_size=GES_SAMPLE_ARRAY_NUM_L;
                    gs.sample_size_raw=GES_SAMPLE_RAW_ARRAY_NUM_L;
                    gs.ges_sample_raw_cur=GES_SAMPLE_ARRAY_NUM_S;
                    return;
                }*/
                
                uint64_t tick=Systick_Get();
                
                //上次是HOLD手势,此次不是HOLD且不为空，避免误触发
                if ((gs.last_ges==GES_LEVEL2_HOLD)&&(res!=GES_LEVEL2_HOLD))
                {
                    res=GES_NULL;
                    gs.last_ges=res;
                    gs.last_ges_time=tick;
                    return;
                }   
//              //上次是HOLD手势，此次也是HOLD手势
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
                    //开SW2
                    //HAL_GPIO_WritePin(SW2_GPIO_Port, SW2_Pin, GPIO_PIN_SET);
                }
                else if (res==GES_LEVEL2_DOWN)
                {
                    //HAL_GPIO_WritePin(LED_DOWN_GPIO_Port, LED_DOWN_Pin, GPIO_PIN_SET);
                    led_counter=LED_FLASH_DELAY;
                    LOG_DEBUG("Ges: DOWN!\n");
                    Ges_Log();
                    //关SW2
                    //HAL_GPIO_WritePin(SW2_GPIO_Port, SW2_Pin, GPIO_PIN_RESET);
                }
                else if (res==GES_LEVEL2_LEFT)
                {
                    //HAL_GPIO_WritePin(LED_LEFT_GPIO_Port, LED_LEFT_Pin, GPIO_PIN_SET);
                    led_counter=LED_FLASH_DELAY;
                    LOG_DEBUG("Ges: LEFT!\n");
                    Ges_Log();
                    //关SW1
                    //HAL_GPIO_WritePin(SW1_GPIO_Port, SW1_Pin, GPIO_PIN_RESET);
                }
                else if (res==GES_LEVEL2_RIGHT)
                {
                    //HAL_GPIO_WritePin(LED_RIGHT_GPIO_Port, LED_RIGHT_Pin, GPIO_PIN_SET);
                    led_counter=LED_FLASH_DELAY;
                    LOG_DEBUG("Ges: RIGHT!\n");
                    Ges_Log();
                    //开SW1
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
                //当前是long采样，改变sample_size继续采样
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

//系统时钟自加
void Systick_Inc(void)
{
    __disable_irq();
    sys_tick++;
    __enable_irq();
}

//获得系统时钟
uint64_t Systick_Get(void)
{
    return sys_tick;
}
