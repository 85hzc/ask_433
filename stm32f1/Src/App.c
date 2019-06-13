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
#include "siliconSi115x.h"

extern volatile uint16_t I2C_SDA_PIN;
extern volatile uint16_t I2C_SCL_PIN;

extern TIM_HandleTypeDef htim3;
extern uint32_t          turn_off;
uint16_t                 brightness = 500;
static uint32_t          tickstart;
uint64_t                 sys_tick=0;          //系统tick计数
gs_struct gs={0};
uint16_t led_counter=0;
//uint16_t led

gs_comb_struct comb_ges;

void init_counter(void)
{
    gs.hold_flag=0;
    gs.hold_counter=0;
    gs.approach_counter=0;
    gs.leave_counter=0;
    gs.new_hold_AorL_size = GES_SAMPLE_RAW_ARRAY_NUM_L;
}

//变量初始化
void App_Var_Init(void)
{
    //gs.sample_counter=0;
    //gs.sample_ready_flag=0;
    //gs.sample_start_flag=0;
    //gs.sample_frequency=SAMPLE_FREQUENCY_DEF;
    gs.sample_size=GES_SAMPLE_ARRAY_NUM_S;
    gs.sample_size_raw=GES_SAMPLE_RAW_ARRAY_NUM_S;
    gs.new_hold_AorL_size = GES_SAMPLE_RAW_ARRAY_NUM_L;

    for (uint8_t i=0;i<GES_CHN_NUM;i++)
    {
        gs.sample_down_limit[i]=SAMPLE_DOWN_LIMIT_DEF;
        gs.sample_up_limit[i]=SAMPLE_UP_LIMIT_DEF;
    }

    memset(&comb_ges, 0, sizeof(gs_comb_struct));
}

//应用初始化
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

//加入一次采样值
//输入：     ps->读取的采样值数组
//返回：     0-->数组未填充完毕
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

//对每一次采样值判断是否手势到来
//输入：     ps->读取的采样值数组
//返回：     0-->数组未完全包含手势
//        1-->数组手势已完毕
uint8_t Ges_Detect_Sample(uint16_t *ps)
{
    uint8_t i,j,k,new_hold_AorL_size;
    uint32_t sum;
    uint16_t ges_dynamic_sample_raw_cur_temp;


    if(!gs.ges_dynamic_start && 
        ((ps[0]-gs.sample_base_last[0]>SAMPLE_DOWN_LIMIT_DEF) ||
        (ps[1]-gs.sample_base_last[1]>SAMPLE_DOWN_LIMIT_DEF)))
    {
        LOG_DEBUG("ges start.\r\n");
        gs.ges_sample_raw_cur=0; //基准值数组复位

        gs.ges_dynamic_start = true;
        gs.ges_dynamic_sample_size = 0;
        gs.ges_dynamic_sample_raw_cur = 0;
        //gs.sample_size = GES_SAMPLE_ARRAY_NUM_L;
        //gs.sample_size_raw = GES_SAMPLE_RAW_ARRAY_NUM_L;

        //hold_time = 
    }
    else
    {
    }

    if(gs.ges_dynamic_start)
    {
        for (i=0;i<GES_CHN_NUM;i++)
        {
            ges_dynamic_sample_raw_cur_temp=gs.ges_dynamic_sample_raw_cur;
            gs.ges_dynamic_sample_array[i][ges_dynamic_sample_raw_cur_temp]=ps[i];
            ges_dynamic_sample_raw_cur_temp++;
        }
        //在每一帧的开始记录时间
        //if (gs.ges_sample_raw_cur==0)
        //    gs.frame_start_tick=Systick_Get();

        gs.ges_dynamic_sample_size++; //can be the last array index[24]
        gs.ges_dynamic_sample_raw_cur++;
        if (gs.ges_dynamic_sample_raw_cur>=gs.new_hold_AorL_size)
        {
            LOG_DEBUG("sample array overflow!  HOLD/AP/LE size[%d].\r\n", gs.new_hold_AorL_size);
            gs.ges_dynamic_start = 0;
            gs.ges_sample_raw_cur=0;            //基准值计算数组此时重新开始取值
            return 1;
        }

        if(gs.ges_dynamic_start &&
            (ps[0]-gs.sample_base_last[0]<SAMPLE_DOWN_LIMIT_DEF) &&
            (ps[1]-gs.sample_base_last[1]<SAMPLE_DOWN_LIMIT_DEF) &&
            (ps[2]-gs.sample_base_last[2]<SAMPLE_DOWN_LIMIT_DEF))
        {
            LOG_DEBUG("ges stop.\r\n");

            LOG_DEBUG("hold_flag=0.\r\n");
            gs.hold_flag = 0;

            gs.ges_sample_raw_cur=0;            //基准值计算数组此时重新开始取值
            gs.ges_dynamic_start = 0;
            return 1;
        }
    }

    return 0;
}

//手势采样技术
/*
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
*/
#if (0)

//对某一帧进行寻找峰值
//输入： ps      --> 读取的采样值数组
//      chn     --> 寻峰的通道号
//      pindex  --> 峰值的index指针
//返回：             在此帧中具有的手势类型
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
//输入： ps      --> 读取的采样值数组
//      chn     --> 寻边沿的通道号
//      pindex  --> 上升沿超过阈值的index指针
//返回：             在此帧中具有的手势类型
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


//对某一帧进行波形趋势寻找
//输入： chn     --> 寻边沿的通道号
//
//返回：             在此帧中具有的手势类型   （接近/原理）
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

    if(total[0]>total[1] && total[1]>total[2] &&
        total[2]>40)
    {
        LOG_DEBUG("Ges_Trend_Find {chn[%d] leave}\r\n",chn);
        return GES_LEVEL1_LEAVE;
    }
    if(total[0]<total[1] && total[1]<total[2]) //&& total[0]>40)
    {
        LOG_DEBUG("Ges_Trend_Find {chn[%d] apporach}\r\n",chn);
        return GES_LEVEL1_APPROACH;
    }

    return GES_NULL;
}

uint8_t Ges_Trend_DynamicFind(uint8_t chn)
{
    uint8_t i,j,num_per_section;
    uint32_t total[SECTION_NUM];

    num_per_section = GES_SAMPLE_ARRAY_NUM_L/SECTION_NUM;

    memset(total,0,sizeof(total));
    for(i=0;i<SECTION_NUM;i++)
    {
        for(j=0;j<num_per_section;j++)
        {
            total[i] += gs.ges_dynamic_sample_array[chn][j+i*num_per_section];
        }
    }
    LOG_DEBUG("                                                 chn[%d] leave %d %d %d  %d\r\n",chn,total[0],total[1],total[2],total[0]-total[2]);

    if(total[0]>total[1] && total[1]>total[2] &&
        total[0]-total[2]>=80)
    {
        LOG_DEBUG("                                                 Ges_Trend_Find  leave\r\n");
        return GES_LEVEL1_LEAVE;
    }
    if(total[0]<total[1] && total[1]<total[2] &&
        total[2]-total[0]>=80)
    {
        LOG_DEBUG("                                                 Ges_Trend_Find  appor\r\n");
        return GES_LEVEL1_APPROACH;
    }

    return GES_NULL;
}

uint8_t Ges_Hold_Judge(uint8_t chn)
{
    uint8_t i,j,res=0;
    uint32_t total[8];
    uint32_t tt= 0, tt1=0;


    memset(total,0,sizeof(total));
    for(i=0;i<8;i++)
    {
        for(j=0;j<3;j++)
        {
            total[i] += gs.ges_dynamic_sample_array[chn][j+i*3];
            tt += gs.ges_dynamic_sample_array[chn][j+i*3];
        }
    }

    tt1 = tt/8;
    for(i=0;i<8;i++)
    {
        //LOG_DEBUG("[i:%d] %d\r\n",i,abs(tt1-total[i])*100/tt1);
        if(abs(tt1-total[i])*100/tt1>70)
        {
            LOG_DEBUG("Ges_Hold_Judge fail 2.\r\n");
            return 2;
        }
        if(abs(tt1-total[i])*100/tt1>10)
        {
            res = 1;
        }
    }

    LOG_DEBUG("Ges_Hold_Judge fail %d.\r\n",res);
    return res;
}

uint8_t Ges_Hold_Height_Judge()
{
    uint8_t chn;
    uint8_t i,res=0;
    uint64_t total[2]=0;
    uint32_t tt1[2]=0;

    for(chn=0;chn<2;chn++)
    {
        for(i=0;i<GES_SAMPLE_ARRAY_NUM_L;i++)
        {
            total[chn] += gs.ges_dynamic_sample_array[chn][i];
        }
        tt1[chn] = total[chn]/GES_SAMPLE_ARRAY_NUM_L;
        LOG_DEBUG("Ges_Hold_Height_Judge ch%d[%d].\r\n",chn,tt1[chn]);
    }

    if(tt1[0]+tt1[1]<200)
    {
        LOG_DEBUG("Ges_Hold_Height_Judge too low!\r\n");
        return 0;
    }

    if(!gs.hold_height_start)
        gs.hold_height_start = tt1[0]+tt1[1];
    else if(!gs.hold_height_stop)
        gs.hold_height_stop = tt1[0]+tt1[1];
    else
    {
        gs.hold_height_start = gs.hold_height_stop;
        gs.hold_height_stop = tt1[0]+tt1[1];
    }
    
    LOG_DEBUG("gs.hold_flag[%d]!\r\n",gs.hold_flag);

    if(gs.hold_flag)
    {
        if(gs.hold_height_start<gs.hold_height_stop)
        {

            LOG_DEBUG("Ges_Hold_Height_Judge approach[%d]!\r\n",gs.hold_height_stop-gs.hold_height_start);

            for(i=0;i<gs.hold_height_stop-gs.hold_height_start;i++)
            {
                brightness++;
                if(brightness>1000)
                {
                    return 1;
                }
                __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1, (uint32_t)brightness);
            }
        }
        else
        {
            LOG_DEBUG("Ges_Hold_Height_Judge leave[%d]!\r\n",gs.hold_height_start-gs.hold_height_stop);

            for(i=0;i<gs.hold_height_start-gs.hold_height_stop;i++)
            {
                brightness--;
                if(brightness<10)
                {
                    return 1;
                }
                __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1, (uint32_t)brightness);
            }
        }
        //turn_off = 1;
    }

    return 1;
}



uint8_t Ges_Hold_Height_Valid_Judge()
{
    uint8_t chn;
    uint8_t i,res=0;
    uint64_t total[2]=0;
    uint32_t tt1[2]=0;

    for(chn=0;chn<2;chn++)
    {
        for(i=0;i<GES_SAMPLE_ARRAY_NUM_L;i++)
        {
            total[chn] += gs.ges_dynamic_sample_array[chn][i];
        }
        tt1[chn] = total[chn]/GES_SAMPLE_ARRAY_NUM_L;
        LOG_DEBUG("Ges_Hold_Height_Judge ch%d[%d].\r\n",chn,tt1[chn]);
    }

    if(tt1[0]+tt1[1]<250)
    {
        LOG_DEBUG("Ges_Hold_Height_Judge < 250 too low!\r\n");
        return 0;
    }

    return 1;
}


uint8_t Ges_Hold_Height_AorL_Judge()
{
    uint8_t chn;
    uint8_t i,res=0;
    uint64_t total[2]=0;
    uint32_t tt1[2]=0;

    for(chn=0;chn<2;chn++)
    {
        for(i=0;i<GES_SAMPLE_ARRAY_NUM_S;i++)
        {
            total[chn] += gs.ges_dynamic_sample_array[chn][i];
        }
        tt1[chn] = total[chn]/GES_SAMPLE_ARRAY_NUM_S;
        LOG_DEBUG("Ges_Hold_Height_AorL_Judge ch%d[%d].\r\n",chn,tt1[chn]);
    }

    if(!gs.hold_height_start)
        gs.hold_height_start = tt1[0]+tt1[1];
    else if(!gs.hold_height_stop)
        gs.hold_height_stop = tt1[0]+tt1[1];
    else
    {
        gs.hold_height_start = gs.hold_height_stop;
        gs.hold_height_stop = tt1[0]+tt1[1];
    }
    LOG_DEBUG("stop-start[%d]!\r\n",gs.hold_height_stop-gs.hold_height_start);

    if(gs.hold_height_start<gs.hold_height_stop-10)
    {
        LOG_DEBUG("Approach!\r\n");
        brightness+=20;
        if(brightness>1000)
        {
            return 1;
        }
        __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1, (uint32_t)brightness);
    }
    else if(gs.hold_height_start-10>gs.hold_height_stop)
    {
        LOG_DEBUG("Leave!\r\n");
        brightness-=20;
        if(brightness<100)
        {
            return 1;
        }
        __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1, (uint32_t)brightness);
    }
    //turn_off = 1;

    return 1;
}

//对某一帧进行波形寻找
//输入： chn     --> 寻边沿的通道号
//      pindex  --> 峰值index指针
//返回：             在此帧中具有的手势类型
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
        LOG_DEBUG("                                                 after a hold Ges.\r\n");
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
            //峰值是最后一个数据
            if (gs.wave[chn].peak_index==(gs.sample_size-1))
            {
                res=GES_LEVEL1_APPROACH;
                //*pindex=gs.wave[chn].peak_index;
            }
            //峰值是第一个数据
            else if (gs.wave[chn].peak_index==0)
            {
                res=GES_LEVEL1_LEAVE;
                //*pindex=gs.wave[chn].peak_index;
            }
            //峰值在中间
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

//判断两个波形发生的先后关系
//输入： chn1    --> 第一波形
//      chn2    --> 第二波形
//返回： chn1    --> 第一波形领先
//      chn2    --> 第二波形领先
//      0xFF    --> 无法判断
uint8_t Ges_Wave_Lead(uint8_t chn1,uint8_t chn2)
{
    if ((gs.wave[chn1].rise_index!=GES_INVALID_INDEX)&&
        (gs.wave[chn2].rise_index!=GES_INVALID_INDEX))
    {
        if (gs.wave[chn1].rise_index<gs.wave[chn2].rise_index)
            return chn1;
        else if (gs.wave[chn1].rise_index>gs.wave[chn2].rise_index)
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

    if ((gs.wave[chn1].peak_index!=GES_INVALID_INDEX)&&
        (gs.wave[chn2].peak_index!=GES_INVALID_INDEX))
    {
        if (gs.wave[chn1].peak_index<gs.wave[chn2].peak_index)
            return chn1;
        else if (gs.wave[chn1].peak_index>gs.wave[chn2].peak_index)
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


//对某一帧进行波形寻找
//输入： chn     --> 寻边沿的通道号
//      pindex  --> 峰值index指针
//返回：             在此帧中具有的手势类型
uint8_t Ges_Wave_DynamicSearch(uint8_t chn,uint16_t *pindex)
{
    uint16_t i,res;
//  uint32_t sum;
    uint8_t rise_flag = 0;
    uint8_t wave_flag=0,hold_flag=1, hold_section_times;
    res=GES_NULL;
    
    //初始化
    gs.wave[chn].fall_index=GES_INVALID_INDEX;
    gs.wave[chn].rise_index=GES_INVALID_INDEX;
    gs.wave[chn].peak_index=GES_INVALID_INDEX;

    hold_section_times = gs.ges_dynamic_sample_size/2;
/*
    if(gs.ges_dynamic_sample_size==GES_SAMPLE_ARRAY_NUM_L)
    {
        wave_flag = Ges_Hold_Judge(chn);
    }
*/
    //求峰值、上升沿、下降沿及峰值、上升沿、下降沿位置和平均值
    for (i=0,gs.wave[chn].peak_value=0;i<gs.ges_dynamic_sample_size;i++)
    {
        if ((i>=hold_section_times)&&(gs.ges_dynamic_sample_array[chn][i]<SAMPLE_HOLD_LIMIT_DEF))
            hold_flag=0;

        if ((gs.ges_dynamic_sample_array[chn][i]>gs.wave[chn].peak_value)&&
            (gs.ges_dynamic_sample_array[chn][i]>=gs.sample_up_limit[chn]))
        {
            gs.wave[chn].peak_value=gs.ges_dynamic_sample_array[chn][i];
            gs.wave[chn].peak_index=i;
        }

        if (i<gs.ges_dynamic_sample_size-1)
        {
            if (i==0) //动态采用的数组，第一个元素已经符合采样阈值
            {
                if (gs.ges_dynamic_sample_array[chn][i]>=gs.sample_down_limit[chn])
                {
                    gs.wave[chn].rise_value=gs.ges_dynamic_sample_array[chn][i];
                    gs.wave[chn].rise_index=i;
                }
            }

            //还未发现波峰之前，rise 有可能需要更新，增强健壮性
            if ((gs.ges_dynamic_sample_array[chn][i]<gs.sample_down_limit[chn])&&
                (gs.ges_dynamic_sample_array[chn][i+1]>=gs.sample_down_limit[chn])&&
                ((gs.wave[chn].rise_index==GES_INVALID_INDEX)||gs.wave[chn].peak_index==GES_INVALID_INDEX))
            {
                gs.wave[chn].rise_value=gs.ges_dynamic_sample_array[chn][i+1];
                gs.wave[chn].rise_index=i+1;
            }
            if ((gs.ges_dynamic_sample_array[chn][i]>=gs.sample_down_limit[chn])&&
                (gs.ges_dynamic_sample_array[chn][i+1]<gs.sample_down_limit[chn])&&
                (gs.wave[chn].fall_index==GES_INVALID_INDEX)&&
                (gs.wave[chn].peak_index!=GES_INVALID_INDEX))
            {
                gs.wave[chn].fall_value=gs.ges_dynamic_sample_array[chn][i];
                gs.wave[chn].fall_index=i;
            }
        }
    }

    if ((gs.ges_dynamic_sample_size==gs.new_hold_AorL_size/*32*/)&&(hold_flag))
    {
        LOG_DEBUG("                                                 after a hold Judge.\r\n");
/*
        if(gs.hold_flag)
        {
            res = Ges_Trend_DynamicFind(chn);
        }
*/
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

    if ((gs.wave[chn].peak_index!=GES_INVALID_INDEX)&&(gs.wave[chn].peak_value>gs.sample_up_limit[chn]))
    {
        LOG_DEBUG("                                                 after not hold or less %d.\r\n",GES_SAMPLE_ARRAY_NUM_L);

        if(GES_SAMPLE_ARRAY_NUM_L == gs.ges_dynamic_sample_size)//采样队列满
        {
            if(gs.hold_flag)
            {
                res = Ges_Trend_DynamicFind(chn);
            }
            else if(/*2==wave_flag && */GES_NULL==res)
            {
                res = GES_LEVEL1_FULL;
            }
        }
        else
        {
            //峰值是最后一个数据
            if (gs.wave[chn].peak_index==gs.ges_dynamic_sample_size-1)
            {
                res=GES_LEVEL1_APPROACH;
                //*pindex=gs.wave[chn].peak_index;
            }
            //峰值是第一个数据
            else if (gs.wave[chn].peak_index==0)
            {
                res=GES_LEVEL1_LEAVE;
                //*pindex=gs.wave[chn].peak_index;
            }
            //峰值在中间
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

//对某一帧进行手势分析
//返回：         在此帧中具有的手势类型
uint8_t Ges_Analysis(void)
{
    uint8_t i,basic_ges,res,chn,hold_flag;
    uint16_t index;
    
    res=GES_NULL;
    chn=0;
    hold_flag=0;

#if(!CALIB_OBO)
    //标准化手势数组
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
            LOG_DEBUG("                                                 tick: %lld, CHN%d ges: approach!\r\n",gs.ges_record[i].add_time,i);
#endif
        }
        else if (basic_ges==GES_LEVEL1_LEAVE)
        {
            gs.ges_record[i].ges=basic_ges;
            //gs.ges_record[i].add_time=gs.frame_start_tick+(index*SAMPLE_FREQUENCY_DEF*GES_SAMPLE_AVE_NUM);
            chn|=(1<<i);
//          res=1;
#if (LOG_ENABLE)
            LOG_DEBUG("                                                 tick: %lld, CHN%d ges: leave!\r\n",gs.ges_record[i].add_time,i);
#endif
        }
        else if (basic_ges==GES_LEVEL1_FULL)
        {
            gs.ges_record[i].ges=basic_ges;
            //gs.ges_record[i].add_time=gs.frame_start_tick+(index*SAMPLE_FREQUENCY_DEF*GES_SAMPLE_AVE_NUM);
            chn|=(1<<i);
//          res=1;
#if (LOG_ENABLE)
            LOG_DEBUG("                                                 tick: %lld, CHN%d ges: full!\r\n",gs.ges_record[i].add_time,i);
#endif
        }
        else if (basic_ges==GES_LEVEL1_HOLD)
        {
            gs.ges_record[i].ges=basic_ges;
            //gs.ges_record[i].add_time=gs.frame_start_tick+(index*SAMPLE_FREQUENCY_DEF*GES_SAMPLE_AVE_NUM);
            chn|=(1<<i);
#if (LOG_ENABLE)
            LOG_DEBUG("                                                 tick: %lld, CHN%d ges: hold!\r\n",gs.ges_record[i].add_time,i);
#endif
#if(HOLD_GES==1)
            hold_flag++;
#endif
//          res=GES_LEVEL2_HOLD;
//          return res;
        }
    }
    
    //有新的手势
    if (chn)
    {
#if(HOLD_GES==1)
        if (hold_flag)
        {
            res=GES_LEVEL2_HOLD;
            return res;
        }
#endif
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
            //重置last_ges_record
            gs.last_ges_record[GES_CHN_UP]=gs.ges_record[GES_CHN_UP];
        if (chn&(1<<GES_CHN_DOWN_LEFT))
            //重置last_ges_record
            gs.last_ges_record[GES_CHN_DOWN_LEFT]=gs.ges_record[GES_CHN_DOWN_LEFT];
        if (chn&(1<<GES_CHN_DOWN_RIGHT))
            //重置last_ges_record
            gs.last_ges_record[GES_CHN_DOWN_RIGHT]=gs.ges_record[GES_CHN_DOWN_RIGHT];
        
        uint8_t left_up,left_right,right_up;
        //left_up=Ges_Wave_Lead(GES_CHN_DOWN_LEFT,GES_CHN_UP);
        left_right=Ges_Wave_Lead(GES_CHN_DOWN_LEFT,GES_CHN_DOWN_RIGHT);
        //right_up=Ges_Wave_Lead(GES_CHN_DOWN_RIGHT,GES_CHN_UP);
#if 0
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
#endif
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

//对某一帧进行手势分析
//返回：         在此帧中具有的手势类型
uint8_t Ges_DynamicAnalysis(void)
{
    uint8_t i,basic_ges,res,chn=0,hold_flag,full_flag;
    uint16_t index;
    
    res=GES_NULL;
    chn=0;
    hold_flag=0;
    full_flag=0;

    if(gs.ges_dynamic_sample_size<3)
        return res;

    //标准化手势数组
    Ges_DynamicNormalize();

    for (i=0;i<GES_CHN_NUM;i++)
    {
//      basic_ges=Ges_Peak_Find(gs.ges_sample_array[i],i,&index);
//      basic_ges=Ges_Edge_Find(gs.ges_sample_array[i],i,&index);
        basic_ges=Ges_Wave_DynamicSearch(i,&index);
        if (basic_ges==GES_LEVEL1_APPROACH)
        {
            gs.ges_record[i].ges=basic_ges;
            chn|=(1<<i);
#if (LOG_ENABLE)
            LOG_DEBUG("                                                 tick: %lld, CHN%d ges: approach!\r\n",gs.ges_record[i].add_time,i);
#endif
        }
        else if (basic_ges==GES_LEVEL1_LEAVE)
        {
            gs.ges_record[i].ges=basic_ges;
            chn|=(1<<i);
#if (LOG_ENABLE)
            LOG_DEBUG("                                                 tick: %lld, CHN%d ges: leave!\r\n",gs.ges_record[i].add_time,i);
#endif
        }
        else if (basic_ges==GES_LEVEL1_FULL)
        {
            gs.ges_record[i].ges=basic_ges;
            chn|=(1<<i);
            full_flag++;
#if (LOG_ENABLE)
            LOG_DEBUG("                                                 tick: %lld, CHN%d ges: full!\r\n",gs.ges_record[i].add_time,i);
#endif
        }
        else if (basic_ges==GES_LEVEL1_HOLD)
        {
            gs.ges_record[i].ges=basic_ges;
            chn|=(1<<i);
#if (LOG_ENABLE)
            LOG_DEBUG("                                                 tick: %lld, CHN%d ges: hold!\r\n",gs.ges_record[i].add_time,i);
#endif
#if(HOLD_GES==1)
            hold_flag++;
#endif
//          res=GES_LEVEL2_HOLD;
//          return res;
        }
    }
    
    //有新的手势
    if (chn==0x3)
    {
        LOG_DEBUG("                           hold %d,full %d,apC %d,leC %d\r\n",hold_flag,full_flag,gs.approach_counter,gs.leave_counter);

#if(HOLD_GES==1)
#if 0//有hold的chn，均判定hold
        if ((hold_flag && !full_flag) || 
            (((gs.approach_counter==0)&&(gs.leave_counter==0))&&
                ((gs.ges_record[GES_CHN_DOWN_LEFT].ges==GES_LEVEL1_APPROACH&&gs.ges_record[GES_CHN_DOWN_RIGHT].ges==GES_LEVEL1_LEAVE)||
                (gs.ges_record[GES_CHN_DOWN_LEFT].ges==GES_LEVEL1_LEAVE&&gs.ges_record[GES_CHN_DOWN_RIGHT].ges==GES_LEVEL1_APPROACH))))
#else
/*
        if ((hold_flag>=2) || 
            (((gs.approach_counter==0)&&(gs.leave_counter==0))&&
            ((hold_flag && !full_flag) || 
                (((gs.ges_record[GES_CHN_DOWN_LEFT].ges==GES_LEVEL1_APPROACH&&
                        gs.ges_record[GES_CHN_DOWN_RIGHT].ges==GES_LEVEL1_LEAVE)||
                    (gs.ges_record[GES_CHN_DOWN_LEFT].ges==GES_LEVEL1_LEAVE&&
                        gs.ges_record[GES_CHN_DOWN_RIGHT].ges==GES_LEVEL1_APPROACH))))))
*/
        if (hold_flag>=2)
#endif
        {
            res=GES_LEVEL2_HOLD;
            return res;
        }
#endif
        //接近和远离判断
        //if ((chn&(1<<GES_CHN_DOWN_LEFT))&&(chn&(1<<GES_CHN_DOWN_RIGHT)))
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
            
            //一个接近，一个远离，去除抖动
            else if(((gs.ges_record[GES_CHN_DOWN_LEFT].ges==GES_LEVEL1_APPROACH)||
                (gs.ges_record[GES_CHN_DOWN_RIGHT].ges==GES_LEVEL1_APPROACH)) && gs.approach_counter)
            {
                res=GES_LEVEL2_APPROACH;
                return res;
            }
            else if(((gs.ges_record[GES_CHN_DOWN_LEFT].ges==GES_LEVEL1_LEAVE)||
                (gs.ges_record[GES_CHN_DOWN_RIGHT].ges==GES_LEVEL1_LEAVE)) && gs.leave_counter)
            {
                res=GES_LEVEL2_LEAVE;
                return res;
            }
            else if(gs.ges_record[GES_CHN_DOWN_LEFT].ges==GES_LEVEL1_HOLD ||
                gs.ges_record[GES_CHN_DOWN_RIGHT].ges==GES_LEVEL1_HOLD)
            {
                res=GES_NULL;
                return res;
            }
                //只要有接近或者远离的手型，就不做左右判断
            else if((gs.ges_record[GES_CHN_DOWN_LEFT].ges==GES_LEVEL1_LEAVE)||
                (gs.ges_record[GES_CHN_DOWN_RIGHT].ges==GES_LEVEL1_LEAVE) ||
                (gs.ges_record[GES_CHN_DOWN_LEFT].ges==GES_LEVEL1_APPROACH) ||
                (gs.ges_record[GES_CHN_DOWN_RIGHT].ges==GES_LEVEL1_APPROACH))
            {
                res=GES_NULL;
                return res;
            }
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
        //left_up=Ges_Wave_Lead(GES_CHN_DOWN_LEFT,GES_CHN_UP);
        left_right=Ges_Wave_Lead(GES_CHN_DOWN_LEFT,GES_CHN_DOWN_RIGHT);
        //right_up=Ges_Wave_Lead(GES_CHN_DOWN_RIGHT,GES_CHN_UP);
#if 0
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
#endif
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
    }

    return res;
}

#if(COMB_GES==1)
void Ges_Comb(uint8_t ges)
{
    LOG_DEBUG("                                                ----Ges_Comb----\r\n");

    uint64_t tick=Systick_Get();

    if(tick>comb_ges.start_ges_time+MULTIGES_DISTANCE_MIN)
    {
        LOG_DEBUG("                                               combination geste tick time lease %d ms, reset.\r\n",MULTIGES_DISTANCE_MIN);
        memset(&comb_ges, 0, sizeof(gs_comb_struct));
    }

    if(comb_ges.gesid==0)//the first ges of combination
        comb_ges.start_ges_time = tick;


#if 0
    if(comb_ges.gesid<3)
    {
        comb_ges.ges[comb_ges.gesid] = ges;
    }

    LOG_DEBUG("                                               ----gesid=%d %x %x %x----\r\n",comb_ges.gesid,comb_ges.ges[0],comb_ges.ges[1],comb_ges.ges[2]);
    comb_ges.gesid++;

    if(comb_ges.ges[0]==GES_LEVEL2_LEFT&&
        comb_ges.ges[1]==GES_LEVEL2_RIGHT&&
        comb_ges.ges[2]==GES_LEVEL2_LEFT)
    {
        LOG_DEBUG("                                               ----left----\r\n");
        memset(&comb_ges, 0, sizeof(gs_comb_struct));
        HAL_GPIO_WritePin(LED_LEFT_GPIO_Port, LED_LEFT_Pin, GPIO_PIN_RESET);
        led_counter=LED_FLASH_DELAY;
    }
    if(comb_ges.ges[0]==GES_LEVEL2_RIGHT&&
        comb_ges.ges[1]==GES_LEVEL2_LEFT&&
        comb_ges.ges[2]==GES_LEVEL2_RIGHT)
    {
        LOG_DEBUG("                                               ----right----\r\n");
        memset(&comb_ges, 0, sizeof(gs_comb_struct));
        HAL_GPIO_WritePin(LED_RIGHT_GPIO_Port, LED_RIGHT_Pin, GPIO_PIN_RESET);
        led_counter=LED_FLASH_DELAY;
    }
#else
    if(comb_ges.gesid<2)
    {
        comb_ges.ges[comb_ges.gesid] = ges;
    }

    LOG_DEBUG("                                               ----gesid=%d %x %x ----\r\n",comb_ges.gesid,comb_ges.ges[0],comb_ges.ges[1]);
    comb_ges.gesid++;

    if(comb_ges.ges[0]==GES_LEVEL2_LEFT&&
        comb_ges.ges[1]==GES_LEVEL2_RIGHT)
    {
        LOG_DEBUG("                                               ----left----\r\n");
        memset(&comb_ges, 0, sizeof(gs_comb_struct));
        HAL_GPIO_WritePin(LED_LEFT_GPIO_Port, LED_LEFT_Pin, GPIO_PIN_RESET);
        led_counter=LED_FLASH_DELAY;
    }
    else if(comb_ges.ges[0]==GES_LEVEL2_RIGHT&&
        comb_ges.ges[1]==GES_LEVEL2_LEFT)
    {
        LOG_DEBUG("                                               ----right----\r\n");
        memset(&comb_ges, 0, sizeof(gs_comb_struct));
        HAL_GPIO_WritePin(LED_RIGHT_GPIO_Port, LED_RIGHT_Pin, GPIO_PIN_RESET);
        led_counter=LED_FLASH_DELAY;
    }
    else if(comb_ges.ges[0]==GES_LEVEL2_LEFT&&
        comb_ges.ges[1]==GES_LEVEL2_LEFT)
    {
        LOG_DEBUG("                                               ----left left----\r\n");
        memset(&comb_ges, 0, sizeof(gs_comb_struct));
        HAL_GPIO_WritePin(LED_LEFT_GPIO_Port, LED_LEFT_Pin, GPIO_PIN_RESET);
        led_counter=LED_FLASH_DELAY*5;
    }
    else if(comb_ges.ges[0]==GES_LEVEL2_RIGHT&&
        comb_ges.ges[1]==GES_LEVEL2_RIGHT)
    {
        LOG_DEBUG("                                               ----right right----\r\n");
        memset(&comb_ges, 0, sizeof(gs_comb_struct));
        HAL_GPIO_WritePin(LED_RIGHT_GPIO_Port, LED_RIGHT_Pin, GPIO_PIN_RESET);
        led_counter=LED_FLASH_DELAY*5;
    }
        
#endif
}
#endif

#if(!CALIB_OBO)
//手势基准定标，采用平均值定标
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
            LOG_DEBUG("                                       CHN %d-> base: %d %d\r\n",i,gs.sample_base_last[i],gs.sample_base[i]);
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

//归一化采样数组
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
    //for (j=0;j<gs.sample_size;j++)
    {
        //LOG_DEBUG(" %5d %5d %5d\r\n",
    //        gs.ges_sample_array[1][j],gs.ges_sample_array[2][j],gs.ges_sample_array[0][j]);
    }
    LOG_DEBUG("                                         [%5d %5d %5d]sample[%d]\r\n",gs.sample_base_last[1],gs.sample_base_last[2],gs.sample_base_last[0],gs.sample_size);
#endif
}
#endif


//归一化采样数组
void Ges_DynamicNormalize(void)
{
    uint8_t i,j;
    for (i=0;i<GES_CHN_NUM;i++)
    {
        for (j=0;j<gs.ges_dynamic_sample_size;j++)
        {
        
            if (gs.ges_dynamic_sample_array[i][j]>=gs.sample_base_last[i]) {
                gs.ges_dynamic_sample_array[i][j]=gs.ges_dynamic_sample_array[i][j]-gs.sample_base_last[i];
            }
            else {
                gs.ges_dynamic_sample_array[i][j]=0;
                //gs.ges_sample_array[i][j]=gs.sample_base_last[i]-gs.ges_sample_array[i][j];
            }
        }
    }

#if (LOG_ENABLE)
    for (j=0;j<gs.ges_dynamic_sample_size;j++)
    {
        LOG_DEBUG("                                            ges[%d]:[ %5d  %5d]\r\n",j,
            gs.ges_dynamic_sample_array[1][j],gs.ges_dynamic_sample_array[0][j]);
    }
    LOG_DEBUG("                                            [%5d  %5d] DynamicNormalize sample[%d]\r\n",
        gs.sample_base_last[1],gs.sample_base_last[0],gs.ges_dynamic_sample_size);
#endif
}

#if(CALIB_OBO)
//归一化采样数组
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

//手势基准定标，采用实时平均值
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
#if 0//(LOG_ENABLE)
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
            //打印去除基准值的归一化数组
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

//主任务
void App_Task(void)
#if 0
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
        //一次数组填充完毕
        if (res==1)
        {
//#if (LOG_ENABLE)
#if(CALIB_OBO)
            LOG_DEBUG("sample[%d]\r\n",gs.sample_size);
#endif
//#endif
            //第一次采样数据作为定标用
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
            LOG_DEBUG("sample[%d] %x[13L 14R 15H 17A 18LE]\r\n",gs.sample_size,gs.last_ges);
#endif
            res=Ges_Analysis();
            if (res)//else return to 10 samples
            {
                //当前是short采样，改变sample_size继续采样
                if (gs.sample_size==GES_SAMPLE_ARRAY_NUM_S)
                {
                    gs.sample_size=GES_SAMPLE_ARRAY_NUM_L;
                    gs.sample_size_raw=GES_SAMPLE_RAW_ARRAY_NUM_L;
                    gs.ges_sample_raw_cur=GES_SAMPLE_ARRAY_NUM_S;
                    return;
                }

                uint64_t tick=Systick_Get();
                
                //上次是HOLD手势,此次不是HOLD且不为空，避免误触发,但是hold之后，要允许leave和approach
                if ((gs.last_ges==GES_LEVEL2_HOLD)&&(res!=GES_LEVEL2_HOLD)&&
                    (res!=GES_LEVEL2_APPROACH)&&(res!=GES_LEVEL2_LEAVE))
                {
                    LOG_DEBUG("######IGNORE [%x] after GES_LEVEL2_HOLD#######\r\n",res);
                    gs.last_ges=GES_NULL;
                    gs.last_ges_time=tick;
                    return;
                }

                //上次是APPROACH or LEAVE手势,此次是LEFT or RIGHT，避免误触发
                if (((gs.last_ges==GES_LEVEL2_APPROACH) || (gs.last_ges==GES_LEVEL2_LEAVE)) && 
                    ((res==GES_LEVEL2_LEFT)||(res==GES_LEVEL2_RIGHT)))
                {
                    LOG_DEBUG("######IGNORE [%x] after [%s]#######\r\n",res,(gs.last_ges==GES_LEVEL2_APPROACH)?"APPROACH":"LEAVE");
                    gs.last_ges=GES_NULL;
                    gs.last_ges_time=tick;
                    return;
                }

                //有效手势更新
                if (tick>gs.last_ges_time+GES_DISTANCE_MIN)
                {
                    //if(!gs.hold_saveself)
                        gs.last_ges=res;
                    gs.last_ges_time=tick;
                }
                else//无效手势，间隔过短
                {
                    #if 0//实现连续手势，取消间隔700ms手势过滤
                    //连续触发接近或者离开手势
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
                    #else
                        gs.last_ges=res;
                        gs.last_ges_time=tick;
                        if(!(gs.last_ges == GES_LEVEL2_HOLD && res == GES_LEVEL2_LEAVE))
                        {
                            LOG_DEBUG("                                                 tick time lease %d ms\r\n",GES_DISTANCE_MIN);
                        //    return;
                        }
                    #endif
                }

                if (res==GES_LEVEL2_UP)
                {
                    HAL_GPIO_WritePin(LED_UP_GPIO_Port, LED_UP_Pin, GPIO_PIN_RESET);
                    led_counter=LED_FLASH_DELAY;
                    LOG_DEBUG("                                                 Ges: UP!\r\n");
                    Ges_Log();
                    //开SW2
                    //HAL_GPIO_WritePin(SW2_GPIO_Port, SW2_Pin, GPIO_PIN_SET);
                }
                else if (res==GES_LEVEL2_DOWN)
                {
                    HAL_GPIO_WritePin(LED_DOWN_GPIO_Port, LED_DOWN_Pin, GPIO_PIN_RESET);
                    led_counter=LED_FLASH_DELAY;
                    LOG_DEBUG("                                                 Ges: DOWN!\r\n");
                    Ges_Log();
                    //关SW2
                    //HAL_GPIO_WritePin(SW2_GPIO_Port, SW2_Pin, GPIO_PIN_RESET);
                }
                else if (res==GES_LEVEL2_LEFT)
                {
#if(COMB_GES==0)
                    HAL_GPIO_WritePin(LED_LEFT_GPIO_Port, LED_LEFT_Pin, GPIO_PIN_RESET);
                    led_counter=LED_FLASH_DELAY;
#endif
                    LOG_DEBUG("                                                 Ges: LEFT!\r\n");
#if(COMB_GES==1)
                    Ges_Comb(GES_LEVEL2_LEFT);
#endif
                    Ges_Log();
                    //关SW1
                    //HAL_GPIO_WritePin(SW1_GPIO_Port, SW1_Pin, GPIO_PIN_RESET);
                }
                else if (res==GES_LEVEL2_RIGHT)
                {
#if(COMB_GES==0)
                    HAL_GPIO_WritePin(LED_RIGHT_GPIO_Port, LED_RIGHT_Pin, GPIO_PIN_RESET);
                    led_counter=LED_FLASH_DELAY;
#endif
                    LOG_DEBUG("                                                 Ges: RIGHT!\r\n");
#if(COMB_GES==1)
                    Ges_Comb(GES_LEVEL2_RIGHT);
#endif
                    Ges_Log();
                    //开SW1
                    //HAL_GPIO_WritePin(SW1_GPIO_Port, SW1_Pin, GPIO_PIN_SET);
                }
#if(HOLD_GES==1)
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
#endif
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
                //当前是long采样，改变sample_size继续采样
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
#else
{
    uint16_t ps[3];
    uint8_t res;
    //static uint8_t calib_num=0;
    static uint8_t firstboot=true;

    memset(ps,0,sizeof(ps));

    getSensorDataByHostout(ps);

    if(!ps[0]&&!ps[1]&&!ps[2])
    {
        LOG_DEBUG(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\r\n");
        return;
    }

    res=Ges_Add_Sample(ps);
    //一次数组填充完毕
    if (res==1)
    {

        if (gs.hold_flag && 
            (gs.frame_start_tick>gs.last_ges_time+GES_TREND_CLEARUP_MIN))
        {
            LOG_DEBUG(">>>>>>>>>>>>>>>>>init_counter>>>>>>>>>>>>>>\r\n");
            gs.last_ges = GES_NULL;
            init_counter();
        }

        //第一次采样数据作为定标用
        //if (!calib_num)
        //if(GES_SAMPLE_ARRAY_NUM_S==gs.sample_size)
        if(0==gs.hold_flag && gs.ges_dynamic_start==false)
        {
#if (LOG_ENABLE)
            LOG_DEBUG("                                       Ges_Calib sample[%d]\r\n",gs.sample_size);
#endif
            Ges_Calib(firstboot);
            firstboot = false;
            //calib_num++;
            //return;
        }
    }

#if (!LOG_ENABLE)
    //LOG_DEBUG("sample[%d] %x[13L 14R 15H 17A 18LE]\r\n",gs.sample_size,gs.last_ges);
#endif

    if(firstboot)
        return;

    res = Ges_Detect_Sample(ps);
    if(!res)//当前手势过程不完整，继续采样
        return;

    res=Ges_DynamicAnalysis();
    if (res)//else return to 10 samples
    {
        uint64_t tick=Systick_Get();

        //上次是HOLD手势,此次不是HOLD且不为空，避免误触发,但是hold之后，要允许leave和approach
        if ((gs.last_ges==GES_LEVEL2_HOLD)&&(res!=GES_LEVEL2_HOLD)&&
            (res!=GES_LEVEL2_APPROACH)&&(res!=GES_LEVEL2_LEAVE))
        {
            LOG_DEBUG("######IGNORE [0x%x] after GES_LEVEL2_HOLD#######\r\n",res);
            gs.last_ges=GES_NULL;
            gs.last_ges_time=tick;
            init_counter();
            return;
        }

        //上次是APPROACH or LEAVE手势,此次是LEFT or RIGHT，避免误触发
        if (((gs.last_ges==GES_LEVEL2_APPROACH) || (gs.last_ges==GES_LEVEL2_LEAVE)) && 
            ((res==GES_LEVEL2_LEFT)||(res==GES_LEVEL2_RIGHT)))
        {
            LOG_DEBUG("######IGNORE [0x%x] after [%s]#######\r\n",res,(gs.last_ges==GES_LEVEL2_APPROACH)?"APPROACH":"LEAVE");
            gs.last_ges=GES_NULL;
            gs.last_ges_time=tick;
            init_counter();
            return;
        }

        //上次是APPROACH手势,此次是leave相反，避免接近过程结束的误触发
        if ((gs.last_ges==GES_LEVEL2_APPROACH) && (res==GES_LEVEL2_LEAVE))
        {
            LOG_DEBUG("######IGNORE [LEAVE] after [APPROACH]#######\r\n");
            gs.last_ges=GES_NULL;
            gs.last_ges_time=tick;
            init_counter();
            return;
        }

        //有效手势更新
        if (tick>gs.last_ges_time+GES_DISTANCE_MIN)
        {
            //if(!gs.hold_saveself)
                gs.last_ges=res;
            gs.last_ges_time=tick;
        }
        else//无效手势，间隔过短
        {
            #if 0//实现连续手势，取消间隔700ms手势过滤
            //连续触发接近或者离开手势
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
            #else
                gs.last_ges=res;
                gs.last_ges_time=tick;
                if(!(gs.last_ges == GES_LEVEL2_HOLD && res == GES_LEVEL2_LEAVE))
                {
                    LOG_DEBUG("                                                 tick time lease %d ms, and go on.\r\n",GES_DISTANCE_MIN);
                //    return;
                }
            #endif
        }

        if (res==GES_LEVEL2_UP)
        {
            HAL_GPIO_WritePin(LED_UP_GPIO_Port, LED_UP_Pin, GPIO_PIN_RESET);
            led_counter=LED_FLASH_DELAY;
            LOG_DEBUG("                                                 Ges: UP!\r\n");
            Ges_Log();
        }
        else if (res==GES_LEVEL2_DOWN)
        {
            HAL_GPIO_WritePin(LED_DOWN_GPIO_Port, LED_DOWN_Pin, GPIO_PIN_RESET);
            led_counter=LED_FLASH_DELAY;
            LOG_DEBUG("                                                 Ges: DOWN!\r\n");
            Ges_Log();
        }
        else if (res==GES_LEVEL2_LEFT)
        {
#if(COMB_GES==0)
            HAL_GPIO_WritePin(LED_LEFT_GPIO_Port, LED_LEFT_Pin, GPIO_PIN_RESET);
            led_counter=LED_FLASH_DELAY;
#endif
            LOG_DEBUG("                                                 Ges: LEFT!\r\n");
#if(COMB_GES==1)
            Ges_Comb(GES_LEVEL2_LEFT);
#endif
            Ges_Log();
        }
        else if (res==GES_LEVEL2_RIGHT)
        {
#if(COMB_GES==0)
            HAL_GPIO_WritePin(LED_RIGHT_GPIO_Port, LED_RIGHT_Pin, GPIO_PIN_RESET);
            led_counter=LED_FLASH_DELAY;
#endif
            LOG_DEBUG("                                                 Ges: RIGHT!\r\n");
#if(COMB_GES==1)
            Ges_Comb(GES_LEVEL2_RIGHT);
#endif
            Ges_Log();
        }
#if(HOLD_GES==1)
        else if (res==GES_LEVEL2_HOLD)
        {
            LOG_DEBUG("                         gs.hold_counter:%d,pwm:%d\r\n",gs.hold_counter,brightness/10);
            gs.hold_counter++;
            if (gs.hold_counter==SAMPLE_HOLD_COUNT)
            {

                gs.hold_flag = Ges_Hold_Height_Valid_Judge();
                if(gs.hold_flag)
                {
                    HAL_GPIO_WritePin(LED_LEFT_GPIO_Port, LED_LEFT_Pin, GPIO_PIN_RESET);
                    HAL_GPIO_WritePin(LED_RIGHT_GPIO_Port, LED_RIGHT_Pin, GPIO_PIN_RESET);
                    HAL_GPIO_WritePin(LED_UP_GPIO_Port, LED_UP_Pin, GPIO_PIN_RESET);
                    HAL_GPIO_WritePin(LED_DOWN_GPIO_Port, LED_DOWN_Pin, GPIO_PIN_RESET);
                    led_counter=LED_HOLD_FLASH_DELAY;
                    //Ges_Hold_Height_Judge();
                    LOG_DEBUG("                         Ges: HOLD!\r\n");
                    Ges_Log();
                    //gs.hold_flag = 1;
                    gs.new_hold_AorL_size = GES_SAMPLE_RAW_ARRAY_NUM_S;
                }
                else
                {
                    gs.hold_counter -= 1;
                }
            }

            if(gs.hold_flag)
            {
                Ges_Hold_Height_AorL_Judge();
            }

#if 0
            if(gs.hold_counter>4)
            {
            /*
                if (gs.sample_size==GES_SAMPLE_ARRAY_NUM_L)
                {
                    gs.sample_size=GES_SAMPLE_ARRAY_NUM_S;
                    gs.sample_size_raw=GES_SAMPLE_RAW_ARRAY_NUM_S;
                }*/
                gs.last_ges = GES_NULL;
                gs.hold_counter = 0;
                //gs.hold_flag = 0;
            }
#endif
        }
#endif
/*
        else if (res==GES_LEVEL2_DCLK)
        {
            HAL_GPIO_WritePin(LED_DCLK_GPIO_Port, LED_DCLK_Pin, GPIO_PIN_RESET);
            led_counter=LED_FLASH_DELAY;
            LOG_DEBUG("                         Ges: DCLK!\r\n");
            Ges_Log();
        }
*/
        else if (res==GES_LEVEL2_APPROACH)
        {
            //if(gs.approach_counter)
            {
                LOG_DEBUG("                         Ges: APPROACH!\r\n");
                HAL_GPIO_WritePin(LED_UP_GPIO_Port, LED_UP_Pin, GPIO_PIN_RESET);
                led_counter=50;
                Ges_Log();
            }
            gs.approach_counter++;
            gs.leave_counter = 0;
        }
        else if (res==GES_LEVEL2_LEAVE)
        {
            LOG_DEBUG("                         Ges: LEAVE!\r\n");
            HAL_GPIO_WritePin(LED_DOWN_GPIO_Port, LED_DOWN_Pin, GPIO_PIN_RESET);
            led_counter=50;
            Ges_Log();
            gs.leave_counter++;
            gs.approach_counter=0;
        }
        /*
        //准备接受下一手势动作
        if (gs.sample_size==GES_SAMPLE_ARRAY_NUM_L)
        {
            gs.sample_size=GES_SAMPLE_ARRAY_NUM_S;
            gs.sample_size_raw=GES_SAMPLE_RAW_ARRAY_NUM_S;
        }*/

        gs.analy_fail_counter = 0;
    }
    else
    {
        LOG_DEBUG("Ges Analysis fail.\r\n");
        /*
        //当前是long采样，改变sample_size继续采样
        if (gs.sample_size==GES_SAMPLE_ARRAY_NUM_L)
        {
            gs.sample_size=GES_SAMPLE_ARRAY_NUM_S;
            gs.sample_size_raw=GES_SAMPLE_RAW_ARRAY_NUM_S;
            return;
        }*/
        gs.last_ges = GES_NULL;
        init_counter();
        gs.analy_fail_counter++;
        if(gs.analy_fail_counter>5)
        {
            Ges_Calib(firstboot);
            gs.analy_fail_counter = 0;
        }
    }
#if (LOG_ENABLE)
    LOG_DEBUG("\r\n\n");
#endif
    //}
}
#endif

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
