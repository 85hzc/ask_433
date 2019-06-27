#ifndef __APP_H
#define __APP_H

#include "main.h"

#define LED_FLASH_DELAY             300             //LEDָʾ�ĵ���״̬����ʱ�䣬300ms
#define LED_HOLD_FLASH_DELAY        100             //LEDָʾ�ĵ���״̬����ʱ�䣬100ms

#define SAMPLE_FREQUENCY_DEF        10              //���Ʋ���Ƶ�ʣ�10ms
#define SAMPLE_DOWN_LIMIT_DEF       0x000A          //������ֵ����
#define SAMPLE_UP_LIMIT_DEF         0x0020          //����������ֵ
#define SAMPLE_HOLD_LIMIT_DEF       0x0020//0x0040  //hold�Ĵ�����ֵ
#define SAMPLE_HOLD_END_SHIFT       GES_SAMPLE_ARRAY_NUM_S  //�Ӳ�������ĩβ��10����Ϊ����׼
#define SAMPLE_HOLD_COUNT           2               //���3��HOLD��count�ٴη���һ��HOLD
#define SAMPLE_HOLD_LIST_NUM        10              //hold��������Ԫ�ظ���
//#define SAMPLE_HOLD_START

#define GES_CHN_NUM                 2               //���Ʋ�����ͨ������3
#define GES_CHN_UP                  2               //��ͨ��
#define GES_CHN_DOWN_LEFT           1               //����ͨ��
#define GES_CHN_DOWN_RIGHT          0               //����ͨ��
#define GES_CHN_INVALID             0xFF            //��Чͨ��

#define GES_SAMPLE_AVE_NUM          1               //����ƽ�������ݴ�С��1
#define GES_SAMPLE_ARRAY_NUM_S      8               //���Ʋ����Ķ������С��8
#define GES_SAMPLE_ARRAY_NUM_L      32              //���Ʋ����ĳ������С��24
#define GES_SAMPLE_ARRAY_FULL       (GES_SAMPLE_ARRAY_NUM_L)//���Ʋ����ĳ������С��24

#define GES_SAMPLE_RAW_ARRAY_NUM_S  (GES_SAMPLE_ARRAY_NUM_S*GES_SAMPLE_AVE_NUM)
#define GES_SAMPLE_RAW_ARRAY_NUM_L  (GES_SAMPLE_ARRAY_NUM_L*GES_SAMPLE_AVE_NUM)
                                                    //ʵ��һ�η����������ĵ���Ϊ��GES_SAMPLE_RAW_ARRAY_NUM=GES_SAMPLE_ARRAY_NUM*GES_SAMPLE_AVE_NUM    
                                                    //���Ʋ�����ԭʼ����
                                                    
#define GES_TIME_DIF_MIN            10              //����ʱ������Сֵ��10ms
#define GES_TIME_DIF_MAX            1000            //����ʱ�������ֵ��1000ms
#define GES_DISTANCE_MIN            700             //�������Ƶ���Сֵ��700ms
#define MULTIGES_DISTANCE_MIN       2000            //����������Ƶ���Сֵ��2.5s

#define GES_TREND_CLEARUP_MIN       1000            //�������Ƶ���Сֵ��1000ms

#define POSITIVE                    1               //�������ݼ��ֵ���ڻ�׼ֵ
#define NEGATIVE                    2               //�������ݼ��ֵС�ڻ�׼ֵ

//��������
#define GES_NULL                    0x00            //��
#define GES_LEVEL1_APPROACH         0x01            //����
#define GES_LEVEL1_LEAVE            0x02            //�뿪
#define GES_LEVEL1_FULL             0x03            //���������Ŀ���->�뿪����
#define GES_LEVEL1_HOLD             0x04            //hold
#define GES_LEVEL2_UP               0x11            //��
#define GES_LEVEL2_DOWN             0x12            //��
#define GES_LEVEL2_LEFT             0x13            //��
#define GES_LEVEL2_RIGHT            0x14            //��
#define GES_LEVEL2_HOLD             0x15            //HOLDģʽ
#define GES_LEVEL2_DCLK             0x16            //˫��ģʽ
#define GES_LEVEL2_APPROACH         0x17            //�ӽ�
#define GES_LEVEL2_LEAVE            0x18            //Զ��


#define GES_INVALID_INDEX           0xFFFF          //��Ч��������ֵ

#define CALIB_OBO                   0
#define SECTION_NUM                 3

typedef struct
{
    uint16_t rise_index;            //�����������������մ��ڴ������޵ĵ�
    uint16_t rise_value;            //������ֵ
    uint16_t peak_index;            //������������
    uint16_t peak_value;            //����ֵ
    uint16_t fall_index;            //�½��������������պõ��������޵�ǰһ��
    uint16_t fall_value;            //�½���ֵ
}wave_struct;

typedef struct
{
    uint8_t ges;                            //��������
    uint64_t add_time;              //����ʱ��
}ges_struct;

typedef struct
{
    //uint8_t sample_start_flag;                                              //������ʼ��־
    //uint8_t sample_ready_flag;                                              //������ɱ�־
    //uint32_t sample_counter;                                                //������ʼ������
    //uint32_t sample_frequency;                                              //����Ƶ��
    uint16_t sample_size;                                                   //��������Ԫ�ظ���
    uint16_t sample_size_raw;                                               //����raw����Ԫ�ظ���
    
    uint16_t sample_base[GES_CHN_NUM];                                      //ÿ·�Ļ�׼
    uint16_t sample_base_last[GES_CHN_NUM];                                 //��һ����ÿ·�Ļ�׼
    uint16_t sample_down_limit[GES_CHN_NUM];                                //ÿ·�Ĵ������ޣ����ڴ�ֵ����ӽ�
    uint16_t sample_up_limit[GES_CHN_NUM];                                  //ÿ·�Ĵ������ޣ����ڴ�ֵ���¼����
    
    uint16_t ges_sample_raw_array[GES_CHN_NUM][GES_SAMPLE_RAW_ARRAY_NUM_L]; //���Ʋ���ԭʼ����
    uint16_t ges_sample_array[GES_CHN_NUM][GES_SAMPLE_ARRAY_NUM_L];         //���Ʋ�������
    uint16_t ges_sample_raw_cur;                                            //����raw���鵱ǰԪ��

    //uint16_t ges_dynamic_sample_raw_array[GES_CHN_NUM][GES_SAMPLE_RAW_ARRAY_NUM_L]; //���Ʋ���ԭʼ����
    uint16_t ges_dynamic_sample_array[GES_CHN_NUM][GES_SAMPLE_ARRAY_NUM_L];         //���Ʋ�������
    uint16_t ges_dynamic_sample_raw_cur;                                            //����raw���鵱ǰԪ��
    uint8_t  ges_dynamic_sample_size;
    uint8_t  ges_dynamic_start;
    
    uint16_t ges_sample_Calib_array[GES_CHN_NUM][GES_SAMPLE_ARRAY_NUM_L];           //���Ʋ�������
    
    uint64_t frame_start_tick;                                              //��ǰ֡��������ʼʱ��
    ges_struct last_ges_record[GES_CHN_NUM];                                //���Ƽ�¼��ÿһ·����һ����������
    ges_struct ges_record[GES_CHN_NUM];                                     //���Ƽ�¼��ÿһ·����һ����������  
//  uint8_t ges_num[GES_CHN_NUM];                                           //�����������ЧԪ��
    
    wave_struct wave[GES_CHN_NUM];
    
    uint8_t last_ges;
    uint64_t last_ges_time;

    uint8_t sample_mode;
    uint16_t hold_height_start;
    uint16_t hold_height_stop;
    uint8_t  new_hold_AorL_size;
    uint8_t approach_counter;
    uint8_t leave_counter;
    uint8_t hold_counter;
    uint8_t hold_flag;
    //uint8_t calib_force;
    uint8_t analy_fail_counter;
    uint16_t hold_list[SAMPLE_HOLD_LIST_NUM];
//  uint16_t led_flash_counter;
}gs_struct;

#if(COMB_GES==1)
typedef struct
{
    uint8_t ges[3];
    uint8_t gesid;
    uint64_t start_ges_time;

    //uint64_t last_ges_time;
}gs_comb_struct;
#endif

void App_Var_Init(void);
void App_Init(void);
void LED_Tick_Check(void);
uint8_t Ges_Add_Sample(uint16_t *ps);
//void Ges_Tick_Check(void);
//void Ges_It_Handle(void);
//uint8_t Ges_Peak_Find(uint16_t *ps,uint8_t chn,uint16_t *pindex);
//uint8_t Ges_Edge_Find(uint16_t *ps,uint8_t chn,uint16_t *pindex);
uint8_t Ges_Wave_Search(uint8_t chn,uint16_t *pindex);
uint8_t Ges_Wave_Lead(uint8_t chn1,uint8_t chn2);
uint8_t Ges_Analysis(void);
void Ges_Calib(uint8_t firstboot);
void Ges_Normalize(void);
void Ges_DynamicNormalize(void);
void Ges_Log(void);
//uint8_t Ges_SampleSize_Switch(void);
void App_Task(void);
void Systick_Inc(void);
uint64_t Systick_Get(void);

#endif  //__APP_H
