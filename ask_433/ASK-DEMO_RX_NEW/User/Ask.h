#ifndef     __ASK_H_
#define     __ASK_H_

#include "All_Includes.h"

#define max_time_h       60      //�������������ʱ�� data*TCC_time
#define min_time_h       16      //��������С����ʱ��
#define max_time_l       14      //խ�����������ʱ��
#define min_time_l       4      //խ������С����ʱ��

//#define max_time_h       40      //�������������ʱ�� data*TCC_time
//#define min_time_h       18      //��������С����ʱ��
//#define max_time_l       15      //խ�����������ʱ��
//#define min_time_l       6      //խ������С����ʱ��

#if 0 //ŷ����͸��
#define ASK_SEND_LEN        4
#else
#define ASK_SEND_LEN        3
#define READ_REALTIME
#endif
#define RECV_BIT_NUMBER     (ASK_SEND_LEN*8)       //�趨���յ�λ��
#define inport              PD_IDR_IDR3
//#define       ASK             PD_IDR_IDR4

void Ask_Init();
void Ask_process();
void Recieve();
void RecieveError();
void ProcessRecv();
void Learn_Sender();
void ReadSelfAddr();
void Dele_Sender();
void ProcessOut();
void Write_Coder();

#endif
