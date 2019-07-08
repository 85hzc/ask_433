
#ifndef __MBI5153_H
#define __MBI5153_H

#define TWO_TIMER_PULSE                             0
#define TIMER                                       0//(TIM1 1£»TIM3 3)

#define SEPR_SECTION                                1
#define SECTION_PER_SCAN                            1


#define GCLKNUM                                     5


#define MBI5153_SIZE                                1
#define SCAN_LINE                                   8

#define SCAN_LINE_1                                 0x0
#define SCAN_LINE_2                                 0x1
#define SCAN_LINE_4                                 0x3
#define SCAN_LINE_8                                 0x7
#define SCAN_LINE_16                                0xf
#define SCAN_LINE_32                                0x1f

#define SECS                                        8

#define SCAN_SUPPORT


void display_X(void);
void display_Sink(void);
void cycleScan_X(uint8_t type);
void cycleScan_Sink(void);


#endif

