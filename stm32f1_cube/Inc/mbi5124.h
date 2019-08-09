
#ifndef __MBI5153_H
#define __MBI5153_H


#define SEPR_SECTION                                1
#define SECTION_PER_SCAN                            1


#define MBI5124_SIZE                                1
#define SCAN_LINE                                   16

#define SCAN_LINE_1                                 0x0
#define SCAN_LINE_2                                 0x1
#define SCAN_LINE_4                                 0x3
#define SCAN_LINE_8                                 0x7
#define SCAN_LINE_16                                0xf
#define SCAN_LINE_32                                0x1f

#define SECS                                        4

#define SCAN_SUPPORT


void MBI5124_X(void);
void MBI5124Sink(void);
void cycleScan_X(uint8_t type);
void cycleScan_Sink(void);


#endif

