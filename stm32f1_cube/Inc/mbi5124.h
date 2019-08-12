
#ifndef __MBI5124_H
#define __MBI5124_H


#define MBI5124_SIZE                                1
#define SCAN_LINE                                   16

#define SECS                                        4


void MBI5124_X(void);
void MBI5124Sink(void);
void cycleScan_X(uint8_t type);
void cycleScan_Sink(void);


#endif

