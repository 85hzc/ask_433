
#ifndef __MCUGPIO_H
#define __MCUGPIO_H


#define SCAN_LINE                                   16
#define SECS                                        3

void McuGpio_X(void);
void McuGpio_Sink(void);
void cycleScan_X(uint8_t type);
void cycleScan_Sink(void);

typedef enum
{
    ROW_CS = 0,
    COLUMN_CS,
} SIGNAL_PORT_E;

typedef enum
{
    PULLUP = 0,
    PULLDOWN,
} PULL_STATUS_E;

#endif

