
#ifndef __EPLOS_H
#define __EPLOS_H

#define I2C_ADDR01                  0x20  //A1-3  pulldown
#define I2C_ADDR23                  0x21

#define EPLOS_I2C_ADD01             0x00  //010 000 0B=0x20
#define EPLOS_CFG01                 0x01
#define EPLOS_STATUS01              0x02
#define EPLOS_I2C_MON01             0x03 //checksum 初始化
                                        /*
                                        0b:校验和字节在同一帧中的寄存器字节内容之后不被传输(                        如果主请求，下一个寄存器内容将被传输)               
                                        1b:校验和字节是在同一帧中的寄存器字节内容之后传输的(除非主机首先停止传输)
                                        */
#define EPLOS_I2C_CS01              0x04
#define EPLOS_SCI_MON01             0x05
#define EPLOS_DIAG_OUT_SEL01        0x06
#define EPLOS_DIAG_OUT_DAT0         0x07
#define EPLOS_DIAG_OUT_DAT1         0x08
#define EPLOS_DIAG_TM_DAT0          0x09
#define EPLOS_DIAG_TM_DAT1          0x0A
#define EPLOS_DIAG_TM_CFG01         0x0B
#define EPLOS_DIAG_VCP_DAT0         0x0C
#define EPLOS_DIAG_VCP_DAT1         0x0D
#define EPLOS_DIAG_VCP_CFG0         0x0E
#define EPLOS_DIAG_VCP_CFG1         0x0F

#define EPLOS_I2C_ADD23             0x00  //010 000 1B=0x21
#define EPLOS_CFG23                 0x01
#define EPLOS_STATUS23              0x02
#define EPLOS_I2C_MON23             0x03
#define EPLOS_I2C_CS23              0x04
#define EPLOS_SCI_MON23             0x05
#define EPLOS_DIAG_OUT_SEL23        0x06
#define EPLOS_DIAG_OUT_DAT2         0x07
#define EPLOS_DIAG_OUT_DAT3         0x08
#define EPLOS_DIAG_TM_DAT2          0x09
#define EPLOS_DIAG_TM_DAT3          0x0A
#define EPLOS_DIAG_TM_CFG23         0x0B
#define EPLOS_DIAG_VCP_DAT2         0x0C
#define EPLOS_DIAG_VCP_DAT3         0x0D
#define EPLOS_DIAG_VCP_CFG2         0x0E
#define EPLOS_DIAG_VCP_CFG3         0x0F


#define QT_PIXELS                   256

#endif

