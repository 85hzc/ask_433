#ifndef __SILICONSI115X_H__
#define __SILICONSI115X_H__

#include "main.h"
#include "stm32f1xx_hal.h"
//#include "drv.h"


//#define SILAB_log(M, ...) custom_log("SILAB", M, ##__VA_ARGS__)
//#define SILAB_log_trace() custom_log_trace("SILAB")

#define false 0
#define true  1


//I2C / part address Defines:
#define SI1133_I2C_ADDR                  0x55
#define SI1153_I2C_ADDR                  0x53
#define SI1153_REG_HW_ID                 0x01
#define SI1133_REG_HW_ID                 0x01
#define SI1133_HW_ID                     0x03
#define SI1153_REG_PART_ID               0x00
#define SI1153_PART_ID                   0x53
#define SI1153_PROX_HW_ID                0x00
#define SI1153_LR_PROX_HW_ID             0x01
#define SI1153_SUN_PROX_HW_ID            0x02

#define SI1153_READ_FLAG                (1)
#define SI1153_WRITE_FLAG               (0)

#define SILAB_WRITE(s,v,n) \
    HAL_I2C_Mem_Write(&hi2c2, SI1153_I2C_ADDR<<1, s, 1, v, n, 500)
#define SILAB_READ(s,v,n)  \
    HAL_I2C_Mem_Read(&hi2c2, SI1153_I2C_ADDR<<1, s, 1, v, n, 500)


typedef struct Si115xSample
{
    uint8_t     irq_status;
    int32_t     ch0;
    int32_t     ch1;
    int32_t     ch2;
    int32_t     ch3;
} Si115xSample_t;

typedef enum {
    SILAB_SENSOR_OK = 0,
    SILAB_SENSOR_ERROR = 1,
    SILAB_SENSOR_TIMEOUT = 2,
    SILAB_SENSOR_NOT_IMPLEMENTED = 3
} SILAB_SENSOR_StatusTypeDef;

#define SILAB_WHO_AM_I_ADDR		0x01

#define SI115x_REG_PART_ID      0x00
#define SI115x_REG_HW_ID        0x01
#define SI115x_REG_REV_ID       0x02
#define SI115x_REG_HOSTIN0      0x0A
#define SI115x_REG_COMMAND      0x0B
#define SI115x_REG_IRQ_ENABLE   0x0F
#define SI115x_REG_RESPONSE1    0x10
#define SI115x_REG_RESPONSE0    0x11
#define SI115x_REG_IRQ_STATUS   0x12
#define SI115x_REG_HOSTOUT0     0x13
#define SI115x_REG_HOSTOUT1     0x14
#define SI115x_REG_HOSTOUT2     0x15
#define SI115x_REG_HOSTOUT3     0x16
#define SI115x_REG_HOSTOUT4     0x17
#define SI115x_REG_HOSTOUT5     0x18
#define SI115x_REG_HOSTOUT6     0x19
#define SI115x_REG_HOSTOUT7     0x1A
#define SI115x_REG_HOSTOUT8     0x1B
#define SI115x_REG_HOSTOUT9     0x1C
#define SI115x_REG_HOSTOUT10    0x1D
#define SI115x_REG_HOSTOUT11    0x1E
#define SI115x_REG_HOSTOUT12    0x1F
#define SI115x_REG_HOSTOUT13    0x20
#define SI115x_REG_HOSTOUT14    0x21
#define SI115x_REG_HOSTOUT15    0x22
#define SI115x_REG_HOSTOUT16    0x23
#define SI115x_REG_HOSTOUT17    0x24
#define SI115x_REG_HOSTOUT18    0x25
#define SI115x_REG_HOSTOUT19    0x26
#define SI115x_REG_HOSTOUT20    0x27
#define SI115x_REG_HOSTOUT21    0x28
#define SI115x_REG_HOSTOUT22    0x29
#define SI115x_REG_HOSTOUT23    0x2A
#define SI115x_REG_HOSTOUT24    0x2B
#define SI115x_REG_HOSTOUT25    0x2C

/*******************************************************************************
 ************************** Si115x I2C Parameter Offsets ***********************
 ******************************************************************************/
#define SI115x_PARAM_I2C_ADDR          0x00
#define SI115x_PARAM_CH_LIST           0x01
#define SI115x_PARAM_ADCCONFIG0        0x02
#define SI115x_PARAM_ADCSENS0          0x03
#define SI115x_PARAM_ADCPOST0          0x04
#define SI115x_PARAM_MEASCONFIG0       0x05
#define SI115x_PARAM_ADCCONFIG1        0x06
#define SI115x_PARAM_ADCSENS1          0x07
#define SI115x_PARAM_ADCPOST1          0x08
#define SI115x_PARAM_MEASCONFIG1       0x09
#define SI115x_PARAM_ADCCONFIG2        0x0A
#define SI115x_PARAM_ADCSENS2          0x0B
#define SI115x_PARAM_ADCPOST2          0x0C
#define SI115x_PARAM_MEASCONFIG2       0x0D
#define SI115x_PARAM_ADCCONFIG3        0x0E
#define SI115x_PARAM_ADCSENS3          0x0F
#define SI115x_PARAM_ADCPOST3          0x10
#define SI115x_PARAM_MEASCONFIG3       0x11
#define SI115x_PARAM_ADCCONFIG4        0x12
#define SI115x_PARAM_ADCSENS4          0x13
#define SI115x_PARAM_ADCPOST4          0x14
#define SI115x_PARAM_MEASCONFIG4       0x15
#define SI115x_PARAM_ADCCONFIG5        0x16
#define SI115x_PARAM_ADCSENS5          0x17
#define SI115x_PARAM_ADCPOST5          0x18
#define SI115x_PARAM_MEASCONFIG5       0x19
#define SI115x_PARAM_MEASRATE_H        0x1A
#define SI115x_PARAM_MEASRATE_L        0x1B
#define SI115x_PARAM_MEASCOUNT0        0x1C
#define SI115x_PARAM_MEASCOUNT1        0x1D
#define SI115x_PARAM_MEASCOUNT2        0x1E
#define SI115x_PARAM_LED1_A            0x1F
#define SI115x_PARAM_LED1_B            0x20
#define SI115x_PARAM_LED2_A            0x21
#define SI115x_PARAM_LED2_B            0x22
#define SI115x_PARAM_LED3_A            0x23
#define SI115x_PARAM_LED3_B            0x24
#define SI115x_PARAM_THRESHOLD0_H      0x25
#define SI115x_PARAM_THRESHOLD0_L      0x26
#define SI115x_PARAM_THRESHOLD1_H      0x27
#define SI115x_PARAM_THRESHOLD1_L      0x28
#define SI115x_PARAM_THRESHOLD2_H      0x29
#define SI115x_PARAM_THRESHOLD2_L      0x2A
#define SI115x_PARAM_BURST             0x2B

#define SI115x_CMD_NOP                 0x00
#define SI115x_CMD_RESET               0x01
#define SI115x_CMD_NEW_ADDR            0x02
#define SI115x_CMD_FORCE_CH            0x11
#define SI115x_CMD_PAUSE_CH            0x12
#define SI115x_CMD_AUTO_CH             0x13
#define SI115x_CMD_PARAM_SET           0x80
#define SI115x_CMD_PARAM_QUERY         0x40

/*******************************************************************************
 *******    Si115x Register and Parameter Bit Definitions  *********************
 ******************************************************************************/
#define SI115x_RSP0_CHIPSTAT_MASK      0xe0
#define SI115x_RSP0_COUNTER_MASK       0x1f
#define SI115x_RSP0_SLEEP              0x20

//Defines for different parts:
#define SI1133_AA00_UV                   1
#define SI1153_AA00_GESTURE              2 // not supported, no demo for this sensor
#define SI1153_AA09_LONG_RANGE_PROX      3
#define SI1153_AA9X_SUNLIGHT_IMMUNE_PROX 4
#define SI1153_AA00_PROX                 5
#define SI1153_AA00_PROX_ALS             6
#define SI1133_AA00_UV_ALS               7
#define SI11XX_NONE                      0

extern SILAB_SENSOR_StatusTypeDef SILAB_ReadID();

#endif
