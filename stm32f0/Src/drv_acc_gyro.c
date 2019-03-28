/**
 ******************************************************************************
 * @file    drv_i2c_peripheral.c
 * @author  MEMS Application Team
 * @version V1.1
 * @date    10-August-2016
 * @brief   drv_i2c_peripheral source file
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "drv_acc_gyro.h"
#include "drv.h"
#include "string.h"

#define ACC_I2C_ADDR_L          0x31U
#define ACC_I2C_ADDR_H          0x33U

#define ACC_WRITE(s,v,n) \
    HAL_I2C_Mem_Write(&hi2c2, ACC_I2C_ADDR_H, s|0x80, 1, v, n, 1000)
#define ACC_READ(s,v,n)  \
    HAL_I2C_Mem_Read(&hi2c2, ACC_I2C_ADDR_H, s|0x80, 1, v, n, 1000)

#define LIS2DH12_ID             0x33U
#define LIS2DH12_WHO_AM_I              0x0FU
#define LIS2DH12_OUT_X_L              0x28U
#define LIS2DH12_OUT_X_H              0x29U
#define LIS2DH12_OUT_Y_L              0x2AU
#define LIS2DH12_OUT_Y_H              0x2BU
#define LIS2DH12_OUT_Z_L              0x2CU
#define LIS2DH12_OUT_Z_H              0x2DU
#define LIS2DH12_FIFO_CTRL_REG        0x2EU

#define PROPERTY_DISABLE                (0U)
#define PROPERTY_ENABLE                 (1U)

typedef union{
  int16_t i16bit[3];
  uint8_t u8bit[6];
} axis3bit16_t;

#define LIS2DH12_CTRL_REG1             0x20U
typedef struct {
  uint8_t xen               : 1;
  uint8_t yen               : 1;
  uint8_t zen               : 1;
  uint8_t lpen              : 1;
  uint8_t odr               : 4;
} lis2dh12_ctrl_reg1_t;

#define LIS2DH12_CTRL_REG4      0x23U
typedef struct {
  uint8_t sim               : 1;
  uint8_t st                : 2;
  uint8_t hr                : 1;
  uint8_t fs                : 2;
  uint8_t ble               : 1;
  uint8_t bdu               : 1;
} lis2dh12_ctrl_reg4_t;

#define LIS2DH12_STATUS_REG           0x27U
typedef struct {
  uint8_t xda               : 1;
  uint8_t yda               : 1;
  uint8_t zda               : 1;
  uint8_t zyxda             : 1;
  uint8_t xor               : 1;
  uint8_t yor               : 1;
  uint8_t zor               : 1;
  uint8_t zyxor             : 1;
} lis2dh12_status_reg_t;

typedef enum {
  LIS2DH12_POWER_DOWN                      = 0x00,
  LIS2DH12_ODR_1Hz                         = 0x01,
  LIS2DH12_ODR_10Hz                        = 0x02,
  LIS2DH12_ODR_25Hz                        = 0x03,
  LIS2DH12_ODR_50Hz                        = 0x04,
  LIS2DH12_ODR_100Hz                       = 0x05,
  LIS2DH12_ODR_200Hz                       = 0x06,
  LIS2DH12_ODR_400Hz                       = 0x07,
  LIS2DH12_ODR_1kHz620_LP                  = 0x08,
  LIS2DH12_ODR_5kHz376_LP_1kHz344_NM_HP    = 0x09,
} lis2dh12_odr_t;

typedef enum {
  LIS2DH12_2g   = 0,
  LIS2DH12_4g   = 1,
  LIS2DH12_8g   = 2,
  LIS2DH12_16g  = 3,
} lis2dh12_fs_t;

typedef enum {
  LIS2DH12_HR_12bit   = 0,
  LIS2DH12_NM_10bit   = 1,
  LIS2DH12_LP_8bit    = 2,
} lis2dh12_op_md_t;

#define LIS2DH12_FROM_FS_2g_HR_TO_mg(lsb)  (float)((int16_t)lsb>>4)* 1.0f

/* Private variables ---------------------------------------------------------*/
extern I2C_HandleTypeDef hi2c2;

static uint32_t tickstart;

/* Private function prototypes -----------------------------------------------*/
static uint8_t drv_acc_id_get(uint8_t *buff);
static uint8_t drv_acc_block_data_update_set(uint8_t val);
static uint8_t drv_acc_data_rate_set(lis2dh12_odr_t val);
static uint8_t drv_acc_full_scale_set(lis2dh12_fs_t val);
static uint8_t drv_acc_operating_mode_set(lis2dh12_op_md_t val);
static uint8_t drv_acc_status_get(lis2dh12_status_reg_t *val);
static uint8_t drv_acc_acceleration_raw_get(uint8_t *buff);

/**
  * @brief  The application entry point.
  */
void Drv_ACC_Init(void)
{
  uint8_t whoami = 0;

  HAL_Delay(1000);
  /*
   *  Check device ID
   */
  drv_acc_id_get(&whoami);
  if (whoami!=LIS2DH12_ID)
  {
    Drv_SERIAL_Log("acc init failed");
    return;
  }
  /*
   *  Enable Block Data Update
   */
  drv_acc_block_data_update_set(PROPERTY_ENABLE);
  /*
   * Set Output Data Rate
   */
  drv_acc_data_rate_set(LIS2DH12_ODR_10Hz);
  /*
   * Set full scale
   */  
  drv_acc_full_scale_set(LIS2DH12_2g);
  /*
   * Set device in continuous mode
   */   
  drv_acc_operating_mode_set(LIS2DH12_HR_12bit);
}

void Drv_ACC_Proc(void)
{
  if((HAL_GetTick() - tickstart) >= 1000)
  {
    axis3bit16_t data_raw_acceleration;  
    float acceleration_mg[3];
    /*
     * Read output only if new value is available
     */
    lis2dh12_status_reg_t status_reg;
    drv_acc_status_get(&status_reg);
    
    if (status_reg.zyxda)
    {
      /* Read accelerometer data */
      memset(data_raw_acceleration.u8bit, 0x00, 3*sizeof(int16_t));
      drv_acc_acceleration_raw_get(data_raw_acceleration.u8bit);
      acceleration_mg[0] = LIS2DH12_FROM_FS_2g_HR_TO_mg( data_raw_acceleration.i16bit[0] );
      acceleration_mg[1] = LIS2DH12_FROM_FS_2g_HR_TO_mg( data_raw_acceleration.i16bit[1] );
      acceleration_mg[2] = LIS2DH12_FROM_FS_2g_HR_TO_mg( data_raw_acceleration.i16bit[2] );

      Drv_SERIAL_Log("%4.2f %4.2f %4.2f", 
        acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
    }
  
    tickstart = HAL_GetTick();
  }
}

static uint8_t drv_acc_id_get(uint8_t *buff)
{
  return ACC_READ(LIS2DH12_WHO_AM_I, buff, 1);
}

static uint8_t drv_acc_block_data_update_set(uint8_t val)
{
  lis2dh12_ctrl_reg4_t ctrl_reg4;
  uint8_t ret;

  ret = ACC_READ(LIS2DH12_CTRL_REG4, (uint8_t*)&ctrl_reg4, 1);
  if (ret == 0) {
    ctrl_reg4.bdu = val;
    ret = ACC_WRITE(LIS2DH12_CTRL_REG4, (uint8_t*)&ctrl_reg4, 1);
  }
  return ret;
}

static uint8_t drv_acc_data_rate_set(lis2dh12_odr_t val)
{
  lis2dh12_ctrl_reg1_t ctrl_reg1;
  uint8_t ret;

  ret = ACC_READ(LIS2DH12_CTRL_REG1, (uint8_t*)&ctrl_reg1, 1);
  if (ret == 0) {
    ctrl_reg1.odr = (uint8_t)val;
    ret = ACC_WRITE(LIS2DH12_CTRL_REG1, (uint8_t*)&ctrl_reg1, 1);
  }
  return ret;
}

static uint8_t drv_acc_full_scale_set(lis2dh12_fs_t val)
{
  lis2dh12_ctrl_reg4_t ctrl_reg4;
  uint8_t ret;

  ret = ACC_READ(LIS2DH12_CTRL_REG4, (uint8_t*)&ctrl_reg4, 1);
  if (ret == 0) {
    ctrl_reg4.fs = (uint8_t)val;
    ret = ACC_WRITE(LIS2DH12_CTRL_REG4, (uint8_t*)&ctrl_reg4, 1);
  }
  return ret;
}

static uint8_t drv_acc_operating_mode_set(lis2dh12_op_md_t val)
{
  lis2dh12_ctrl_reg1_t ctrl_reg1;
  lis2dh12_ctrl_reg4_t ctrl_reg4;
  uint8_t ret;

  ret = ACC_READ(LIS2DH12_CTRL_REG1,
                          (uint8_t*)&ctrl_reg1, 1);
  if (ret == 0) {
    ret = ACC_READ(LIS2DH12_CTRL_REG4,
                            (uint8_t*)&ctrl_reg4, 1);    
    if (ret == 0) {
      if ( val == LIS2DH12_HR_12bit ) {
        ctrl_reg1.lpen = 0;
        ctrl_reg4.hr   = 1;      
      }
      if (val == LIS2DH12_NM_10bit) {
        ctrl_reg1.lpen = 0;
        ctrl_reg4.hr   = 0;
      }
      if (val == LIS2DH12_LP_8bit) {
        ctrl_reg1.lpen = 1;
        ctrl_reg4.hr   = 0;
      }
      ret = ACC_WRITE(LIS2DH12_CTRL_REG1, (uint8_t*)&ctrl_reg1, 1);
      if (ret == 0) {
        ret = ACC_WRITE(LIS2DH12_CTRL_REG4, (uint8_t*)&ctrl_reg4, 1);
      } 
    }
  }
  return ret;
}

static uint8_t drv_acc_status_get(lis2dh12_status_reg_t *val)
{
  return ACC_READ(LIS2DH12_STATUS_REG, (uint8_t*) val, 1);
}

static uint8_t drv_acc_acceleration_raw_get(uint8_t *buff)
{
  return ACC_READ(LIS2DH12_OUT_X_L, buff, 6);
}

