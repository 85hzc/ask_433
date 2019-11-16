#include "main.h"
#include "stm32f1xx_hal.h"
//#include "drv.h"
#include "app.h"
#include "siliconSi115x.h"

extern gs_struct gs;
extern volatile uint16_t I2C_SDA_PIN;
extern volatile uint16_t I2C_SCL_PIN;
static uint32_t tickstart;

static int16_t _waitUntilSleep()
{
  int16_t retval = -1;
  uint8_t count = 0;
  // This loops until the Si115x is known to be in its sleep state
  // or if an i2c error occurs
  while(count < 5)
  {
    retval = Si115xReadFromRegister(SI115x_REG_RESPONSE0);
    if((retval&SI115x_RSP0_CHIPSTAT_MASK) == SI115x_RSP0_SLEEP)
      break;
    if(retval <  0)
      return retval;
    count++;
  }
  return 0;
}


int16_t _sendCmd(uint8_t command)
{
  int16_t  response;
  int8_t   retval;
  uint8_t  count = 0;

  // Get the response register contents
  response = Si115xReadFromRegister(SI115x_REG_RESPONSE0);
  if(response < 0)
  {
    return response;
  }

  response = response & SI115x_RSP0_COUNTER_MASK;

  // Double-check the response register is consistent
  while(count < 5)
  {
    if((retval = _waitUntilSleep()) != 0)
      return retval;

    if(command == 0)
      break; // Skip if the command is NOP

    retval = Si115xReadFromRegister(SI115x_REG_RESPONSE0);

    if((retval&SI115x_RSP0_COUNTER_MASK) == response)
      break;
    else if(retval < 0)
      return retval;
    else
      response = retval & SI115x_RSP0_COUNTER_MASK;

    count++;
  } // end loop

  // Send the Command
  if(retval = (Si115xWriteToRegister(SI115x_REG_COMMAND, command, 1))!= 0)
  {
    return retval;
  }

  count = 0;
  // Expect a change in the response register
  while(count < 5)
  {
    if(command == 0)
      break; // Skip if the command is NOP

    retval = Si115xReadFromRegister(SI115x_REG_RESPONSE0);
    if((retval & SI115x_RSP0_COUNTER_MASK) != response)
      break;
    else if(retval < 0)
      return retval;

    count++;
  } // end loop

  return 0;
}


int16_t Si115xBlockWrite(uint8_t address, uint8_t length, uint8_t * data)
{

    LOG_DEBUG("Si115xBlockWrite address=0x%x\r\n",address);

    //siliconSensor_IO_Write_block(data, SI1153_I2C_ADDR, address, length);
    //siliconSensor_IO_Read(&data, SI1153_I2C_ADDR, RegisterAddr, 1);
    SILAB_IO_Write_Block(data, SI1153_I2C_ADDR, address, length);

    return 0;
}


int16_t Si115xParamSet(uint8_t address, uint8_t value)
{
  int16_t retval;
  uint8_t buffer[2];
  int16_t response_stored;
  int16_t response;

  LOG_DEBUG("Si115xParamSet address=0x%x\r\n",address);

  retval = _waitUntilSleep();
  if(retval !=0)
  {
    return retval;
  }

  response_stored = SI115x_RSP0_COUNTER_MASK
                    & Si115xReadFromRegister(SI115x_REG_RESPONSE0);

  buffer[0] = value;
  buffer[1] = 0x80 + (address & 0x3F);

  retval = Si115xBlockWrite(SI115x_REG_HOSTIN0,
                            2,
                            (uint8_t*) buffer);
  if(retval != 0)
    return retval;

  // Wait for command to finish
  response = Si115xReadFromRegister(SI115x_REG_RESPONSE0);
  while((response & SI115x_RSP0_COUNTER_MASK) == response_stored)
  {
    response = Si115xReadFromRegister(SI115x_REG_RESPONSE0);
  }

  if(retval < 0)
    return retval;
  else
    return 0;
}

int16_t Si115xReset()
{
  int16_t retval = 0;

  LOG_DEBUG("------------Si115xReset-------------\r\n");

  // Do not access the Si115x earlier than 25 ms from power-up.
  // Uncomment the following lines if Si115xReset() is the first
  // instruction encountered, and if your system MCU boots up too
  // quickly.
  HAL_Delay(10);
  HAL_Delay(10);
  HAL_Delay(10);

  // Perform the Reset Command
  retval += Si115xWriteToRegister(SI115x_REG_COMMAND, 1, 1);

  // Delay for 10 ms. This delay is needed to allow the Si115x
  // to perform internal reset sequence.
  HAL_Delay(10);

  return retval;
}


// ch0: prox, large IR photodiode, 97us integration time, low signal range, LED2 = 321mA, LED1 = LED3 = none, accumulate 1, no right shift
int16_t Si115xInitLongRangeProx(  )
{
    int16_t    retval;

    retval  = Si115xReset( );
    HAL_Delay(100);
    LOG_DEBUG("------------Si115xInitLongRangeProx 3cho-------------\r\n");

#if(SENSOR3==1)
    retval += Si115xParamSet( SI115x_PARAM_CH_LIST, 0x01);

    retval += Si115xParamSet( SI115x_PARAM_LED1_A, 0x3f);

    retval += Si115xParamSet( SI115x_PARAM_ADCCONFIG0, 0x62);
    retval += Si115xParamSet( SI115x_PARAM_MEASCONFIG0, 0x01);

    retval += Si115xParamSet( SI115x_PARAM_ADCSENS0, 0x03);
#else

#if(CHN3_GES)
    retval += Si115xParamSet( SI115x_PARAM_CH_LIST, 0x07);
#else
    retval += Si115xParamSet( SI115x_PARAM_CH_LIST, 0x03);
#endif

    retval += Si115xParamSet( SI115x_PARAM_LED1_A, 0x3f);
    retval += Si115xParamSet( SI115x_PARAM_LED2_A, 0x3f);
#if(CHN3_GES)
    retval += Si115xParamSet( SI115x_PARAM_LED3_A, 0x3f);
#endif

    retval += Si115xParamSet( SI115x_PARAM_ADCCONFIG0, 0x62);//0x02
    retval += Si115xParamSet( SI115x_PARAM_MEASCONFIG0, 0x01);

    retval += Si115xParamSet( SI115x_PARAM_ADCCONFIG1, 0x62);//0x62
    retval += Si115xParamSet( SI115x_PARAM_MEASCONFIG1, 0x02);
#if(CHN3_GES)
    retval += Si115xParamSet( SI115x_PARAM_ADCCONFIG2, 0x02);//0x62
    retval += Si115xParamSet( SI115x_PARAM_MEASCONFIG2, 0x04);
#endif
    retval += Si115xParamSet( SI115x_PARAM_ADCSENS0, 0x03);
    retval += Si115xParamSet( SI115x_PARAM_ADCSENS1, 0x03);
#if(CHN3_GES)
    retval += Si115xParamSet( SI115x_PARAM_ADCSENS2, 0x03);
#endif
#endif
    //retval += Si115xWriteToRegister( SI115x_REG_IRQ_ENABLE, 0x07, 1);

    return retval;
}

int16_t Si115xInitProxAls( uint8_t proxOnly )
{
    int16_t    retval;

    retval  = Si115xReset( );
    HAL_Delay(100);

    if (proxOnly) // prox only, no als
    {
        retval += Si115xParamSet(  SI115x_PARAM_LED1_A, 0x3f);
        retval += Si115xParamSet(  SI115x_PARAM_CH_LIST, 0x01);
        retval += Si115xParamSet(  SI115x_PARAM_ADCCONFIG0, 0x62);
        retval += Si115xParamSet(  SI115x_PARAM_ADCSENS0, 0x80);//80
        retval += Si115xParamSet(  SI115x_PARAM_ADCPOST0, 0x40);
        retval += Si115xParamSet(  SI115x_PARAM_MEASCONFIG0, 0x21);
        retval += Si115xWriteToRegister(  SI115x_REG_IRQ_ENABLE, 0x01);
    }
    else // prox + als
    {
        retval += Si115xParamSet(  SI115x_PARAM_LED1_A, 0x3f); // LED1
        retval += Si115xParamSet(  SI115x_PARAM_CH_LIST, 0x0f);
        retval += Si115xParamSet(  SI115x_PARAM_ADCCONFIG0, 0x62);
        retval += Si115xParamSet(  SI115x_PARAM_ADCSENS0, 0x80);
        retval += Si115xParamSet(  SI115x_PARAM_ADCPOST0, 0x40);
        retval += Si115xParamSet(  SI115x_PARAM_MEASCONFIG0, 0x21); //LED1
        retval += Si115xParamSet(  SI115x_PARAM_ADCCONFIG1, 0x4d);
        retval += Si115xParamSet(  SI115x_PARAM_ADCSENS1, 0xe1);
        retval += Si115xParamSet(  SI115x_PARAM_ADCPOST1, 0x40);
        retval += Si115xParamSet(  SI115x_PARAM_ADCCONFIG2, 0x41);
        retval += Si115xParamSet(  SI115x_PARAM_ADCSENS2, 0xe1);
        retval += Si115xParamSet(  SI115x_PARAM_ADCPOST2, 0x50);
        retval += Si115xParamSet(  SI115x_PARAM_ADCCONFIG3, 0x4d);
        retval += Si115xParamSet(  SI115x_PARAM_ADCSENS3, 0x87);
        retval += Si115xParamSet(  SI115x_PARAM_ADCPOST3, 0x40);
        retval += Si115xWriteToRegister(  SI115x_REG_IRQ_ENABLE, 0x0f);
    }

    return retval;
}

void Si115xForce(void)
{
    //LOG_DEBUG("------------Si115xForce-------------\r\n");
    _sendCmd(0x11);
}

void getSensorDataByHostout(uint16_t *ps)
{

#if(SENSOR3==1)
    I2C_SCL_PIN = SCL1_Pin;
    I2C_SDA_PIN = SDA1_Pin;
    Si115xForce();
    HAL_Delay(1);

    ps[0] = Si115xReadFromRegister (SI115x_REG_HOSTOUT1) +
         256*Si115xReadFromRegister(SI115x_REG_HOSTOUT0);
    
    I2C_SCL_PIN = SCL2_Pin;
    I2C_SDA_PIN = SDA2_Pin;
    Si115xForce();
    HAL_Delay(1);

    ps[1] = Si115xReadFromRegister (SI115x_REG_HOSTOUT1) +
         256*Si115xReadFromRegister(SI115x_REG_HOSTOUT0);

    I2C_SCL_PIN = SCL3_Pin;
    I2C_SDA_PIN = SDA3_Pin;
    Si115xForce();
    HAL_Delay(1);

    ps[2] = Si115xReadFromRegister (SI115x_REG_HOSTOUT1) +
         256*Si115xReadFromRegister(SI115x_REG_HOSTOUT0);

#if (SERSOR_LOG_ENABLE)
    //LOG_DEBUG("%5d  %5d  %5d\r\n",ps[1]<=gs.sample_base[1]?0:ps[1]-gs.sample_base[1],
    //                    ps[2]<=gs.sample_base[2]?0:ps[2]-gs.sample_base[2],
    //                    ps[0]<=gs.sample_base[0]?0:ps[0]-gs.sample_base[0]);
    LOG_DEBUG("[%5d %5d %5d]\r\n",
                        //ps[1],ps[2],ps[0],
                        //gs.sample_base_last[1],gs.sample_base_last[2],gs.sample_base_last[0],
                        ps[1]<=gs.sample_base_last[1]?gs.sample_base_last[1]-ps[1]:ps[1]-gs.sample_base_last[1],
                        ps[2]<=gs.sample_base_last[2]?gs.sample_base_last[2]-ps[2]:ps[2]-gs.sample_base_last[2],
                        ps[0]<=gs.sample_base_last[0]?gs.sample_base_last[0]-ps[0]:ps[0]-gs.sample_base_last[0]);
                        //ps[1]<=gs.sample_base[1]?gs.sample_base[1]-ps[1]:ps[1]-gs.sample_base[1],
                        //ps[2]<=gs.sample_base[2]?gs.sample_base[2]-ps[2]:ps[2]-gs.sample_base[2],
                        //ps[0]<=gs.sample_base[0]?gs.sample_base[0]-ps[0]:ps[0]-gs.sample_base[0]);
#endif
#else
    I2C_SCL_PIN = SCL3_Pin;
    I2C_SDA_PIN = SDA3_Pin;
    Si115xForce();
    HAL_Delay(1);

    ps[0] = Si115xReadFromRegister (SI115x_REG_HOSTOUT1) +
         256*Si115xReadFromRegister(SI115x_REG_HOSTOUT0);
    ps[1] = Si115xReadFromRegister (SI115x_REG_HOSTOUT3) +
         256*Si115xReadFromRegister(SI115x_REG_HOSTOUT2);
#if(CHN3_GES)
    ps[2] = Si115xReadFromRegister (SI115x_REG_HOSTOUT5) +
         256*Si115xReadFromRegister(SI115x_REG_HOSTOUT4);
#endif

#if (SERSOR_LOG_ENABLE)
#if(CHN3_GES)
    LOG_DEBUG("[%5d %5d %5d]\r\n",
                        ps[1]<=gs.sample_base_last[1]?gs.sample_base_last[1]-ps[1]:ps[1]-gs.sample_base_last[1],
                        ps[2]<=gs.sample_base_last[2]?gs.sample_base_last[2]-ps[2]:ps[2]-gs.sample_base_last[2],
                        ps[0]<=gs.sample_base_last[0]?gs.sample_base_last[0]-ps[0]:ps[0]-gs.sample_base_last[0]);
#else
    LOG_DEBUG("[%5d %5d]\r\n",
                        ps[1]<=gs.sample_base_last[1]?gs.sample_base_last[1]-ps[1]:ps[1]-gs.sample_base_last[1],
                        ps[0]<=gs.sample_base_last[0]?gs.sample_base_last[0]-ps[0]:ps[0]-gs.sample_base_last[0]);

#endif
#endif
#endif
}


void DEMO_Init()
{
    uint8_t data = 0xff;

    LOG_DEBUG("------------DEMO_Init-------------\r\n");

    //Get HW ID of the part and use that to decide what we are talking to
    data = Si115xReadFromRegister( SI1153_REG_HW_ID);

    switch(data){
        case SI1153_PROX_HW_ID:     //In case 0, this could be multiple sensors, but we assume PROX and declare if the switch is set to another sensor then there is unpredictable behavior.
            Si115xInitProxAls(false);
            break;
        case SI1153_LR_PROX_HW_ID:
            Si115xInitLongRangeProx();
            break;
    }
    Si115xForce();
}


void Drv_SILICON_Proc(void)
{
  //if((HAL_GetTick() - tickstart) >= 20) {

    //tickstart = HAL_GetTick();
    //Si115xForce();
    //HAL_Delay(2);
    App_Task();
  //}
}


void i2cmsginfo(char const *tx, int txlen, char const *rx, int rxlen)
{
#if 0
    int i;
    if (tx != NULL) {
        LOG_DEBUG("TX:");
        for(i=0;i<txlen;i++) {
            LOG_DEBUG(" 0x%x", tx[i]);
        }
        
        LOG_DEBUG("\r\n");
    }
    if (rx != NULL) {
        LOG_DEBUG("RX:");
        for(i=0;i<rxlen;i++) {
            LOG_DEBUG(" 0x%x", rx[i]);
        }
        LOG_DEBUG("\r\n");
    }
    LOG_DEBUG("----------\r\n");
#endif
}


