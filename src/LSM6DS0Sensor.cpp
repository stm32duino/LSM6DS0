/**
 ******************************************************************************
 * @file    LSM6DS0Sensor.cpp
 * @author  AST
 * @version V1.0.0
 * @date    7 September 2017
 * @brief   Implementation of an LSM6DS0 Inertial Measurement Unit (IMU) 6 axes
 *          sensor.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */


/* Includes ------------------------------------------------------------------*/

#include "Arduino.h"
#include "LSM6DS0Sensor.h"


/* Class Implementation ------------------------------------------------------*/
/** Constructor
 * @param i2c object of an helper class which handles the I2C peripheral
 * @param address the address of the component's instance
 */
LSM6DS0Sensor::LSM6DS0Sensor(TwoWire *i2c, uint8_t address) : dev_i2c(i2c), address(address)
{
  dev_spi = NULL;
  X_isEnabled = 0;
  G_isEnabled = 0;
}

/** Constructor
 * @param spi object of an helper class which handles the SPI peripheral
 * @param cs_pin the chip select pin
 * @param spi_speed the SPI speed
 */
LSM6DS0Sensor::LSM6DS0Sensor(SPIClass *spi, int cs_pin, uint32_t spi_speed) : dev_spi(spi), cs_pin(cs_pin), spi_speed(spi_speed)
{
  dev_i2c = NULL;
  address = 0;
  X_isEnabled = 0U;
  G_isEnabled = 0U;
}

/**
 * @brief  Configure the sensor in order to be used
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DS0StatusTypeDef LSM6DS0Sensor::begin()
{
  if(dev_spi)
  {
    // Configure CS pin
    pinMode(cs_pin, OUTPUT);
    digitalWrite(cs_pin, HIGH); 
  }

  /* Enable BDU */
  if ( LSM6DS0_ACC_GYRO_W_BlockDataUpdate( (void *)this, LSM6DS0_ACC_GYRO_BDU_ENABLE ) == MEMS_ERROR )
  {
    return LSM6DS0_STATUS_ERROR;
  }

  /* Output data rate selection - power down. */
  if ( LSM6DS0_ACC_GYRO_W_AccelerometerDataRate( (void *)this, LSM6DS0_ACC_GYRO_ODR_XL_POWER_DOWN ) == MEMS_ERROR )
  {
    return LSM6DS0_STATUS_ERROR;
  }

  /* Full scale selection */
  if ( Set_X_FS( 2.0f ) == LSM6DS0_STATUS_ERROR )
  {
    return LSM6DS0_STATUS_ERROR;
  }

  /* Enable axes */
  if ( LSM6DS0_ACC_GYRO_W_AccelerometerAxisX( (void *)this, LSM6DS0_ACC_GYRO_XEN_XL_ENABLE ) == MEMS_ERROR )
  {
    return LSM6DS0_STATUS_ERROR;
  }

  if ( LSM6DS0_ACC_GYRO_W_AccelerometerAxisY( (void *)this, LSM6DS0_ACC_GYRO_YEN_XL_ENABLE ) == MEMS_ERROR )
  {
    return LSM6DS0_STATUS_ERROR;
  }

  if ( LSM6DS0_ACC_GYRO_W_AccelerometerAxisZ( (void *)this, LSM6DS0_ACC_GYRO_ZEN_XL_ENABLE ) == MEMS_ERROR )
  {
    return LSM6DS0_STATUS_ERROR;
  }

  /* Enable BDU */
  if ( LSM6DS0_ACC_GYRO_W_BlockDataUpdate( (void *)this, LSM6DS0_ACC_GYRO_BDU_ENABLE ) == MEMS_ERROR )
  {
    return LSM6DS0_STATUS_ERROR;
  }

  /* Output data rate selection - power down. */
  if ( LSM6DS0_ACC_GYRO_W_GyroDataRate( (void *)this, LSM6DS0_ACC_GYRO_ODR_G_POWER_DOWN ) == MEMS_ERROR )
  {
    return LSM6DS0_STATUS_ERROR;
  }

  /* Full scale selection */
  if ( Set_G_FS( 2000.0f ) == LSM6DS0_STATUS_ERROR )
  {
    return LSM6DS0_STATUS_ERROR;
  }

  /* Enable axes */
  if ( LSM6DS0_ACC_GYRO_W_GyroAxisX( (void *)this, LSM6DS0_ACC_GYRO_XEN_G_ENABLE ) == MEMS_ERROR )
  {
    return LSM6DS0_STATUS_ERROR;
  }

  if ( LSM6DS0_ACC_GYRO_W_GyroAxisY( (void *)this, LSM6DS0_ACC_GYRO_YEN_G_ENABLE ) == MEMS_ERROR )
  {
    return LSM6DS0_STATUS_ERROR;
  }

  if ( LSM6DS0_ACC_GYRO_W_GyroAxisZ( (void *)this, LSM6DS0_ACC_GYRO_ZEN_G_ENABLE ) == MEMS_ERROR )
  {
    return LSM6DS0_STATUS_ERROR;
  }
  
  X_Last_ODR = 119.0f;

  X_isEnabled = 0;
  
  G_Last_ODR = 119.0f;

  G_isEnabled = 0;

  return LSM6DS0_STATUS_OK;
}

/**
 * @brief  Disable the sensor and relative resources
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DS0StatusTypeDef LSM6DS0Sensor::end()
{
  /* Disable both acc and gyro */
  if (Disable_X() != LSM6DS0_STATUS_OK)
  {
    return LSM6DS0_STATUS_ERROR;
  }

  if (Disable_G() != LSM6DS0_STATUS_OK)
  {
    return LSM6DS0_STATUS_ERROR;
  }

  /* Reset CS configuration */
  if(dev_spi)
  {
    // Configure CS pin
    pinMode(cs_pin, INPUT); 
  }

  return LSM6DS0_STATUS_OK;
}

/**
 * @brief  Enable LSM6DS0 Accelerator
 * @retval LSM6DS0_STATUS_OK in case of success, an error code otherwise
 */
LSM6DS0StatusTypeDef LSM6DS0Sensor::Enable_X(void)
{ 
  /* Check if the component is already enabled */
  if ( X_isEnabled == 1 )
  {
    return LSM6DS0_STATUS_OK;
  }
  
  /* Output data rate selection. */
  if ( Set_X_ODR_When_Enabled( X_Last_ODR ) == LSM6DS0_STATUS_ERROR )
  {
    return LSM6DS0_STATUS_ERROR;
  }
  
  X_isEnabled = 1;
  
  return LSM6DS0_STATUS_OK;
}

/**
 * @brief  Enable LSM6DS0 Gyroscope
 * @retval LSM6DS0_STATUS_OK in case of success, an error code otherwise
 */
LSM6DS0StatusTypeDef LSM6DS0Sensor::Enable_G(void)
{ 
  /* Check if the component is already enabled */
  if ( G_isEnabled == 1 )
  {
    return LSM6DS0_STATUS_OK;
  }
  
  /* Output data rate selection. */
  if ( Set_G_ODR_When_Enabled( G_Last_ODR ) == LSM6DS0_STATUS_ERROR )
  {
    return LSM6DS0_STATUS_ERROR;
  }
  
  G_isEnabled = 1;
  
  return LSM6DS0_STATUS_OK;
}

/**
 * @brief  Disable LSM6DS0 Accelerator
 * @retval LSM6DS0_STATUS_OK in case of success, an error code otherwise
 */
LSM6DS0StatusTypeDef LSM6DS0Sensor::Disable_X(void)
{ 
  /* Check if the component is already disabled */
  if ( X_isEnabled == 0 )
  {
    return LSM6DS0_STATUS_OK;
  }
  
  /* Store actual output data rate. */
  if ( Get_X_ODR( &X_Last_ODR ) == LSM6DS0_STATUS_ERROR )
  {
    return LSM6DS0_STATUS_ERROR;
  }
  
  /* Output data rate selection - power down. */
  if ( LSM6DS0_ACC_GYRO_W_AccelerometerDataRate( (void *)this, LSM6DS0_ACC_GYRO_ODR_XL_POWER_DOWN ) == MEMS_ERROR )
  {
    return LSM6DS0_STATUS_ERROR;
  }
  
  X_isEnabled = 0;
  
  return LSM6DS0_STATUS_OK;
}

/**
 * @brief  Disable LSM6DS0 Gyroscope
 * @retval LSM6DS0_STATUS_OK in case of success, an error code otherwise
 */
LSM6DS0StatusTypeDef LSM6DS0Sensor::Disable_G(void)
{ 
  /* Check if the component is already disabled */
  if ( G_isEnabled == 0 )
  {
    return LSM6DS0_STATUS_OK;
  }
  
  /* Store actual output data rate. */
  if ( Get_G_ODR( &G_Last_ODR ) == LSM6DS0_STATUS_ERROR )
  {
    return LSM6DS0_STATUS_ERROR;
  }
  
  /* Output data rate selection - power down. */
  if ( LSM6DS0_ACC_GYRO_W_GyroDataRate( (void *)this, LSM6DS0_ACC_GYRO_ODR_G_POWER_DOWN ) == MEMS_ERROR )
  {
    return LSM6DS0_STATUS_ERROR;
  }
  
  G_isEnabled = 0;
  
  return LSM6DS0_STATUS_OK;
}

/**
 * @brief  Read ID of LSM6DS0 Accelerometer and Gyroscope
 * @param  p_id the pointer where the ID of the device is stored
 * @retval LSM6DS0_STATUS_OK in case of success, an error code otherwise
 */
LSM6DS0StatusTypeDef LSM6DS0Sensor::ReadID(uint8_t *p_id)
{
  if(!p_id)
  { 
    return LSM6DS0_STATUS_ERROR;
  }

  /* Read WHO AM I register */
  if ( LSM6DS0_ACC_GYRO_R_WHO_AM_I_( (void *)this, p_id ) == MEMS_ERROR )
  {
    return LSM6DS0_STATUS_ERROR;
  }
  
  return LSM6DS0_STATUS_OK;
}

/**
 * @brief  Read data from LSM6DS0 Accelerometer
 * @param  pData the pointer where the accelerometer data are stored
 * @retval LSM6DS0_STATUS_OK in case of success, an error code otherwise
 */
LSM6DS0StatusTypeDef LSM6DS0Sensor::Get_X_Axes(int32_t *pData)
{
  int16_t dataRaw[3];
  float sensitivity = 0;

  /* Read raw data from LSM6DS0 output register. */
  if ( Get_X_AxesRaw( dataRaw ) == LSM6DS0_STATUS_ERROR )
  {
    return LSM6DS0_STATUS_ERROR;
  }

  /* Get LSM6DS0 actual sensitivity. */
  if ( Get_X_Sensitivity( &sensitivity ) == LSM6DS0_STATUS_ERROR )
  {
    return LSM6DS0_STATUS_ERROR;
  }

  /* Calculate the data. */
  pData[0] = ( int32_t )( dataRaw[0] * sensitivity );
  pData[1] = ( int32_t )( dataRaw[1] * sensitivity );
  pData[2] = ( int32_t )( dataRaw[2] * sensitivity );

  return LSM6DS0_STATUS_OK;
}

/**
 * @brief  Read data from LSM6DS0 Gyroscope
 * @param  pData the pointer where the gyroscope data are stored
 * @retval LSM6DS0_STATUS_OK in case of success, an error code otherwise
 */
LSM6DS0StatusTypeDef LSM6DS0Sensor::Get_G_Axes(int32_t *pData)
{
  int16_t dataRaw[3];
  float sensitivity = 0;

  /* Read raw data from LSM6DS0 output register. */
  if ( Get_G_AxesRaw( dataRaw ) == LSM6DS0_STATUS_ERROR )
  {
    return LSM6DS0_STATUS_ERROR;
  }

  /* Get LSM6DS0 actual sensitivity. */
  if ( Get_G_Sensitivity( &sensitivity ) == LSM6DS0_STATUS_ERROR )
  {
    return LSM6DS0_STATUS_ERROR;
  }

  /* Calculate the data. */
  pData[0] = ( int32_t )( dataRaw[0] * sensitivity );
  pData[1] = ( int32_t )( dataRaw[1] * sensitivity );
  pData[2] = ( int32_t )( dataRaw[2] * sensitivity );

  return LSM6DS0_STATUS_OK;
}

/**
 * @brief  Read Accelerometer Sensitivity
 * @param  pfData the pointer where the accelerometer sensitivity is stored
 * @retval LSM6DS0_STATUS_OK in case of success, an error code otherwise
 */
LSM6DS0StatusTypeDef LSM6DS0Sensor::Get_X_Sensitivity(float *pfData)
{
  LSM6DS0_ACC_GYRO_FS_XL_t fullScale;

  /* Read actual full scale selection from sensor. */
  if ( LSM6DS0_ACC_GYRO_R_AccelerometerFullScale( (void *)this, &fullScale ) == MEMS_ERROR )
  {
    return LSM6DS0_STATUS_ERROR;
  }

  /* Store the sensitivity based on actual full scale. */
  switch( fullScale )
  {
    case LSM6DS0_ACC_GYRO_FS_XL_2g:
      *pfData = ( float )LSM6DS0_ACC_SENSITIVITY_FOR_FS_2G;
      break;
    case LSM6DS0_ACC_GYRO_FS_XL_4g:
      *pfData = ( float )LSM6DS0_ACC_SENSITIVITY_FOR_FS_4G;
      break;
    case LSM6DS0_ACC_GYRO_FS_XL_8g:
      *pfData = ( float )LSM6DS0_ACC_SENSITIVITY_FOR_FS_8G;
      break;
    case LSM6DS0_ACC_GYRO_FS_XL_16g:
      *pfData = ( float )LSM6DS0_ACC_SENSITIVITY_FOR_FS_16G;
      break;
    default:
      *pfData = -1.0f;
      return LSM6DS0_STATUS_ERROR;
  }

  return LSM6DS0_STATUS_OK;
}

/**
 * @brief  Read Gyroscope Sensitivity
 * @param  pfData the pointer where the gyroscope sensitivity is stored
 * @retval LSM6DS0_STATUS_OK in case of success, an error code otherwise
 */
LSM6DS0StatusTypeDef LSM6DS0Sensor::Get_G_Sensitivity(float *pfData)
{
  LSM6DS0_ACC_GYRO_FS_G_t fullScale;

  /* Read actual full scale selection from sensor. */
  if ( LSM6DS0_ACC_GYRO_R_GyroFullScale( (void *)this, &fullScale ) == MEMS_ERROR )
  {
    return LSM6DS0_STATUS_ERROR;
  }

  /* Store the sensitivity based on actual full scale. */
  switch( fullScale )
  {
    case LSM6DS0_ACC_GYRO_FS_G_245dps:
      *pfData = ( float )LSM6DS0_GYRO_SENSITIVITY_FOR_FS_245DPS;
      break;
    case LSM6DS0_ACC_GYRO_FS_G_500dps:
      *pfData = ( float )LSM6DS0_GYRO_SENSITIVITY_FOR_FS_500DPS;
      break;
    case LSM6DS0_ACC_GYRO_FS_G_2000dps:
      *pfData = ( float )LSM6DS0_GYRO_SENSITIVITY_FOR_FS_2000DPS;
      break;
    default:
      *pfData = -1.0f;
      return LSM6DS0_STATUS_ERROR;
  }

  return LSM6DS0_STATUS_OK;
}

/**
 * @brief  Read raw data from LSM6DS0 Accelerometer
 * @param  pData the pointer where the accelerometer raw data are stored
 * @retval LSM6DS0_STATUS_OK in case of success, an error code otherwise
 */
LSM6DS0StatusTypeDef LSM6DS0Sensor::Get_X_AxesRaw(int16_t *pData)
{
  uint8_t regValue[6] = {0, 0, 0, 0, 0, 0};

  /* Read output registers from LSM6DS0_ACC_GYRO_OUT_X_L_XL to LSM6DS0_ACC_GYRO_OUT_Z_H_XL. */
  if ( LSM6DS0_ACC_GYRO_Get_Acceleration( (void *)this, ( uint8_t* )regValue ) == MEMS_ERROR )
  {
    return LSM6DS0_STATUS_ERROR;
  }

  /* Format the data. */
  pData[0] = ( ( ( ( int16_t )regValue[1] ) << 8 ) + ( int16_t )regValue[0] );
  pData[1] = ( ( ( ( int16_t )regValue[3] ) << 8 ) + ( int16_t )regValue[2] );
  pData[2] = ( ( ( ( int16_t )regValue[5] ) << 8 ) + ( int16_t )regValue[4] );

  return LSM6DS0_STATUS_OK;
}

/**
 * @brief  Read raw data from LSM6DS0 Gyroscope
 * @param  pData the pointer where the gyroscope raw data are stored
 * @retval LSM6DS0_STATUS_OK in case of success, an error code otherwise
 */
LSM6DS0StatusTypeDef LSM6DS0Sensor::Get_G_AxesRaw(int16_t *pData)
{
  uint8_t regValue[6] = {0, 0, 0, 0, 0, 0};

  /* Read output registers from LSM6DS0_ACC_GYRO_OUT_X_L_G to LSM6DS0_ACC_GYRO_OUT_Z_H_G. */
  if ( LSM6DS0_ACC_GYRO_Get_AngularRate( (void *)this, ( uint8_t* )regValue ) == MEMS_ERROR )
  {
    return LSM6DS0_STATUS_ERROR;
  }

  /* Format the data. */
  pData[0] = ( ( ( ( int16_t )regValue[1] ) << 8 ) + ( int16_t )regValue[0] );
  pData[1] = ( ( ( ( int16_t )regValue[3] ) << 8 ) + ( int16_t )regValue[2] );
  pData[2] = ( ( ( ( int16_t )regValue[5] ) << 8 ) + ( int16_t )regValue[4] );

  return LSM6DS0_STATUS_OK;
}

/**
 * @brief  Read LSM6DS0 Accelerometer output data rate
 * @param  odr the pointer to the output data rate
 * @retval LSM6DS0_STATUS_OK in case of success, an error code otherwise
 */
LSM6DS0StatusTypeDef LSM6DS0Sensor::Get_X_ODR(float* odr)
{
  LSM6DS0_ACC_GYRO_ODR_XL_t odr_low_level;

  /* Accelerometer ODR forced to be same like gyroscope ODR. */
  if(G_isEnabled == 1)
  {
    *odr = G_Last_ODR;
    return LSM6DS0_STATUS_OK;
  }

  if ( LSM6DS0_ACC_GYRO_R_AccelerometerDataRate( (void *)this, &odr_low_level ) == MEMS_ERROR )
  {
    return LSM6DS0_STATUS_ERROR;
  }

  switch( odr_low_level )
  {
    case LSM6DS0_ACC_GYRO_ODR_XL_POWER_DOWN:
      *odr =   0.0f;
      break;
    case LSM6DS0_ACC_GYRO_ODR_XL_10Hz:
      *odr =  10.0f;
      break;
    case LSM6DS0_ACC_GYRO_ODR_XL_50Hz:
      *odr =  50.0f;
      break;
    case LSM6DS0_ACC_GYRO_ODR_XL_119Hz:
      *odr = 119.0f;
      break;
    case LSM6DS0_ACC_GYRO_ODR_XL_238Hz:
      *odr = 238.0f;
      break;
    case LSM6DS0_ACC_GYRO_ODR_XL_476Hz:
      *odr = 476.0f;
      break;
    case LSM6DS0_ACC_GYRO_ODR_XL_952Hz:
      *odr = 952.0f;
      break;
    default:
      *odr =  -1.0f;
      return LSM6DS0_STATUS_ERROR;
  }

  return LSM6DS0_STATUS_OK;
}

/**
 * @brief  Read LSM6DS0 Gyroscope output data rate
 * @param  odr the pointer to the output data rate
 * @retval LSM6DS0_STATUS_OK in case of success, an error code otherwise
 */
LSM6DS0StatusTypeDef LSM6DS0Sensor::Get_G_ODR(float* odr)
{
  LSM6DS0_ACC_GYRO_ODR_G_t odr_low_level;

  if ( LSM6DS0_ACC_GYRO_R_GyroDataRate( (void *)this, &odr_low_level ) == MEMS_ERROR )
  {
    return LSM6DS0_STATUS_ERROR;
  }

  switch( odr_low_level )
  {
    case LSM6DS0_ACC_GYRO_ODR_G_POWER_DOWN:
      *odr =   0.0f;
      break;
    case LSM6DS0_ACC_GYRO_ODR_G_15Hz:
      *odr =  15.0f;
      break;
    case LSM6DS0_ACC_GYRO_ODR_G_60Hz:
      *odr =  60.0f;
      break;
    case LSM6DS0_ACC_GYRO_ODR_G_119Hz:
      *odr = 119.0f;
      break;
    case LSM6DS0_ACC_GYRO_ODR_G_238Hz:
      *odr = 238.0f;
      break;
    case LSM6DS0_ACC_GYRO_ODR_G_476Hz:
      *odr = 476.0f;
      break;
    case LSM6DS0_ACC_GYRO_ODR_G_952Hz:
      *odr = 952.0f;
      break;
    default:
      *odr =  -1.0f;
      return LSM6DS0_STATUS_ERROR;
  }

  return LSM6DS0_STATUS_OK;
}

/**
 * @brief  Set LSM6DS0 Accelerometer output data rate
 * @param  odr the output data rate to be set
 * @retval LSM6DS0_STATUS_OK in case of success, an error code otherwise
 */
LSM6DS0StatusTypeDef LSM6DS0Sensor::Set_X_ODR(float odr)
{
  if(X_isEnabled == 1)
  {
    if(Set_X_ODR_When_Enabled(odr) == LSM6DS0_STATUS_ERROR)
    {
      return LSM6DS0_STATUS_ERROR;
    }
  }
  else
  {
    if(Set_X_ODR_When_Disabled(odr) == LSM6DS0_STATUS_ERROR)
    {
      return LSM6DS0_STATUS_ERROR;
    }
  }
  
  return LSM6DS0_STATUS_OK;
}

/**
 * @brief  Set LSM6DS0 Accelerometer output data rate when enabled
 * @param  odr the output data rate to be set
 * @retval LSM6DS0_STATUS_OK in case of success, an error code otherwise
 */
LSM6DS0StatusTypeDef LSM6DS0Sensor::Set_X_ODR_When_Enabled(float odr)
{
  LSM6DS0_ACC_GYRO_ODR_XL_t new_odr;

  new_odr = ( odr <=  10.0f ) ? LSM6DS0_ACC_GYRO_ODR_XL_10Hz
          : ( odr <=  50.0f ) ? LSM6DS0_ACC_GYRO_ODR_XL_50Hz
          : ( odr <= 119.0f ) ? LSM6DS0_ACC_GYRO_ODR_XL_119Hz
          : ( odr <= 238.0f ) ? LSM6DS0_ACC_GYRO_ODR_XL_238Hz
          : ( odr <= 476.0f ) ? LSM6DS0_ACC_GYRO_ODR_XL_476Hz
          :                     LSM6DS0_ACC_GYRO_ODR_XL_952Hz;

  if ( LSM6DS0_ACC_GYRO_W_AccelerometerDataRate( (void *)this, new_odr ) == MEMS_ERROR )
  {
    return LSM6DS0_STATUS_ERROR;
  }
  
  if(Get_X_ODR( &X_Last_ODR ) == LSM6DS0_STATUS_ERROR )
  {
    return LSM6DS0_STATUS_ERROR;
  }

  return LSM6DS0_STATUS_OK;
}

/**
 * @brief  Set LSM6DS0 Accelerometer output data rate when disabled
 * @param  odr the output data rate to be set
 * @retval LSM6DS0_STATUS_OK in case of success, an error code otherwise
 */
LSM6DS0StatusTypeDef LSM6DS0Sensor::Set_X_ODR_When_Disabled(float odr)
{ 
  X_Last_ODR = ( odr <=  10.0f ) ? 10.0f
             : ( odr <=  50.0f ) ? 50.0f
             : ( odr <= 119.0f ) ? 119.0f
             : ( odr <= 238.0f ) ? 238.0f
             : ( odr <= 476.0f ) ? 476.0f
             :                     952.0f;
                                 
  return LSM6DS0_STATUS_OK;
}

/**
 * @brief  Set LSM6DS0 Gyroscope output data rate
 * @param  odr the output data rate to be set
 * @retval LSM6DS0_STATUS_OK in case of success, an error code otherwise
 */
LSM6DS0StatusTypeDef LSM6DS0Sensor::Set_G_ODR(float odr)
{
  if(G_isEnabled == 1)
  {
    if(Set_G_ODR_When_Enabled(odr) == LSM6DS0_STATUS_ERROR)
    {
      return LSM6DS0_STATUS_ERROR;
    }
  }
  else
  {
    if(Set_G_ODR_When_Disabled(odr) == LSM6DS0_STATUS_ERROR)
    {
      return LSM6DS0_STATUS_ERROR;
    }
  }
  
  return LSM6DS0_STATUS_OK;
}

/**
 * @brief  Set LSM6DS0 Gyroscope output data rate when enabled
 * @param  odr the output data rate to be set
 * @retval LSM6DS0_STATUS_OK in case of success, an error code otherwise
 */
LSM6DS0StatusTypeDef LSM6DS0Sensor::Set_G_ODR_When_Enabled(float odr)
{
  LSM6DS0_ACC_GYRO_ODR_G_t new_odr;

  new_odr = ( odr <=  15.0f ) ? LSM6DS0_ACC_GYRO_ODR_G_15Hz
          : ( odr <=  60.0f ) ? LSM6DS0_ACC_GYRO_ODR_G_60Hz
          : ( odr <= 119.0f ) ? LSM6DS0_ACC_GYRO_ODR_G_119Hz
          : ( odr <= 238.0f ) ? LSM6DS0_ACC_GYRO_ODR_G_238Hz
          : ( odr <= 476.0f ) ? LSM6DS0_ACC_GYRO_ODR_G_476Hz
          :                     LSM6DS0_ACC_GYRO_ODR_G_952Hz;

  if ( LSM6DS0_ACC_GYRO_W_GyroDataRate( (void *)this, new_odr ) == MEMS_ERROR )
  {
    return LSM6DS0_STATUS_ERROR;
  }

  if(Get_G_ODR( &G_Last_ODR ) == LSM6DS0_STATUS_ERROR )
  {
    return LSM6DS0_STATUS_ERROR;
  }

  return LSM6DS0_STATUS_OK;
}

/**
 * @brief  Set LSM6DS0 Gyroscope output data rate when disabled
 * @param  odr the output data rate to be set
 * @retval LSM6DS0_STATUS_OK in case of success, an error code otherwise
 */
LSM6DS0StatusTypeDef LSM6DS0Sensor::Set_G_ODR_When_Disabled(float odr)
{
  G_Last_ODR = ( odr <=  15.0f ) ? 15.0f
             : ( odr <=  60.0f ) ? 60.0f
             : ( odr <= 119.0f ) ? 119.0f
             : ( odr <= 238.0f ) ? 238.0f
             : ( odr <= 476.0f ) ? 476.0f
             :                     952.0f;

  return LSM6DS0_STATUS_OK;
}

/**
 * @brief  Read LSM6DS0 Accelerometer full scale
 * @param  fullScale the pointer to the full scale
 * @retval LSM6DS0_STATUS_OK in case of success, an error code otherwise
 */
LSM6DS0StatusTypeDef LSM6DS0Sensor::Get_X_FS(float* fullScale)
{
  LSM6DS0_ACC_GYRO_FS_XL_t fs_low_level;

  if ( LSM6DS0_ACC_GYRO_R_AccelerometerFullScale( (void *)this, &fs_low_level ) == MEMS_ERROR )
  {
    return LSM6DS0_STATUS_ERROR;
  }

  switch( fs_low_level )
  {
    case LSM6DS0_ACC_GYRO_FS_XL_2g:
      *fullScale =  2.0f;
      break;
    case LSM6DS0_ACC_GYRO_FS_XL_4g:
      *fullScale =  4.0f;
      break;
    case LSM6DS0_ACC_GYRO_FS_XL_8g:
      *fullScale =  8.0f;
      break;
    case LSM6DS0_ACC_GYRO_FS_XL_16g:
      *fullScale = 16.0f;
      break;
    default:
      *fullScale = -1.0f;
      return LSM6DS0_STATUS_ERROR;
  }

  return LSM6DS0_STATUS_OK;
}

/**
 * @brief  Read LSM6DS0 Gyroscope full scale
 * @param  fullScale the pointer to the full scale
 * @retval LSM6DS0_STATUS_OK in case of success, an error code otherwise
 */
LSM6DS0StatusTypeDef LSM6DS0Sensor::Get_G_FS(float* fullScale)
{
  LSM6DS0_ACC_GYRO_FS_G_t fs_low_level;

  if ( LSM6DS0_ACC_GYRO_R_GyroFullScale( (void *)this, &fs_low_level ) == MEMS_ERROR )
  {
    return LSM6DS0_STATUS_ERROR;
  }

  switch( fs_low_level )
  {
    case LSM6DS0_ACC_GYRO_FS_G_245dps:
      *fullScale =  245.0f;
      break;
    case LSM6DS0_ACC_GYRO_FS_G_500dps:
      *fullScale =  500.0f;
      break;
    case LSM6DS0_ACC_GYRO_FS_G_2000dps:
      *fullScale = 2000.0f;
      break;
    default:
      *fullScale =   -1.0f;
      return LSM6DS0_STATUS_ERROR;
  }

  return LSM6DS0_STATUS_OK;
}

/**
 * @brief  Set LSM6DS0 Accelerometer full scale
 * @param  fullScale the full scale to be set
 * @retval LSM6DS0_STATUS_OK in case of success, an error code otherwise
 */
LSM6DS0StatusTypeDef LSM6DS0Sensor::Set_X_FS(float fullScale)
{
  LSM6DS0_ACC_GYRO_FS_XL_t new_fs;

  new_fs = ( fullScale <= 2.0f ) ? LSM6DS0_ACC_GYRO_FS_XL_2g
         : ( fullScale <= 4.0f ) ? LSM6DS0_ACC_GYRO_FS_XL_4g
         : ( fullScale <= 8.0f ) ? LSM6DS0_ACC_GYRO_FS_XL_8g
         :                         LSM6DS0_ACC_GYRO_FS_XL_16g;

  if ( LSM6DS0_ACC_GYRO_W_AccelerometerFullScale( (void *)this, new_fs ) == MEMS_ERROR )
  {
    return LSM6DS0_STATUS_ERROR;
  }

  return LSM6DS0_STATUS_OK;
}

/**
 * @brief  Set LSM6DS0 Gyroscope full scale
 * @param  fullScale the full scale to be set
 * @retval LSM6DS0_STATUS_OK in case of success, an error code otherwise
 */
LSM6DS0StatusTypeDef LSM6DS0Sensor::Set_G_FS(float fullScale)
{
  LSM6DS0_ACC_GYRO_FS_G_t new_fs;

  new_fs = ( fullScale <= 245.0f ) ? LSM6DS0_ACC_GYRO_FS_G_245dps
         : ( fullScale <= 500.0f ) ? LSM6DS0_ACC_GYRO_FS_G_500dps
         :                           LSM6DS0_ACC_GYRO_FS_G_2000dps;

  if ( LSM6DS0_ACC_GYRO_W_GyroFullScale( (void *)this, new_fs ) == MEMS_ERROR )
  {
    return LSM6DS0_STATUS_ERROR;
  }

  return LSM6DS0_STATUS_OK;
}

/**
 * @brief Read the data from register
 * @param reg register address
 * @param data register data
 * @retval LSM6DS0_STATUS_OK in case of success
 * @retval LSM6DS0_STATUS_ERROR in case of failure
 */
LSM6DS0StatusTypeDef LSM6DS0Sensor::ReadReg( uint8_t reg, uint8_t *data )
{

  if ( LSM6DS0_ACC_GYRO_ReadReg( (void *)this, reg, data, 1 ) == MEMS_ERROR )
  {
    return LSM6DS0_STATUS_ERROR;
  }

  return LSM6DS0_STATUS_OK;
}

/**
 * @brief Write the data to register
 * @param reg register address
 * @param data register data
 * @retval LSM6DS0_STATUS_OK in case of success
 * @retval LSM6DS0_STATUS_ERROR in case of failure
 */
LSM6DS0StatusTypeDef LSM6DS0Sensor::WriteReg( uint8_t reg, uint8_t data )
{

  if ( LSM6DS0_ACC_GYRO_WriteReg( (void *)this, reg, &data, 1 ) == MEMS_ERROR )
  {
    return LSM6DS0_STATUS_ERROR;
  }

  return LSM6DS0_STATUS_OK;
}


uint8_t LSM6DS0_IO_Write( void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite )
{
  return ((LSM6DS0Sensor *)handle)->IO_Write(pBuffer, WriteAddr, nBytesToWrite);
}

uint8_t LSM6DS0_IO_Read( void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead )
{
  return ((LSM6DS0Sensor *)handle)->IO_Read(pBuffer, ReadAddr, nBytesToRead);
}
