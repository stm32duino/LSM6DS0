/**
 ******************************************************************************
 * @file    LSM6DS0Sensor.h
 * @author  AST
 * @version V1.0.0
 * @date    7 September 2017
 * @brief   Abstract Class of an LSM6DS0 Inertial Measurement Unit (IMU) 6 axes
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


/* Prevent recursive inclusion -----------------------------------------------*/

#ifndef __LSM6DS0Sensor_H__
#define __LSM6DS0Sensor_H__


/* Includes ------------------------------------------------------------------*/

#include "Wire.h"
#include "SPI.h"
#include "LSM6DS0_ACC_GYRO_Driver.h"

/* Defines -------------------------------------------------------------------*/

#define LSM6DS0_ACC_SENSITIVITY_FOR_FS_2G   0.061f  /**< Sensitivity value for 2 g full scale [mg/LSB] */
#define LSM6DS0_ACC_SENSITIVITY_FOR_FS_4G   0.122f  /**< Sensitivity value for 4 g full scale [mg/LSB] */
#define LSM6DS0_ACC_SENSITIVITY_FOR_FS_8G   0.244f  /**< Sensitivity value for 8 g full scale [mg/LSB] */
#define LSM6DS0_ACC_SENSITIVITY_FOR_FS_16G  0.732f  /**< Sensitivity value for 16 g full scale [mg/LSB] */

#define LSM6DS0_GYRO_SENSITIVITY_FOR_FS_245DPS   08.75f  /**< Sensitivity value for 245 dps full scale [mdps/LSB] */
#define LSM6DS0_GYRO_SENSITIVITY_FOR_FS_500DPS   17.50f  /**< Sensitivity value for 500 dps full scale [mdps/LSB] */
#define LSM6DS0_GYRO_SENSITIVITY_FOR_FS_2000DPS  70.00f  /**< Sensitivity value for 2000 dps full scale [mdps/LSB] */


/* Typedefs ------------------------------------------------------------------*/
typedef enum
{
  LSM6DS0_STATUS_OK = 0,
  LSM6DS0_STATUS_ERROR,
  LSM6DS0_STATUS_TIMEOUT,
  LSM6DS0_STATUS_NOT_IMPLEMENTED
} LSM6DS0StatusTypeDef;


/* Class Declaration ---------------------------------------------------------*/

/**
 * Abstract class of an LSM6DS0 Inertial Measurement Unit (IMU) 6 axes
 * sensor.
 */
class LSM6DS0Sensor
{
  public:
    LSM6DS0Sensor                                       (TwoWire *i2c, uint8_t address=LSM6DS0_ACC_GYRO_I2C_ADDRESS_HIGH);
    LSM6DS0Sensor                                       (SPIClass *spi, int cs_pin, uint32_t spi_speed=2000000);
    LSM6DS0StatusTypeDef begin                          (void);
    LSM6DS0StatusTypeDef end                            (void);
    LSM6DS0StatusTypeDef Enable_X                       (void);
    LSM6DS0StatusTypeDef Enable_G                       (void);
    LSM6DS0StatusTypeDef Disable_X                      (void);
    LSM6DS0StatusTypeDef Disable_G                      (void);
    LSM6DS0StatusTypeDef ReadID                         (uint8_t *p_id);
    LSM6DS0StatusTypeDef Get_X_Axes                     (int32_t *pData);
    LSM6DS0StatusTypeDef Get_G_Axes                     (int32_t *pData);
    LSM6DS0StatusTypeDef Get_X_Sensitivity              (float *pfData);
    LSM6DS0StatusTypeDef Get_G_Sensitivity              (float *pfData);
    LSM6DS0StatusTypeDef Get_X_AxesRaw                  (int16_t *pData);
    LSM6DS0StatusTypeDef Get_G_AxesRaw                  (int16_t *pData);
    LSM6DS0StatusTypeDef Get_X_ODR                      (float *odr);
    LSM6DS0StatusTypeDef Get_G_ODR                      (float *odr);
    LSM6DS0StatusTypeDef Set_X_ODR                      (float odr);
    LSM6DS0StatusTypeDef Set_G_ODR                      (float odr);
    LSM6DS0StatusTypeDef Get_X_FS                       (float *fullScale);
    LSM6DS0StatusTypeDef Get_G_FS                       (float *fullScale);
    LSM6DS0StatusTypeDef Set_X_FS                       (float fullScale);
    LSM6DS0StatusTypeDef Set_G_FS                       (float fullScale);
	LSM6DS0StatusTypeDef ReadReg                        (uint8_t reg, uint8_t *data);
	LSM6DS0StatusTypeDef WriteReg                       (uint8_t reg, uint8_t data);
	
	/**
     * @brief Utility function to read data.
     * @param  pBuffer: pointer to data to be read.
     * @param  RegisterAddr: specifies internal address register to be read.
     * @param  NumByteToRead: number of bytes to be read.
     * @retval 0 if ok, an error code otherwise.
     */
    uint8_t IO_Read(uint8_t* pBuffer, uint8_t RegisterAddr, uint16_t NumByteToRead)
    {
      if (dev_spi) {
        dev_spi->beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE3));

        digitalWrite(cs_pin, LOW);

        /* Write Reg Address */
        dev_spi->transfer(RegisterAddr | 0x80);
        /* Read the data */
        for (uint16_t i=0; i<NumByteToRead; i++) {
          *(pBuffer+i) = dev_spi->transfer(0x00);
        }
         
        digitalWrite(cs_pin, HIGH);

        dev_spi->endTransaction();

        return 0;
      }
		
      if (dev_i2c) {
        dev_i2c->beginTransmission(((uint8_t)(((address) >> 1) & 0x7F)));
        dev_i2c->write(RegisterAddr);
        dev_i2c->endTransmission(false);

        dev_i2c->requestFrom(((uint8_t)(((address) >> 1) & 0x7F)), (byte) NumByteToRead);

        int i=0;
        while (dev_i2c->available())
        {
          pBuffer[i] = dev_i2c->read();
          i++;
        }

        return 0;
      }

      return 1;
    }
    
    /**
     * @brief Utility function to write data.
     * @param  pBuffer: pointer to data to be written.
     * @param  RegisterAddr: specifies internal address register to be written.
     * @param  NumByteToWrite: number of bytes to write.
     * @retval 0 if ok, an error code otherwise.
     */
    uint8_t IO_Write(uint8_t* pBuffer, uint8_t RegisterAddr, uint16_t NumByteToWrite)
    {
      if (dev_spi) {
        dev_spi->beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE3));

        digitalWrite(cs_pin, LOW);

        /* Write Reg Address */
        dev_spi->transfer(RegisterAddr);
        /* Write the data */
        for (uint16_t i=0; i<NumByteToWrite; i++) {
          dev_spi->transfer(pBuffer[i]);
        }

        digitalWrite(cs_pin, HIGH);

        dev_spi->endTransaction();

        return 0;                    
      }
  
      if (dev_i2c) {
        dev_i2c->beginTransmission(((uint8_t)(((address) >> 1) & 0x7F)));

        dev_i2c->write(RegisterAddr);
        for (int i = 0 ; i < NumByteToWrite ; i++)
          dev_i2c->write(pBuffer[i]);

        dev_i2c->endTransmission(true);

        return 0;
      }

      return 1;
    }

  private:
    LSM6DS0StatusTypeDef Set_X_ODR_When_Enabled(float odr);
    LSM6DS0StatusTypeDef Set_G_ODR_When_Enabled(float odr);
    LSM6DS0StatusTypeDef Set_X_ODR_When_Disabled(float odr);
    LSM6DS0StatusTypeDef Set_G_ODR_When_Disabled(float odr);

    /* Helper classes. */
    TwoWire *dev_i2c;
    SPIClass *dev_spi;
    
    /* Configuration */
    uint8_t address;
    int cs_pin;
    uint32_t spi_speed;

    uint8_t X_isEnabled;
    float X_Last_ODR;
    uint8_t G_isEnabled;
    float G_Last_ODR;
};

#ifdef __cplusplus
extern "C" {
#endif
uint8_t LSM6DS0_IO_Write( void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite );
uint8_t LSM6DS0_IO_Read( void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead );
#ifdef __cplusplus
}
#endif

#endif