
// ST standard peripheral library includes.
#include "stm32f4xx_i2c.h"

// FS datatypes includes.
#include "FS_DT_I2C.h"

// FS ST abstraction library defines.
#include "FS_STM32F4xx_PinMux.h"

/*
A header which the project itself must supply - should define
the internal buffer size etc.
*/
#include "FS_STM32F4xxI2C_Conf.h"

typedef struct
{
  FS_DT_I2CDriver_t i2c1;
  FS_DT_I2CDriver_t i2c2;
  FS_DT_I2CDriver_t i2c3;

}FS_STM32F4xxI2CDriver;


typedef struct
{
  // A flag to indicate whether or not the peripheral in question should be initialised.
  _Bool initialise;

  // ST's init struct.
  I2C_InitTypeDef * stInitStruct;

  // Pin muxing.
  FS_STM32F4xxMuxablePin sda;
  FS_STM32F4xxMuxablePin scl;

  // EEPROM spoof for slave functionality.
  const char * slaveMemory;
  uint32_t slaveMemorySizeBytes;

}FS_STM32F4xxI2C_PeriphInitStruct;

typedef struct
{
  FS_DT_I2CRole role;
  FS_STM32F4xxI2CDriver * driverInstance;
  FS_STM32F4xxI2C_PeriphInitStruct i2c1_InitStruct;
  FS_STM32F4xxI2C_PeriphInitStruct i2c2_InitStruct;
  FS_STM32F4xxI2C_PeriphInitStruct i2c3_InitStruct;

}FS_STM32F4xxI2C_InitStruct;


void FS_STM32F4xxI2C_InitStructInit(FS_STM32F4xxI2C_InitStruct * initStruct);
_Bool FS_STM32F4xxI2C_Init(FS_STM32F4xxI2C_InitStruct * initStruct);
