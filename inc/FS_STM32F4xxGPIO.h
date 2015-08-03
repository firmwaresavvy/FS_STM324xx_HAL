/**
 *******************************************************************************
 *
 * @file  FS_STM32F4xxGPIO.h
 *
 * @brief FirmwareSavvy STM32F4xx GPIO abstraction header.
 *
 * @author Andy Norris [andy@firmwaresavvy.com]
 *
 *******************************************************************************
 */
// Preprocessor guard.
#ifndef FS_STM32F4XXGPIO_H
#define FS_STM32F4XXGPIO_H

/*------------------------------------------------------------------------------
------------------------------ START INCLUDES ----------------------------------
------------------------------------------------------------------------------*/

// FS library includes.
#include "FS_DT_Conf.h"

// ST library includes.
#include "stm32f4xx_conf.h"

/*------------------------------------------------------------------------------
------------------------------- END INCLUDES -----------------------------------
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
---------------------- START PUBLIC FUNCTION PROTOTYPES ------------------------
------------------------------------------------------------------------------*/

_Bool FS_STM32F4xxGPIO_ModuleInit(FS_DT_GPIO_PinControlInterface_t * instance);

/*------------------------------------------------------------------------------
----------------------- END PUBLIC FUNCTION PROTOTYPES -------------------------
------------------------------------------------------------------------------*/

#endif // FS_STM32F4XXGPIO_H

