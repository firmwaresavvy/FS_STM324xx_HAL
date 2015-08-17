/**
 *******************************************************************************
 *
 * @file  FS_STM32F4xxUSART.c
 *
 * @brief STM32F4xx USART driver header.
 *
 * @author Andy Norris [andy@firmwaresavvy.com]
 *
 *******************************************************************************
 */

// Preprocessor guard.
#ifndef FS_STM32F4XXUSART_H
#define FS_STM32F4XXUSART_H

/*------------------------------------------------------------------------------
------------------------------ START INCLUDES ----------------------------------
------------------------------------------------------------------------------*/

// FS library includes.
#include "FS_DT_Conf.h"
#include "FS_STM32F4xxPinMux.h"

// ST library includes.
#include "stm32f4xx_conf.h"

// Project must supply this header.
#include "FS_STM32F4xxUSART_Conf.h"

/*------------------------------------------------------------------------------
------------------------------- END INCLUDES -----------------------------------
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
---------------------- START PUBLIC TYPE DEFINITIONS ---------------------------
------------------------------------------------------------------------------*/

typedef struct
{
  FS_DT_USARTDriver_t usart1;
  FS_DT_USARTDriver_t usart2;
  FS_DT_USARTDriver_t usart3;
  FS_DT_USARTDriver_t uart4;
  FS_DT_USARTDriver_t uart5;
  FS_DT_USARTDriver_t usart6;

}FS_STM32F4xxUSARTDriver_t;



typedef struct
{
  // A flag to indicate whether or not the peripheral in question should be initialised.
  _Bool initialise;

  // Pointer to the peripheral itself.
  USART_TypeDef * peripheral;

  // ST's init structs.
  USART_InitTypeDef stInitStruct;
  USART_ClockInitTypeDef stClkInitStruct;

  // Pin multiplexing:

  // Control interface.
  FS_STM32F4xxPinMux_t * pinMux;

  // The pins.
  FS_STM32F4xxMuxablePin_t txd;
  FS_STM32F4xxMuxablePin_t rxd;
  FS_STM32F4xxMuxablePin_t rts;
  FS_STM32F4xxMuxablePin_t cts;
  FS_STM32F4xxMuxablePin_t sclk;

  // Buffers.
  uint16_t txBufferSizeBytes;
  uint16_t rxBufferSizeBytes;

}FS_STM32F4xxUSART_PeriphInitStruct_t;

typedef struct
{
  _Bool success;
  void(*mainLoop)(void * params);

}FS_STM32F4xxUSART_InitReturnsStruct_t;

typedef struct
{
  FS_STM32F4xxUSARTDriver_t * instance;
  FS_STM32F4xxUSART_PeriphInitStruct_t usart1InitStruct;
  FS_STM32F4xxUSART_PeriphInitStruct_t usart2InitStruct;
  FS_STM32F4xxUSART_PeriphInitStruct_t usart3InitStruct;
  FS_STM32F4xxUSART_PeriphInitStruct_t uart4InitStruct;
  FS_STM32F4xxUSART_PeriphInitStruct_t uart5InitStruct;
  FS_STM32F4xxUSART_PeriphInitStruct_t usart6InitStruct;

}FS_STM32F4xxUSART_InitStruct_t;


/*------------------------------------------------------------------------------
----------------------- END PUBLIC TYPE DEFINITIONS ----------------------------
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
-------------------- START PUBLIC FUNCTION PROTOTYPES --------------------------
------------------------------------------------------------------------------*/

// Data structure initialisation functions.
void FS_STM32F4xxUSART_InitStructInit(FS_STM32F4xxUSART_InitStruct_t * initStruct);
void FS_STM32F4xxUSART_InitReturnsStructInit(FS_STM32F4xxUSART_InitReturnsStruct_t * returnsStruct);
void FS_STM32F4xxUSART_PeriphInitStructInit(FS_STM32F4xxUSART_PeriphInitStruct_t * initStruct);

// Module initialisation.
FS_STM32F4xxUSART_InitReturnsStruct_t
FS_STM32F4xxUSART_Init(FS_STM32F4xxUSART_InitStruct_t * initStruct);

/*------------------------------------------------------------------------------
--------------------- END PUBLIC FUNCTION PROTOTYPES ---------------------------
------------------------------------------------------------------------------*/

#endif // FS_STM32F4XXUSART_H
