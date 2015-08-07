
// Preprocessor guard.
#ifndef FS_STM32F4XXUSART_H
#define FS_STM32F4XXUSART_H

#include "FS_DT_Conf.h"
#include "FS_STM32F4xxUSART_Conf.h" // Project must supply this header.

typedef struct
{
  FS_DT_USARTDriver_t usart1;
  FS_DT_USARTDriver_t usart2;
  FS_DT_USARTDriver_t usart3;
  FS_DT_USARTDriver_t uart4;
  FS_DT_USARTDriver_t uart5;
  FS_DT_USARTDriver_t usart6;

}FS_STM32F4xxUSARTDriver;



typedef struct
{
  // A flag to indicate whether or not the peripheral in question should be initialised.
  _Bool initialise;

  // Pointer to the peripheral itself.
  USART_TypeDef * peripheral;

  // ST's init struct.
  USART_InitTypeDef * stInitStruct;

  // Pin muxing.
  FS_STM32F4xxMuxablePin txd;
  FS_STM32F4xxMuxablePin rxd;
  FS_STM32F4xxMuxablePin rts;
  FS_STM32F4xxMuxablePin cts;
  FS_STM32F4xxMuxablePin sclk;

  uint16_t txBufferSizeBytes;
  uint16_t rxBufferSizeBytes;

}FS_STM32F4xxUSART_PeriphInitStruct;

typedef struct
{
  FS_STM32F4xxUSARTDriver * instance;
  FS_STM32F4xxUSART_PeriphInitStruct usart1InitStruct;
  FS_STM32F4xxUSART_PeriphInitStruct usart2InitStruct;
  FS_STM32F4xxUSART_PeriphInitStruct usart3InitStruct;
  FS_STM32F4xxUSART_PeriphInitStruct uart4InitStruct;
  FS_STM32F4xxUSART_PeriphInitStruct uart5InitStruct;
  FS_STM32F4xxUSART_PeriphInitStruct usart6InitStruct;

}FS_STM32F4xxUSART_InitStruct;


void FS_STM32F4xxUSART_InitStructInit(FS_STM32F4xxUSART_InitStruct * initStruct);
_Bool FS_STM32F4xxUSART_Init(FS_STM32F4xxUSART_InitStruct * initStruct);

#endif // FS_STM32F4XXUSART_H
