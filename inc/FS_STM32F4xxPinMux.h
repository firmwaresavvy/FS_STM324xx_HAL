// Preprocessor guard.
#ifndef FS_STM324XXPINMUX_H
#define FS_STM324XXPINMUX_H

// C standard library includes.
#include <stdint.h>

// ST library includes.
#include "stm32f4xx_gpio.h"

typedef struct
{
  GPIO_TypeDef * port;
  uint32_t portRCCMask;
  uint16_t pinMask;
  uint8_t pinSource;

}FS_STM32F4xxMuxablePin_t;

typedef struct
{
  void(*runPort)(GPIO_TypeDef * port;);
  void(*initPins)(GPIO_TypeDef * port, GPIO_InitTypeDef * initStruct);
  void(*setPinFunction)(FS_STM32F4xxMuxablePin_t pin, void * peripheral, uint8_t af);

}FS_STM32F4xxPinMux_t;

#endif // FS_STM324XXPINMUX_H
