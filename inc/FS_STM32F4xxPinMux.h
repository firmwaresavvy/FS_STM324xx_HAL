// Preprocessor guard.
#ifndef FS_STM324XXPINMUX_H
#define FS_STM324XXPINMUX_H

// C standard library includes.
#include <stdint.h>

typedef struct
{
  GPIO_TypeDef * port;
  uint16_t pinMask;

}FS_STM32F4xxMuxablePin_t;

typedef struct
{
  void(*runPort)(FS_STM32F4xxMuxablePinPort_e port);
  void(*setPinFunction)(FS_STM32F4xxMuxablePin_t pin, void * peripheral, uint8_t af);

}FS_STM32F4xxPinMux_t;

#endif // FS_STM324XXPINMUX_H
