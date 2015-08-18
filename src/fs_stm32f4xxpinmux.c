// Own header.
#include "FS_STM32F4xxPinMux.h"

// ST library includes.
#include "stm32f4xx_conf.h"

// C standard library includes.
#include <stddef.h>
#include <stdbool.h>


static void runPort(FS_STM32F4xxMuxablePinPort_e port);
static void setPinFunction(FS_STM32F4xxMuxablePin_t pin, void * peripheral, uint8_t af);


void FS_STM32F4xxPinMux_Init(FS_STM32F4xxPinMux_t * instance)
{
  // Assign the pointers in the interface struct.
  instance->runPort = runPort;
  instance->setPinFunction = setPinFunction;
}

static void runPort(FS_STM32F4xxMuxablePinPort_e port)
{
  // TODO Change to deal with being passed a pointer to port!!!!
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA << (uint8_t)port);
}

static void setPinFunction(FS_STM32F4xxMuxablePin_t pin, void * peripheral, uint8_t af)
{
  uint32_t portBaseAddress;

  portBaseAddress = GPIOA_BASE + ( 0x400 * (uint8_t)pin.port );
  GPIO_PinAFConfig( ( GPIO_TypeDef * )portBaseAddress, ( uint8_t )pin.bit, af );
}
