// Preprocessor guard.
#ifndef FS_STM324XXPINMUX_H
#define FS_STM324XXPINMUX_H


typedef enum
{
  FS_STM32F4xxPortA       = 0,
  FS_STM32F4xxPortB       = 1,
  FS_STM32F4xxPortC       = 2,
  FS_STM32F4xxPortD       = 3,
  FS_STM32F4xxPortE       = 4,
  FS_STM32F4xxPortF       = 5,
  FS_STM32F4xxPortG       = 6,
  FS_STM32F4xxPortH       = 7,
  FS_STM32F4xxPortI       = 8,
  FS_STM32F4xxPortNoPort  = 9


}FS_STM32F4xxMuxablePinPort_e;

typedef enum
{
  FS_STM32F4xxPinBit0     = 0,
  FS_STM32F4xxPinBit1     = 1,
  FS_STM32F4xxPinBit2     = 2,
  FS_STM32F4xxPinBit3     = 3,
  FS_STM32F4xxPinBit4     = 4,
  FS_STM32F4xxPinBit5     = 5,
  FS_STM32F4xxPinBit6     = 6,
  FS_STM32F4xxPinBit7     = 7,
  FS_STM32F4xxPinBit8     = 8,
  FS_STM32F4xxPinBit9     = 9,
  FS_STM32F4xxPinBit10    = 10,
  FS_STM32F4xxPinBit11    = 11,
  FS_STM32F4xxPinBit12    = 12,
  FS_STM32F4xxPinBit13    = 13,
  FS_STM32F4xxPinBit14    = 14,
  FS_STM32F4xxPinBit15    = 15,
  FS_STM32F4xxPinNoPin    = 16

}FS_STM32F4xxMuxablePinBit_e;

typedef struct
{
  FS_STM32F4xxMuxablePinPort_e port;
  FS_STM32F4xxMuxablePinBit_e bit;

}FS_STM32F4xxMuxablePin_t;

typedef struct
{
  void(*runPort)(FS_STM32F4xxMuxablePinPort_e port);
  void(*setPinFunction)(FS_STM32F4xxMuxablePin_t * pin, void * peripheral);

}FS_STM32F4xxPinMux_t;

#endif // FS_STM324XXPINMUX_H
