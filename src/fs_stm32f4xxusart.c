// Own header.
#include "FS_STM32F4xxUSART.h"

// Free RTOS includes.
#include "semphr.h"


typedef struct
{
  uint16_t base;
  uint16_t length;
  uint16_t head;
  uint16_t tail;
  uint16_t fillLevel;
  uint16_t highWater; // For debug later.
  SemaphoreHandle_t mutex;

}USARTBuffer;

typedef struct
{
  USART_TypeDef * peripheral;
  _Bool enabled;

  USARTBuffer txBuffer;
  USARTBuffer rxBuffer;

}USART;



/*
Master buffer from which memory is allocated for all input/output buffers
in this driver. This is the main determinant of RAM usage for this module.
Length set by application in FS_STM32F4xxUSART_Conf.h.
*/
static char masterBuffer[FS_STM32F4XXUSART_MASTER_BUFFER_LENGTH_BYTES];
uint16_t masterBufferAllocatedBytes;

static USART usartList[6];

static SemaphoreHandle_t irqSyncSemaphore;

void FS_STM32F4xxUSART_InitStructInit(FS_STM32F4xxUSART_InitStruct * initStruct)
{

}

_Bool FS_STM32F4xxUSART_Init(FS_STM32F4xxUSART_InitStruct * initStruct)
{
  _Bool success;

  // This semaphore will cause the task to block until any U(S)ART interrupt occurs.
  irqSyncSemaphore = xSemaphoreCreateBinary();

  if(initStruct->usart1InitStruct->initialise)
  {
    if( !initUsart( 0, usart1InitStruct ) )
    {
      return false;
    }
  }

  if(initStruct->usart2InitStruct->initialise)
  {
    if( !initUsart( 1, usart1InitStruct ) )
    {
      return false;
    }
  }

  if(initStruct->usart3InitStruct->initialise)
  {
    if( !initUsart( 2, usart1InitStruct ) )
    {
      return false;
    }
  }

  if(initStruct->uart4InitStruct->initialise)
  {
    if( !initUsart( 3, uart4InitStruct ) )
    {
      return false;
    }
  }

  if(initStruct->uart5InitStruct->initialise)
  {
    if( !initUsart( 4, uart4InitStruct ) )
    {
      return false;
    }
  }

  if(initStruct->usart6InitStruct->initialise)
  {
    if( !initUsart( 6, usart1InitStruct ) )
    {
      return false;
    }
  }
}

// Init an individual peripheral instance.
static _Bool initUsart(uint8_t listIndex, FS_STM32F4xxUSART_PeriphInitStruct * initStruct)
{
  _Bool rtsRequired, ctsRequired, sclkRequired;
  uint16_t tempU16;

  /*
  Firstly, check if enough memory remains in the master buffer to
  satisfy the allocation requirements. If not, go no further.
  */
  if( ( initStruct->rxBufferSizeBytes + initStruct->txBufferSizeBytes ) >
      ( FS_STM32F4XXUSART_MASTER_BUFFER_LENGTH_BYTES - masterBufferAllocatedBytes ) )
  {
    return false;
  }

  /*
  Next, check if the IO pins are in their init states. If not, something else has already
  claimed them. Go no further. Only check clock and flow control pins if functionality has
  been requested.
  */
  if( !FS_STM32F4xxPinMux_InInitState(initStruct->txd) ||
      !FS_STM32F4xxPinMux_InInitState(initStruct->rxd) )
  {
    return false;
  }

  // Determine if synchronous mode is to be used.
  if(FS_STM32F4xxPinMux_NoPin != initStruct->sclk)
  {
    sclkRequired = true;
  }

  if( sclkRequired && !FS_STM32F4xxPinMux_InInitState(initStruct->sclk) )
  {
    return false;
  }


  if( ( USART_HardwareFlowControl_CTS == initStruct->stInitStruct->USART_HardwareFlowControl ) ||
      ( USART_HardwareFlowControl_RTS_CTS== initStruct->stInitStruct->USART_HardwareFlowControl ) )
  {
    ctsRequired = true;
  }

  if( ( USART_HardwareFlowControl_RTS == initStruct->stInitStruct->USART_HardwareFlowControl ) ||
      ( USART_HardwareFlowControl_RTS_CTS== initStruct->stInitStruct->USART_HardwareFlowControl ) )
  {
    rtsRequired = true;
  }


  if( ctsRequired && !FS_STM32F4xxPinMux_InInitState(initStruct->cts) )
  {
    return false;
  }

  if( rtsRequired && !FS_STM32F4xxPinMux_InInitState(initStruct->rts) )
  {
    return false;
  }

  // Copy the pertinent information into the USART list.
  usartList[listIndex].enabled = true;
  usartList[listIndex].peripheral = initStruct->peripheral;

  // Init the buffers.
  bufferInit(usartList[listIndex].txBuffer);
  bufferInit(usartList[listIndex].rxBuffer);

  // Start clocking the appropriate port blocks.
  FS_STM32F4xxPinMux_RunPort(initStruct->txd);
  FS_STM32F4xxPinMux_RunPort(initStruct->rxd);
  //...
  //...
  //...
  //...

  //<---------------ANDY START HERE!!!!

  // Attempt to set the pins up for U(S)ART functionality.
  if( !FS_STM32F4xxPinMux_SetPinFunction(initStruct->txd, FS_STM32F4xxPinMux_USART) )
  {
    return false;
  }


}

static void mainLoop(void * params)
{
  uint8_t i;
  USART * usart;
  unsigned char data;

  while(true)
  {
    // If the semaphore can't be taken, there's no work to do and the task will block.
    xSemaphoreTake(irqSyncSemaphore);

    for(i = 0; i < 6; i++)
    {
      usart = &(usartList[i]);

      if(usart->enabled)
      {
        // Do tx tasks.
        if( pop( usart->txBuffer, &data ) )
        {
          USART_SendData(usart->peripheral, (uint16_t)data);
        }

        else
        {
          // If no data to send, prevent any further tx interrupts.
          USART_ITConfig(usart->peripheral, USART_IT_TXE, DISABLE);
        }

        // Check if a byte has been received.
        if( SET == USART_GetFlagStatus(USART_FLAG_RXNE) )
        {
          data = (unsigned char)USART_ReceiveData(usart->peripheral);
          push(usart->rxBuffer, data);
        }
      }
    }
  }
}

// Buffer functions.
static void bufferInit(USARTBuffer * buf, uint16_t base)
{

}

static void bufferPush(USARTBuffer buf, unsigned char data)
{
  xSemaphoreTake(buf->mutex);

  masterBuffer[buf->tail] = data;

  // Wrap if necessary.
  if( ( buf->base + buf->length ) == ( buf->tail  + 1 ) )
  {
    buf->tail = buf->base;
  }

  else
  {
    buf->tail++;
  }

  /*
  If the buffer was already full, we have wrapped and data will be destroyed.
  Otherwise, increment the fill level.
  */
  if( !( buf->fillLevel == buf->length ) )
  {
    buf->fillLevel++;
  }

  // Update the high water mark if necessary.
  if(buf->fillLevel > buf->highWater)
  {
    buf->highWater= buf->fillLevel;
  }

  xSemaphoreGive(buf->mutex);
}

static _Bool bufferPop(USARTBuffer buf, unsigned char * data)
{
   _Bool success;

  xSemaphoreTake(buf->mutex);

  // Check whether there's any data...
  if(buf->fillLevel)
  {
    *data = masterBuffer[buf->head];

    // Wrap if necessary.
    if( ( buf->base + buf->length ) == ( buf->head  + 1 ) )
    {
      buf->head = buf->base;
    }

    else
    {
      buf->head++;
    }

    buf->fillLevel--;

    success = true;
  }

  else
  {
    success = false;
  }

  xSemaphoreGive(buf->mutex);
  return success;
}

// Interrupt handlers.
void USART1_IRQHandler(void)
{

}

void USART2_IRQHandler(void)
{

}

void USART3_IRQHandler(void)
{

}

void UART4_IRQHandler(void)
{

}

void UART5_IRQHandler(void)
{

}

void USART6_IRQHandler(void)
{

}
