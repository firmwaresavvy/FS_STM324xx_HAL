/**
 *******************************************************************************
 *
 * @file  FS_STM32F4xxUSART.c
 *
 * @brief STM32F4xx USART driver for use with FreeRTOS.
 *
 * @author Andy Norris [andy@firmwaresavvy.com]
 *
 *******************************************************************************
 */

/*------------------------------------------------------------------------------
------------------------------ START INCLUDES ----------------------------------
------------------------------------------------------------------------------*/

// Own header.
#include "FS_STM32F4xxUSART.h"

// C standard library includes.
#include <stdbool.h>

// Free RTOS includes.
#include "FreeRTOS.h"
#include "semphr.h"

/*------------------------------------------------------------------------------
------------------------------- END INCLUDES -----------------------------------
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
--------------------- START PRIVATE TYPE DEFINITIONS ---------------------------
------------------------------------------------------------------------------*/

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

/*------------------------------------------------------------------------------
---------------------- END PRIVATE TYPE DEFINITIONS ----------------------------
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
------------------- START PRIVATE FUNCTION PROTOTYPES --------------------------
------------------------------------------------------------------------------*/

// Init functions.
static _Bool initUsart(uint8_t listIndex, FS_STM32F4xxUSART_PeriphInitStruct_t * initStruct);

// Redirection functions for peripheral instances.
static uint16_t usart1_writeBytes(const char * bytes, uint16_t numBytes);
static uint16_t usart1_writeLine(const char * line);
static uint16_t usart1_readBytes(const char * buf, uint16_t numBytes);
static uint16_t usart1_readLine(const char * buf);

static uint16_t usart2_writeBytes(const char * bytes, uint16_t numBytes);
static uint16_t usart2_writeLine(const char * line);
static uint16_t usart2_readBytes(const char * buf, uint16_t numBytes);
static uint16_t usart2_readLine(const char * buf);

static uint16_t usart3_writeBytes(const char * bytes, uint16_t numBytes);
static uint16_t usart3_writeLine(const char * line);
static uint16_t usart3_readBytes(const char * buf, uint16_t numBytes);
static uint16_t usart3_readLine(const char * buf);

static uint16_t uart4_writeBytes(const char * bytes, uint16_t numBytes);
static uint16_t uart4_writeLine(const char * line);
static uint16_t uart4_readBytes(const char * buf, uint16_t numBytes);
static uint16_t uart4_readLine(const char * buf);

static uint16_t uart5_writeBytes(const char * bytes, uint16_t numBytes);
static uint16_t uart5_writeLine(const char * line);
static uint16_t uart5_readBytes(const char * buf, uint16_t numBytes);
static uint16_t uart5_readLine(const char * buf);

static uint16_t usart6_writeBytes(const char * bytes, uint16_t numBytes);
static uint16_t usart6_writeLine(const char * line);
static uint16_t usart6_readBytes(const char * buf, uint16_t numBytes);
static uint16_t usart6_readLine(const char * buf);

// Implementation of FS_DT_USARTDriver_t.
static uint16_t writeBytes(USART * usart, const char * bytes, uint16_t numBytes);
static uint16_t writeLine(USART * usart, const char * line);
static uint16_t readBytes(USART * usart, const char * buf, uint16_t numBytes);
static uint16_t readLine(USART * usart, const char * buf);

// Buffer functions.
static void bufferInit(USARTBuffer * buf);
static void bufferPush(USARTBuffer * buf, unsigned char data);
static _Bool bufferPop(USARTBuffer * buf, unsigned char * data);

// Task main loop.
static void mainLoop(void * params);

/*------------------------------------------------------------------------------
-------------------- END PRIVATE FUNCTION PROTOTYPES ---------------------------
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
-------------------- START PRIVATE GLOBAL VARIABLES ----------------------------
------------------------------------------------------------------------------*/

/*
Master buffer from which memory is allocated for all input/output buffers
in this driver. This is the main determinant of RAM usage for this module.
Length set by application in FS_STM32F4xxUSART_Conf.h.
*/
static char masterBuffer[FS_STM32F4XXUSART_MASTER_BUFFER_LENGTH_BYTES];
uint16_t masterBufferAllocatedBytes;

/*
List to hold control/management details of all U(S)ART peripherals.s
*/
static USART usartList[6];

// Interrupt synchronisation semaphore - allows the task to block when no work exists to be done.
static SemaphoreHandle_t irqSyncSemaphore;

// Interface to pin multiplexing module.
static FS_STM324xxPinMux_t * pinMux;

/*------------------------------------------------------------------------------
--------------------- END PRIVATE GLOBAL VARIABLES -----------------------------
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
------------------------ START PUBLIC FUNCTIONS --------------------------------
------------------------------------------------------------------------------*/

void FS_STM32F4xxUSART_InitStructInit(FS_STM32F4xxUSART_InitStruct_t * initStruct)
{

}

void FS_STM32F4xxUSART_InitReturnsStructInit(FS_STM32F4xxUSART_InitReturnsStruct_t * returnsStruct)
{
  returnsStruct->success = false;
  returnsStruct->mainLoop = NULL;
}

FS_STM32F4xxUSART_InitReturnsStruct_t FS_STM32F4xxUSART_Init(FS_STM32F4xxUSART_InitStruct_t * initStruct)
{
  FS_STM32F4xxUSART_InitReturnsStruct_t returns;

  FS_STM32F4xxUSART_InitReturnsStructInit(&returns);

  // This semaphore will cause the task to block until any U(S)ART interrupt occurs.
  irqSyncSemaphore = xSemaphoreCreateBinary();

  /*
  Initialise the specified peripherals. Note that in the device, the lowest-numbered
  peripheral is USART1 whereas the array containing the peripheral list within this
  driver is zero-indexed. Position in the list is therefore n-1 for USARTn
  (or UARTn where applicable).
  */
  if(initStruct->usart1InitStruct.initialise)
  {
    if( !initUsart( 0, &( initStruct->usart1InitStruct ) ) )
    {
      return returns;
    }
  }

  if(initStruct->usart2InitStruct.initialise)
  {
    if( !initUsart( 1, &( initStruct->usart2InitStruct ) ) )
    {
      return returns;
    }
  }

  if(initStruct->usart3InitStruct.initialise)
  {
    if( !initUsart( 2, &( initStruct->usart3InitStruct ) ) )
    {
      return returns;
    }
  }

  if(initStruct->uart4InitStruct.initialise)
  {
    if( !initUsart( 3, &( initStruct->uart4InitStruct ) ) )
    {
      return returns;
    }
  }

  if(initStruct->uart5InitStruct.initialise)
  {
    if( !initUsart( 4, &( initStruct->uart5InitStruct ) ) )
    {
      return returns;
    }
  }

  if(initStruct->usart6InitStruct.initialise)
  {
    if( !initUsart( 5, &( initStruct->usart6InitStruct ) ) )
    {
      return returns;
    }
  }

  returns.mainLoop = mainLoop;
  returns.success =  true;
  return returns;
}

/*------------------------------------------------------------------------------
------------------------- END PUBLIC FUNCTIONS ---------------------------------
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
----------------------- START PRIVATE FUNCTIONS --------------------------------
------------------------------------------------------------------------------*/

// Init an individual peripheral instance.
static _Bool initUsart(uint8_t listIndex, FS_STM32F4xxUSART_PeriphInitStruct_t * initStruct)
{
  /*
  Firstly, check if enough memory remains in the master buffer to
  satisfy the allocation requirements. If not, go no further.
  */
  if( ( initStruct->rxBufferSizeBytes + initStruct->txBufferSizeBytes ) >
      ( FS_STM32F4XXUSART_MASTER_BUFFER_LENGTH_BYTES - masterBufferAllocatedBytes ) )
  {
    return false;
  }

  // Copy the pertinent information into the USART list.
  usartList[listIndex].enabled = true;
  usartList[listIndex].peripheral = initStruct->peripheral;

  // Init the buffers.
  bufferInit( &( usartList[listIndex].txBuffer ) );
  bufferInit( &( usartList[listIndex].rxBuffer ) );

  // Start clocking the appropriate port blocks and change the pin functions.
  pinMux->runPort(initStruct->txd.port);
  pinMux->setPinFunction( &( initStruct->txd), (void *)initStruct->peripheral );
  pinMux->runPort(initStruct->rxd.port);
  pinMux->setPinFunction( &( initStruct->rxd), (void *)initStruct->peripheral );

  // Determine if synchronous mode is to be used.
  if(FS_STM32F4xxPinNoPin != initStruct->sclk.bit)
  {
    pinMux->runPort(initStruct->sclk.port);
    pinMux->setPinFunction( &( initStruct->sclk), (void *)initStruct->peripheral );
  }

  // Determine if any hardware flow control functionality is required.
  if( ( USART_HardwareFlowControl_CTS == initStruct->stInitStruct->USART_HardwareFlowControl ) ||
      ( USART_HardwareFlowControl_RTS_CTS== initStruct->stInitStruct->USART_HardwareFlowControl ) )
  {
    pinMux->runPort(initStruct->cts.port);
    pinMux->setPinFunction( &( initStruct->cts ), (void *)initStruct->peripheral );
  }

  if( ( USART_HardwareFlowControl_RTS == initStruct->stInitStruct->USART_HardwareFlowControl ) ||
      ( USART_HardwareFlowControl_RTS_CTS== initStruct->stInitStruct->USART_HardwareFlowControl ) )
  {
    pinMux->runPort(initStruct->rts.port);
    pinMux->setPinFunction( &( initStruct->rts ), (void *)initStruct->peripheral );
  }

  return true;
}

// Redirection functions for peripheral instances.
static uint16_t usart1_writeBytes(const char * bytes, uint16_t numBytes)
{
  // Redirect to implementation function if peripheral enabled.
  if(usartList[0].enabled)
  {
    return writeBytes( &( usartList[0] ), bytes, numBytes );
  }

  else
  {
    return 0;
  }
}

static uint16_t usart1_writeLine(const char * line)
{

}

static uint16_t usart1_readBytes(const char * buf, uint16_t numBytes)
{

}

static uint16_t usart1_readLine(const char * buf)
{

}

static uint16_t usart2_writeBytes(const char * bytes, uint16_t numBytes)
{

}

static uint16_t usart2_writeLine(const char * line)
{

}

static uint16_t usart2_readBytes(const char * buf, uint16_t numBytes)
{

}

static uint16_t usart2_readLine(const char * buf)
{

}

static uint16_t usart3_writeBytes(const char * bytes, uint16_t numBytes)
{

}

static uint16_t usart3_writeLine(const char * line)
{

}

static uint16_t usart3_readBytes(const char * buf, uint16_t numBytes)
{

}

static uint16_t usart3_readLine(const char * buf)
{

}

static uint16_t uart4_writeBytes(const char * bytes, uint16_t numBytes)
{

}

static uint16_t uart4_writeLine(const char * line)
{

}

static uint16_t uart4_readBytes(const char * buf, uint16_t numBytes)
{

}

static uint16_t uart4_readLine(const char * buf)
{

}

static uint16_t uart5_writeBytes(const char * bytes, uint16_t numBytes)
{

}

static uint16_t uart5_writeLine(const char * line)
{

}

static uint16_t uart5_readBytes(const char * buf, uint16_t numBytes)
{

}

static uint16_t uart5_readLine(const char * buf)
{

}

static uint16_t usart6_writeBytes(const char * bytes, uint16_t numBytes)
{

}

static uint16_t usart6_writeLine(const char * line)
{

}

static uint16_t usart6_readBytes(const char * buf, uint16_t numBytes)
{

}

static uint16_t usart6_readLine(const char * buf)
{

}

// Implementation of FS_DT_USARTDriver_t.
static uint16_t writeBytes(USART * usart, const char * bytes, uint16_t numBytes)
{

}

static uint16_t writeLine(USART * usart, const char * line)
{

}

static uint16_t readBytes(USART * usart, const char * buf, uint16_t numBytes)
{

}

static uint16_t readLine(USART * usart, const char * buf)
{

}

static void mainLoop(void * params)
{
  uint8_t i;
  USART * usart;
  unsigned char data;

  while(true)
  {
    // If the semaphore can't be taken, there's no work to do and the task will block.
    xSemaphoreTake(irqSyncSemaphore, 0);

    for(i = 0; i < 6; i++)
    {
      usart = &(usartList[i]);

      if(usart->enabled)
      {
        // Do tx tasks.
        if( bufferPop( &( usart->txBuffer ), &data ) )
        {
          USART_SendData(usart->peripheral, (uint16_t)data);
        }

        else
        {
          // If no data to send, prevent any further tx interrupts.
          USART_ITConfig(usart->peripheral, USART_IT_TXE, DISABLE);
        }

        // Check if a byte has been received.
        if( SET == USART_GetFlagStatus(usart->peripheral, USART_FLAG_RXNE) )
        {
          data = (unsigned char)USART_ReceiveData(usart->peripheral);
          bufferPush( &( usart->rxBuffer), data );
        }
      }
    }
  }
}

// Buffer functions.
static void bufferInit(USARTBuffer * buf)
{

}

static void bufferPush(USARTBuffer * buf, unsigned char data)
{
  TickType_t mutexTimeoutTicks;

  mutexTimeoutTicks = FS_STM32F4XXUSART_BUFFER_MUTEX_TIMEOUT_MS *
                      ( 1000 / configTICK_RATE_HZ );

  // Wait until the buffer is available.
  if( xSemaphoreTake(buf->mutex,  mutexTimeoutTicks) )
  {
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

  else
  {
    // TODO: Consider whether or not trapping this error is appropriate.
    while(true);
  }
}

static _Bool bufferPop(USARTBuffer *  buf, unsigned char * data)
{
   _Bool success;

  TickType_t mutexTimeoutTicks;

  mutexTimeoutTicks = FS_STM32F4XXUSART_BUFFER_MUTEX_TIMEOUT_MS *
                      ( 1000 / configTICK_RATE_HZ );

  // Wait until the buffer is available.
  if( xSemaphoreTake(buf->mutex,  mutexTimeoutTicks) )
  {
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

  else
  {
    return false;
  }
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

/*------------------------------------------------------------------------------
----------------------- START PRIVATE FUNCTIONS --------------------------------
------------------------------------------------------------------------------*/
