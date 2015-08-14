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
#include <string.h>

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
static uint16_t usart1_rxBytesAvailable(void);
static uint16_t usart1_readBytes(const char * buf, uint16_t numBytes);
static uint16_t usart1_readLine(const char * buf);
static uint16_t usart1_readLineTruncate(const char * buf, uint16_t maxLen);

static uint16_t usart2_writeBytes(const char * bytes, uint16_t numBytes);
static uint16_t usart2_writeLine(const char * line);
static uint16_t usart2_rxBytesAvailable(void);
static uint16_t usart2_readBytes(const char * buf, uint16_t numBytes);
static uint16_t usart2_readLine(const char * buf);
static uint16_t usart2_readLineTruncate(const char * buf, uint16_t maxLen);

static uint16_t usart3_writeBytes(const char * bytes, uint16_t numBytes);
static uint16_t usart3_writeLine(const char * line);
static uint16_t usart3_rxBytesAvailable(void);
static uint16_t usart3_readBytes(const char * buf, uint16_t numBytes);
static uint16_t usart3_readLine(const char * buf);
static uint16_t usart3_readLineTruncate(const char * buf, uint16_t maxLen);

static uint16_t uart4_writeBytes(const char * bytes, uint16_t numBytes);
static uint16_t uart4_writeLine(const char * line);
static uint16_t uart4_rxBytesAvailable(void);
static uint16_t uart4_readBytes(const char * buf, uint16_t numBytes);
static uint16_t uart4_readLine(const char * buf);
static uint16_t uart4_readLineTruncate(const char * buf, uint16_t maxLen);

static uint16_t uart5_writeBytes(const char * bytes, uint16_t numBytes);
static uint16_t uart5_writeLine(const char * line);
static uint16_t uart5_rxBytesAvailable(void);
static uint16_t uart5_readBytes(const char * buf, uint16_t numBytes);
static uint16_t uart5_readLine(const char * buf);
static uint16_t uart5_readLineTruncate(const char * buf, uint16_t maxLen);

static uint16_t usart6_writeBytes(const char * bytes, uint16_t numBytes);
static uint16_t usart6_writeLine(const char * line);
static uint16_t usart6_rxBytesAvailable(void);
static uint16_t usart6_readBytes(const char * buf, uint16_t numBytes);
static uint16_t usart6_readLine(const char * buf);
static uint16_t usart6_readLineTruncate(const char * buf, uint16_t maxLen);

// Implementation of FS_DT_USARTDriver_t.
static uint16_t writeBytes(USART * usart, const char * bytes, uint16_t numBytes);
static uint16_t writeLine(USART * usart, const char * line);
static uint16_t rxBytesAvailable(USART * usart);
static uint16_t readBytes(USART * usart, const char * buf, uint16_t numBytes);
static uint16_t readLine(USART * usart, const char * buf);
static uint16_t readLineTruncate(USART * usart, const char * buf, uint16_t maxLen);

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
static FS_STM32F4xxPinMux_t * pinMux;

/*------------------------------------------------------------------------------
--------------------- END PRIVATE GLOBAL VARIABLES -----------------------------
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
------------------------ START PUBLIC FUNCTIONS --------------------------------
------------------------------------------------------------------------------*/

void FS_STM32F4xxUSART_InitStructInit(FS_STM32F4xxUSART_InitStruct_t * initStruct)
{
  initStruct->instance = NULL;
  FS_STM32F4xxUSART_PeriphInitStructInit( &( initStruct->usart1InitStruct ) );
  FS_STM32F4xxUSART_PeriphInitStructInit( &( initStruct->usart2InitStruct ) );
  FS_STM32F4xxUSART_PeriphInitStructInit( &( initStruct->usart3InitStruct ) );
  FS_STM32F4xxUSART_PeriphInitStructInit( &( initStruct->uart4InitStruct ) );
  FS_STM32F4xxUSART_PeriphInitStructInit( &( initStruct->uart5InitStruct ) );
  FS_STM32F4xxUSART_PeriphInitStructInit( &( initStruct->usart6InitStruct ) );
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

  // Bind redirection functions to the driver instance.
  initStruct->instance->usart1.readBytes = usart1_readBytes;
  initStruct->instance->usart1.readLine = usart1_readLine;
  initStruct->instance->usart1.writeBytes = usart1_writeBytes;
  initStruct->instance->usart1.writeLine = usart1_writeLine;

  initStruct->instance->usart2.readBytes = usart2_readBytes;
  initStruct->instance->usart2.readLine = usart2_readLine;
  initStruct->instance->usart2.writeBytes = usart2_writeBytes;
  initStruct->instance->usart2.writeLine = usart2_writeLine;

  initStruct->instance->usart3.readBytes = usart3_readBytes;
  initStruct->instance->usart3.readLine = usart3_readLine;
  initStruct->instance->usart3.writeBytes = usart3_writeBytes;
  initStruct->instance->usart3.writeLine = usart3_writeLine;

  initStruct->instance->uart4.readBytes = uart4_readBytes;
  initStruct->instance->uart4.readLine = uart4_readLine;
  initStruct->instance->uart4.writeBytes = uart4_writeBytes;
  initStruct->instance->uart4.writeLine = uart4_writeLine;

  initStruct->instance->uart5.readBytes = uart5_readBytes;
  initStruct->instance->uart5.readLine = uart5_readLine;
  initStruct->instance->uart5.writeBytes = uart5_writeBytes;
  initStruct->instance->uart5.writeLine = uart5_writeLine;

  initStruct->instance->usart6.readBytes = usart6_readBytes;
  initStruct->instance->usart6.readLine = usart6_readLine;
  initStruct->instance->usart6.writeBytes = usart6_writeBytes;
  initStruct->instance->usart6.writeLine = usart6_writeLine;

  returns.mainLoop = mainLoop;
  returns.success =  true;
  return returns;
}

void FS_STM32F4xxUSART_PeriphInitStructInit(FS_STM32F4xxUSART_PeriphInitStruct_t * initStruct)
{
  initStruct->initialise = false;
  initStruct->peripheral = NULL;
  initStruct->txd.bit = FS_STM32F4xxPinNoPin;
  initStruct->txd.port = FS_STM32F4xxPortNoPort;
  initStruct->rxd.bit = FS_STM32F4xxPinNoPin;
  initStruct->rxd.port = FS_STM32F4xxPortNoPort;
  initStruct->cts.bit = FS_STM32F4xxPinNoPin;
  initStruct->cts.port = FS_STM32F4xxPortNoPort;
  initStruct->rts.bit = FS_STM32F4xxPinNoPin;
  initStruct->rts.port = FS_STM32F4xxPortNoPort;
  initStruct->sclk.bit = FS_STM32F4xxPinNoPin;
  initStruct->sclk.port = FS_STM32F4xxPortNoPort;
  initStruct->txBufferSizeBytes = 0;
  initStruct->rxBufferSizeBytes = 0;
  USART_StructInit(initStruct->stInitStruct);
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
  if(usartList[0].enabled)
  {
    return writeLine( &( usartList[0] ), line );
  }

  else
  {
    return 0;
  }
}

static uint16_t usart1_rxBytesAvailable(void)
{
  if(usartList[0].enabled)
  {
    return rxBytesAvailable( &( usartList[0] ) );
  }

  else
  {
    return 0;
  }

}

static uint16_t usart1_readBytes(const char * buf, uint16_t numBytes)
{
  // Redirect to implementation function if peripheral enabled.
  if(usartList[0].enabled)
  {
    return readBytes( &( usartList[0] ), buf, numBytes );
  }

  else
  {
    return 0;
  }
}

static uint16_t usart1_readLine(const char * buf)
{
  // Redirect to implementation function if peripheral enabled.
  if(usartList[0].enabled)
  {
    return readLine( &( usartList[0] ), buf );
  }

  else
  {
    return 0;
  }
}

static uint16_t usart1_readLineTruncate(const char * buf, uint16_t maxLen)
{
  if(usartList[0].enabled)
  {
    return readLineTruncate( &( usartList[0] ), buf, maxLen );
  }

  else
  {
    return 0;
  }
}

static uint16_t usart2_writeBytes(const char * bytes, uint16_t numBytes)
{
  // Redirect to implementation function if peripheral enabled.
  if(usartList[1].enabled)
  {
    return writeBytes( &( usartList[1] ), bytes, numBytes );
  }

  else
  {
    return 0;
  }
}

static uint16_t usart2_writeLine(const char * line)
{
  // Redirect to implementation function if peripheral enabled.
  if(usartList[1].enabled)
  {
    return writeLine( &( usartList[1] ), line );
  }

  else
  {
    return 0;
  }
}

static uint16_t usart2_rxBytesAvailable(void)
{
  if(usartList[1].enabled)
  {
    return rxBytesAvailable( &( usartList[1] ) );
  }

  else
  {
    return 0;
  }
}

static uint16_t usart2_readBytes(const char * buf, uint16_t numBytes)
{
  // Redirect to implementation function if peripheral enabled.
  if(usartList[1].enabled)
  {
    return readBytes( &( usartList[1] ), buf, numBytes );
  }

  else
  {
    return 0;
  }
}

static uint16_t usart2_readLine(const char * buf)
{
  // Redirect to implementation function if peripheral enabled.
  if(usartList[1].enabled)
  {
    return readLine( &( usartList[1] ), buf );
  }

  else
  {
    return 0;
  }
}

static uint16_t usart2_readLineTruncate(const char * buf, uint16_t maxLen)
{
  if(usartList[1].enabled)
  {
    return readLineTruncate( &( usartList[1] ), buf, maxLen );
  }

  else
  {
    return 0;
  }
}

static uint16_t usart3_writeBytes(const char * bytes, uint16_t numBytes)
{
  // Redirect to implementation function if peripheral enabled.
  if(usartList[2].enabled)
  {
    return writeBytes( &( usartList[2] ), bytes, numBytes );
  }

  else
  {
    return 0;
  }
}

static uint16_t usart3_writeLine(const char * line)
{
  // Redirect to implementation function if peripheral enabled.
  if(usartList[2].enabled)
  {
    return writeLine( &( usartList[2] ), line );
  }

  else
  {
    return 0;
  }
}

static uint16_t usart3_rxBytesAvailable(void)
{
  if(usartList[2].enabled)
  {
    return rxBytesAvailable( &( usartList[2] ) ) ;
  }

  else
  {
    return 0;
  }
}

static uint16_t usart3_readBytes(const char * buf, uint16_t numBytes)
{
  // Redirect to implementation function if peripheral enabled.
  if(usartList[2].enabled)
  {
    return readBytes( &( usartList[2] ), buf, numBytes );
  }

  else
  {
    return 0;
  }
}

static uint16_t usart3_readLine(const char * buf)
{
  // Redirect to implementation function if peripheral enabled.
  if(usartList[2].enabled)
  {
    return readLine( &( usartList[2] ), buf );
  }

  else
  {
    return 0;
  }
}

static uint16_t usart3_readLineTruncate(const char * buf, uint16_t maxLen)
{
  if(usartList[2].enabled)
  {
    return readLineTruncate( &( usartList[2] ), buf, maxLen );
  }

  else
  {
    return 0;
  }
}

static uint16_t uart4_writeBytes(const char * bytes, uint16_t numBytes)
{
  // Redirect to implementation function if peripheral enabled.
  if(usartList[3].enabled)
  {
    return writeBytes( &( usartList[3] ), bytes, numBytes );
  }

  else
  {
    return 0;
  }
}

static uint16_t uart4_writeLine(const char * line)
{
  // Redirect to implementation function if peripheral enabled.
  if(usartList[3].enabled)
  {
    return writeLine( &( usartList[3] ), line );
  }

  else
  {
    return 0;
  }
}

static uint16_t uart4_rxBytesAvailable(void)
{
  if(usartList[3].enabled)
  {
    return rxBytesAvailable( &( usartList[3] ) );
  }

  else
  {
    return 0;
  }
}

static uint16_t uart4_readBytes(const char * buf, uint16_t numBytes)
{
  // Redirect to implementation function if peripheral enabled.
  if(usartList[3].enabled)
  {
    return readBytes( &( usartList[3] ), buf, numBytes );
  }

  else
  {
    return 0;
  }
}

static uint16_t uart4_readLine(const char * buf)
{
  // Redirect to implementation function if peripheral enabled.
  if(usartList[3].enabled)
  {
    return readLine( &( usartList[3] ), buf );
  }

  else
  {
    return 0;
  }
}

static uint16_t uart4_readLineTruncate(const char * buf, uint16_t maxLen)
{
  if(usartList[3].enabled)
  {
    return readLineTruncate( &( usartList[3] ), buf, maxLen );
  }

  else
  {
    return 0;
  }
}

static uint16_t uart5_writeBytes(const char * bytes, uint16_t numBytes)
{
  // Redirect to implementation function if peripheral enabled.
  if(usartList[4].enabled)
  {
    return writeBytes( &( usartList[4] ), bytes, numBytes );
  }

  else
  {
    return 0;
  }
}

static uint16_t uart5_writeLine(const char * line)
{
  // Redirect to implementation function if peripheral enabled.
  if(usartList[4].enabled)
  {
    return writeLine( &( usartList[4] ), line );
  }

  else
  {
    return 0;
  }
}

static uint16_t uart5_rxBytesAvailable(void)
{
  if(usartList[4].enabled)
  {
    return rxBytesAvailable( &( usartList[4] ) );
  }

  else
  {
    return 0;
  }
}


static uint16_t uart5_readBytes(const char * buf, uint16_t numBytes)
{
  // Redirect to implementation function if peripheral enabled.
  if(usartList[4].enabled)
  {
    return readBytes( &( usartList[4] ), buf, numBytes );
  }

  else
  {
    return 0;
  }
}

static uint16_t uart5_readLine(const char * buf)
{
  // Redirect to implementation function if peripheral enabled.
  if(usartList[4].enabled)
  {
    return readLine( &( usartList[4] ), buf );
  }

  else
  {
    return 0;
  }
}

static uint16_t uart5_readLineTruncate(const char * buf, uint16_t maxLen)
{
  if(usartList[4].enabled)
  {
    return readLineTruncate( &( usartList[4] ), buf, maxLen );
  }

  else
  {
    return 0;
  }
}

static uint16_t usart6_writeBytes(const char * bytes, uint16_t numBytes)
{
  // Redirect to implementation function if peripheral enabled.
  if(usartList[5].enabled)
  {
    return writeBytes( &( usartList[5] ), bytes, numBytes );
  }

  else
  {
    return 0;
  }
}

static uint16_t usart6_writeLine(const char * line)
{
  // Redirect to implementation function if peripheral enabled.
  if(usartList[5].enabled)
  {
    return writeLine( &( usartList[5] ), line );
  }

  else
  {
    return 0;
  }
}

static uint16_t usart6_rxBytesAvailable(void)
{
  if(usartList[5].enabled)
  {
    return rxBytesAvailable( &( usartList[5] ) );
  }

  else
  {
    return 0;
  }
}


static uint16_t usart6_readBytes(const char * buf, uint16_t numBytes)
{
  // Redirect to implementation function if peripheral enabled.
  if(usartList[5].enabled)
  {
    return readBytes( &( usartList[5] ), buf, numBytes );
  }

  else
  {
    return 0;
  }
}

static uint16_t usart6_readLine(const char * buf)
{
  // Redirect to implementation function if peripheral enabled.
  if(usartList[5].enabled)
  {
    return readLine( &( usartList[5] ), buf );
  }

  else
  {
    return 0;
  }
}

static uint16_t usart6_readLineTruncate(const char * buf, uint16_t maxLen)
{
  if(usartList[5].enabled)
  {
    return readLineTruncate( &( usartList[5] ), buf, maxLen );
  }

  else
  {
    return 0;
  }
}

// Implementation of FS_DT_USARTDriver_t.
static uint16_t writeBytes(USART * usart, const char * bytes, uint16_t numBytes)
{
  uint16_t spaceAfterTail, bytesToCopy, overflowBytes;

  // Check that the number of bytes to write won't overwhelm the buffer.
  if(numBytes > usart->txBuffer.length)
  {
    return 0;
  }

  // Get the buffer's mutex.
  if( xSemaphoreTake( usart->txBuffer.mutex,
                      FS_STM32F4XXUSART_BUFFER_MUTEX_TIMEOUT_TICKS ) )
  {
    // Calculate how much space exists between the tail pointer and the end of the buffer.
    spaceAfterTail = usart->txBuffer.base + usart->txBuffer.length - usart->txBuffer.tail;

    /*
    If the bytes to write fit between the current tail and the end of the buffer,
    we can simply block copy them.
    */
    if(spaceAfterTail <= numBytes)
    {
      memcpy( &( masterBuffer[usart->txBuffer.tail] ), bytes, numBytes);

      /*
      If the number of bytes copied was an exact fit for the remaining space,
      simply wrap the tail pointer.
      */
      if(numBytes == spaceAfterTail)
      {
        usart->txBuffer.tail = usart->txBuffer.base;
      }

      // Otherwise, move the tail up appropriately.
      else
      {
        usart->txBuffer.tail += numBytes;
      }
    }

    // If a wrap is required, split the bytes appropriately into two groups.
    else
    {
      overflowBytes = numBytes - spaceAfterTail;

      // Insert the first block at the end of the buffer.
      memcpy( &( masterBuffer[usart->txBuffer.tail] ), bytes, spaceAfterTail);

      // Insert the remaining bytes at the beginning of the buffer.
      memcpy( &( masterBuffer[usart->txBuffer.base] ), &( bytes[spaceAfterTail] ), overflowBytes);

      usart->txBuffer.tail = usart->txBuffer.base + overflowBytes;
    }

    // Deal with the monitoring variables:

    // If the write hasn't caused any data loss...
    if( ( usart->txBuffer.fillLevel + numBytes ) <= usart->txBuffer.length )
    {
      usart->txBuffer.fillLevel += numBytes;
    }

    // If data loss occurred, by definition the buffer is now full.
    else
    {
      usart->txBuffer.fillLevel = usart->txBuffer.length;
    }

    if(usart->txBuffer.highWater > usart->txBuffer.fillLevel)
    {
      usart->txBuffer.highWater = usart->txBuffer.highWater;
    }
  }

  // Mutex timed out.
  else
  {
    return 0;
  }
}

static uint16_t writeLine(USART * usart, const char * line)
{
  size_t length;

  /*
  First, check that the string will not overwhelm the buffer. Note
  that strlen does not account for the NULL terminator, however,
  we will be replacing it with a \n anyway so the value of length
  does not need to be modified.
  */
  length = strlen(line);

  if(length > usart->txBuffer.length)
  {
    return 0;
  }

  if( writeBytes(usart, line, length - 1) )
  {
    bufferPush( &( usart->txBuffer ), '\n');
    return length;
  }

  // writeBytes failed - indicate to the caller.
  else
  {
    return 0;
  }
}

static uint16_t rxBytesAvailable(USART * usart)
{
  // TODO: Work out if we need to get the mutex or not here!!
  return usart->rxBuffer.fillLevel;
}

static uint16_t readBytes(USART * usart, const char * buf, uint16_t numBytes)
{

}

static uint16_t readLine(USART * usart, const char * buf)
{

}

static uint16_t readLineTruncate(USART * usart, const char * buf, uint16_t maxLen)
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

  // Wait until the buffer is available.
  if( xSemaphoreTake(buf->mutex,  FS_STM32F4XXUSART_BUFFER_MUTEX_TIMEOUT_TICKS) )
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

  // Wait until the buffer is available.
  if( xSemaphoreTake(buf->mutex,  FS_STM32F4XXUSART_BUFFER_MUTEX_TIMEOUT_TICKS) )
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
