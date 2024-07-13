#include "uartRxTask.h"

uint8_t readback;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  //pass received character to main task
  osMessageQueuePut(uartRxQueue, &readback, 0, 0);

  //re-arm interrupt
  HAL_UART_Receive_IT(UART_HANDLER, &readback, 1);
}

uint8_t uartRxByteHandler(uint8_t* uartRxBuffer, uint8_t* uartRxBufferIndex, uint8_t newChar)
{
  uint8_t uartRxMessageReady = SOLAR_FALSE;
  // If the char is the end of message, terminate buffer.
  if (newChar == '\r' || *uartRxBufferIndex == UART_RX_BUFFER_SIZE)
  {
      // Null terminate and process
      uartRxBuffer[(*uartRxBufferIndex)++] = '\0';
      uartRxMessageReady = SOLAR_TRUE;
  } 
  else
  {
      uartRxBuffer[(*uartRxBufferIndex)++] = newChar;
  }

  return uartRxMessageReady;
};

void uartRxStringHandler(uint8_t* uartRxBuffer)
{
    if (strncmp((char*)uartRxBuffer, "preambleSymbols ", 16) == 0) {
        
    }
}

/**
  * @brief  Function implementing the uartRx thread.
  * @param  argument: Not used
  * @retval None
*/
void UartRxTask(void *argument)
{
  HAL_UART_Receive_IT(UART_HANDLER, &readback, 1);
  uint8_t uartRxBufferIndex = 0;
  uint8_t uartRxMessageReady = 0;
  uint8_t uartRxBuffer[UART_RX_BUFFER_SIZE];
  uint8_t newChar = 0;
  /* Infinite loop */
  for(;;)
  {
    osMessageQueueGet(uartRxQueue, &newChar, 0, osWaitForever);
    uartRxMessageReady = uartRxByteHandler(uartRxBuffer, &uartRxBufferIndex, newChar);
    if(uartRxMessageReady) 
    {
        uartRxMessageReady = SOLAR_FALSE;
        uint8_t message[UART_RX_BUFFER_SIZE];
        memcpy(message, uartRxBuffer, uartRxBufferIndex);
        uartRxBufferIndex = 0;
        osMessageQueuePut(debugTaskQueue, message, 0, osWaitForever);
    }
  }
}
