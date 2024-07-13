#include "uartTxTask.h"

/**
  * @brief  Function implementing the uartTx thread.
  * @param  argument: Not used
  * @retval None
*/
void UartTxTask(void *argument)
{
  /* Infinite loop */
  for(;;)
  {
    UartTxData uartTxData = {0};
    osMessageQueueGet(uartTxQueue, &uartTxData, 0, osWaitForever);
    if (HAL_UART_Transmit(UART_HANDLER, uartTxData.data, uartTxData.size, 100000) != HAL_OK){
      //add error handler
    };
    solarFree(uartTxData.data);
  }
}
