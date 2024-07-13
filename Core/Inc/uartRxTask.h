/*
 * uartRxTask.h
 *
 *  Created on: Sep. 08, 2023
 *      Author: Marcelo
 */
#ifndef UART_RX_TASK_H
#define UART_RX_TASK_H

#include <string.h>
#include "stm32l152xe.h"
#include "stm32l1xx_hal.h"
#include "system_defines.h"

extern UART_HandleTypeDef huart1;
extern osMessageQueueId_t uartRxQueue;
extern osMessageQueueId_t debugTaskQueue;

void UartRxTask(void *argument);

#endif /* UART_RX_TASK_H */
