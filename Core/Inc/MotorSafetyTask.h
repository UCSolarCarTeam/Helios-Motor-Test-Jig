/*
 * MotorSafetyTask.h
 *
 *  Created on: Sep 21, 2024
 *      Author: dominic
 */

#ifndef INC_MOTORSAFETYTASK_H_
#define INC_MOTORSAFETYTASK_H_

#include "main.h"
#include "stdint.h"
#include <stdio.h>
#include <string.h>

void sendADCValues(UART_HandleTypeDef* huart, DMA_HandleTypeDef* hdma , uint16_t* dma_adc_buf, uint8_t enable);

uint8_t motorSafetyTask(uint16_t* dma_adc_buf);

#endif /* INC_MOTORSAFETYTASK_H_ */
