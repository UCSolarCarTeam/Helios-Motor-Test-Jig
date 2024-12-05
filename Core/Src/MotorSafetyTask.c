/*
 * MotorSafetyTask.c
 *
 *  Created on: Sep 21, 2024
 *      Author: dominic
 */

#include "MotorSafetyTask.h"
#include "main.h"

/*
    * Sends ADC values over UART
    * 
    * @param huart: The UART handle
    * @param dma_adc_buf: The ADC buffer
    * @param en: enable
    * 
    * @return 0 if the motor command is within safety limits, 1 if the motor command is not within safety limits
*/
void sendADCValues(UART_HandleTypeDef* huart, uint16_t* dma_adc_buf, uint8_t enable){
	if(enable){
		char msg[50] = {0};
        sprintf(msg, "9999\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\r\n", dma_adc_buf[0], dma_adc_buf[1], dma_adc_buf[2], dma_adc_buf[3], dma_adc_buf[4], dma_adc_buf[5], dma_adc_buf[6], dma_adc_buf[7]);
        HAL_UART_Transmit_DMA(huart, (uint8_t*)msg, strlen(msg));
	} else {

	}

	return;
}

/*
    * Checks if the ADC values are within safety limits
    * 
    * @param dma_adc_buf: The ADC buffer
    * 
    * @return 0 if the motor command is within safety limits, 1 if the motor command is not within safety limits
*/
uint8_t checkADCValues(const uint16_t* dma_adc_buf) {
    // Get data Torque, Speed data from CAN
    
    // Get expected current, voltage, torque, speed from formula

    // Get actual current, voltage, torque, speed from ADC values
    // Do necessary conversions

    // Check if the values are within safety limits
    
    return 0;
}

uint8_t motorSafetyTask(uint16_t* dma_adc_buf){
	return 1;
}
