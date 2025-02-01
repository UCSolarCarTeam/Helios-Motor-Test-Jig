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
void sendADCValues(UART_HandleTypeDef* huart, DMA_HandleTypeDef* hdma, uint16_t* dma_adc_buf, uint8_t enable){
	if(enable){
        uint16_t adc_vals[8] = {0};
        
        memcpy(adc_vals, dma_adc_buf, 8*sizeof(uint16_t));

        char msg[80] = {0};
        sprintf(msg, "9999 %d %d %d %d %d %d %d %d 8888\r\n", adc_vals[0], adc_vals[1], adc_vals[2], adc_vals[3], adc_vals[4], adc_vals[5], adc_vals[6], adc_vals[7]);
        
        HAL_UART_DMAStop(huart);
        HAL_UART_Transmit_DMA(huart, (uint16_t*)msg, strlen(msg));
        
    //     HAL_UART_Transmit_DMA(huart, (uint8_t*)"9999 ", 5);

    //     for (int i = 0; i < 8; i++){
    //        sprintf(msg, "%d: %d ", i, adc_vals[i]);
    //        HAL_UART_Transmit_DMA(huart, (uint8_t*)msg, strlen(msg));
    //     }

    //    HAL_UART_Transmit_DMA(huart, (uint8_t*)"8888\r\n", 6);

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
