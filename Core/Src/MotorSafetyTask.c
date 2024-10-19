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
void sendADCValues(UART_HandleTypeDef* huart, uint16_t* dma_adc_buf, uint8_t enable) {
	if(enable){
        uint16_t current_adc_buf[ADC_BUF_LEN] = {0};
		memcpy(current_adc_buf, dma_adc_buf, sizeof(uint16_t)*ADC_BUF_LEN);

		char msg[50] = {0};
        sprintf(msg, "0000\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n%d\n000\r\n", current_adc_buf[0], current_adc_buf[1], current_adc_buf[2], current_adc_buf[3], current_adc_buf[4], current_adc_buf[5], current_adc_buf[6], current_adc_buf[7]);
        HAL_UART_Transmit(huart, (uint8_t*)msg, strlen(msg), 100);
	}

	return;
}

