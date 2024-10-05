/*
 * MotorSafetyTask.c
 *
 *  Created on: Sep 21, 2024
 *      Author: dominic
 */

#include "MotorSafetyTask.h"
#include "main.h"

// TEMP VALUES
#define M1_VOLTAGE_LIMIT 1000
#define M2_VOLTAGE_LIMIT 1000
#define SUPPLY_VOLTAGE_LIMIT 1000
#define MPPT_CURRENT_LIMIT 1000
#define M1_CURRENT_LIMIT 1000
#define M2_CURRENT_LIMIT 1000
#define SUPPLY_CURRENT_LIMIT 1000
#define LOAD_CURRENT_LIMIT 1000

extern uint16_t dma_adc_buf[ADC_BUF_LEN];

void MotorSafetyTask(void) {
    if (dma_adc_buf[0] > M1_VOLTAGE_LIMIT) {
        // Error
    }
    if (dma_adc_buf[1] > M2_VOLTAGE_LIMIT) {
        // Error
    }
    if (dma_adc_buf[2] > SUPPLY_VOLTAGE_LIMIT) {
        // Error
    }
    if (dma_adc_buf[3] > MPPT_CURRENT_LIMIT) {
        // Error
    }
    if (dma_adc_buf[4] > M1_CURRENT_LIMIT) {
        // Error
    }
    if (dma_adc_buf[5] > M2_CURRENT_LIMIT) {
        // Error
    }
    if (dma_adc_buf[6] > SUPPLY_CURRENT_LIMIT) {
        // Error
    }
    if (dma_adc_buf[7] > LOAD_CURRENT_LIMIT) {
        // Error
    }
}