/*
 * system_defines.h
 *
 * constants used for system wide operations
 * 
 *  Created on: Sep 08, 2023
 *      Author: Marcelo
 */
#ifndef SYSTEM_DEFINES_H
#define SYSTEM_DEFINES_H

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#define SOLAR_TRUE 1
#define SOLAR_FALSE 0

#define UART_HANDLER &huart1

#define DEFAULT_TASK_STACK_SIZE 128 * 10

#define DEBUG_TAKE_MAX_TIME_MS 10
#define DEBUG_PRINT_MAX_SIZE 256

#define UART_RX_DATA_QUEUE_COUNT 16
#define UART_TX_DATA_QUEUE_COUNT 16
#define DEBUG_QUEUE_COUNT 10
#define TOGGLE_QUEUE_COUNT 5

#define UART_RX_BUFFER_SIZE 256

extern osMessageQueueId_t uartTxQueue;
extern osMutexId_t vaListMutexHandle;

typedef enum RadioCommandType {
    SET_COMMAND = 0,
    GET_COMMAND = 1,
    WRITE_BUFFER = 2,
    READ_BUFFER = 3,
    WRITE_REGISTER = 4,
    READ_REGISTER = 5,
    TRANSMIT = 6,
    RECEIVE = 7,
    STOP_RADIO = 8,
    START_RADIO = 9,
    RESTART_RADIO = 10,
    OVERWRITE_PARAM = 11
} RadioCommandType;

typedef struct RadioCommand {
    uint16_t size;
    RadioCommandType command;
    uint8_t address;
    uint8_t* data;
} RadioCommand;

typedef struct UartTxData {
    uint8_t size;
    uint8_t* data;
} UartTxData;

void *solarMalloc(size_t xWantedSize);
void solarFree(void *ptr);

void solarPrint(const char* str, ...);

uint8_t isValidDecimalCharacter(char* value);
uint8_t strToIntArray(char* str, int* dest, uint8_t* size);

#endif /* SYSTEM_DEFINES_H */
