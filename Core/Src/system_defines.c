/*
 * system_defines.c
 *
 *  Created on: Sep 14, 2023
 *      Author: Marcelo Li
 */

#include "system_defines.h"

void *solarMalloc(size_t xWantedSize)
{
    void *ret = pvPortMalloc(xWantedSize);
    return ret;
}

/**
 * @brief Free function
 * @param ptr Pointer to the data to free
 */
void solarFree(void* ptr) {
    vPortFree(ptr);
}

void solarPrint(const char* str, ...) {
    //Try to take the VA list mutex
    if(osMutexWait(vaListMutexHandle, 0) == osOK)
    {
        // If we have a message, and can use VA list, extract the string into a new buffer, and null terminate it
        uint8_t str_buffer[DEBUG_PRINT_MAX_SIZE];
        va_list argument_list;
        va_start(argument_list, str);
        int16_t buflen = vsnprintf((char *)(str_buffer), sizeof(str_buffer) - 1, str, argument_list);
        va_end(argument_list);
        if (buflen > 0) {
            str_buffer[buflen] = '\0';
        }

        // Release the VA List Mutex
        osMutexRelease(vaListMutexHandle);

        //Generate a command
        UartTxData uartTxData = {.size = buflen};

        //Copy data into the command
        uartTxData.data = solarMalloc(buflen);
        memcpy(uartTxData.data, str_buffer, buflen); // null character is ignored, since terminal adds it anyways

        //Send this packet off to the UART Task
        osStatus_t ret = osMessageQueuePut(uartTxQueue, &uartTxData, 0, 100);
        if(ret != osOK)
            solarFree(uartTxData.data);
    }
    else
    {
        //TODO: add error functionality reporting functionality
    }
}

//check if the passed character is a ascii decimal value (0-9)
uint8_t isValidDecimalCharacter(char* value) {
    //manually check if value is between ascii values for decimal numbers
    if((uint8_t)(*value) > 47 && (uint8_t)(*value) < 58)
        return SOLAR_TRUE;
    return SOLAR_FALSE;
}


//Separate each integer of a string separated by whitespace into an array of ints
uint8_t strToIntArray(char* str, int* dest, uint8_t* size) {
    int temp = 0;
    int index = 0;
    
    //Iterate untilt he end of the string
    while(*str != 0) {
        //If its a whitespace, finish the current number and start converting a new number
        if(*str == 32)
        {
            dest[index++] = temp;
            temp = 0;
            str++;
            continue;
        }

        //If you ever find an invalid decimal character, exit and report false
        if(!isValidDecimalCharacter(str))
            return SOLAR_FALSE;
        
        //if its valid and not the end of a number, multiply your current number by 10 and add the new digit
        temp *= 10;
        temp += (int)(*str) - 48;
        str++;
    }
    dest[index] = temp;
    *size = index + 1;

    //we reached the end of the string without faults
    return SOLAR_TRUE;
}
