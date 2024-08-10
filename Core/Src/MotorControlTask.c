/*
 * MotorControlTask.c
 *
 *  Created on: Jul 13, 2024
 *      Author: dominic
 */

#include "MotorControlTask.h"

extern uint16_t dma_uart_buf;

/*
    * Checks if the buffer contains a complete command
    * 
    * @param buffer: The buffer to check
    * @param buffer_index: The index of the buffer to check
    * 
    * @return 0 if the buffer does not contain a complete command,  1 if the buffer contains a complete command, 2 if the buffer is full
*/
uint8_t CheckBuffer(uint8_t* buffer, uint8_t* buffer_index) {
    if ((uint8_t)(*buffer_index) == '\r') {
        // Command finished
        return 1;

    } else if ((uint32_t)buffer_index == (buffer[0] + UART_BUF_LEN - 1)){
        // Command not finished, buffer full
        return 2;
    }
    else {
        // Command not finished
        return 0;
    }

    return 0;
}

/*
    * Parses the buffer into a Motor_cmd struct
    * 
    * @param buffer: The buffer to parse
    * @param start_index: The index of the buffer to start parsing
    * @param end_index: The index of the buffer to end parsing
    * @param split: 1 if the buffer is split into two parts, 0 if the buffer is not split
    * 
    * @return A Motor_cmd struct containing the parsed data

*/
void ParseMotorCommand(Motor_cmd* motor_cmd, uint8_t* buffer, uint8_t* start_index, uint8_t* end_index, uint8_t split) {
    // TODO: Convert uart command to Motor_cmd
    if (split == 1) {
        uint16_t temp_buffer[UART_BUF_LEN] = {0};
        uint16_t temp_buffer_index = 0;
        while (temp_buffer_index < UART_BUF_LEN) {
            temp_buffer[temp_buffer_index] = buffer[*start_index + temp_buffer_index];
            temp_buffer_index++;
        }

        // TODO: Parse temp_buffer into motor_cmd
        return;

    } else {
        // TODO: Parse buffer from start_index to end_index into motor_cmd
        return;
    }
}
/*
    * Sends the motor command using CAN
    * 
    * @param motor_cmd: The motor command to send
*/
void SendMotorCommand(Motor_cmd* motor_cmd) {
    uint8_t m1_message[8] = {0}; // 8 bytes for CAN m1_message
    uint8_t m2_message[8] = {0}; // 8 bytes for CAN m2_message

    if(motor_cmd->m1_status == 1){
        m1_message[1] |= motor_cmd->m1_dir << 7;           // Direction
        m1_message[1] = motor_cmd->m1_val & 0xFF;          // Control Value Top
        m1_message[0] = (motor_cmd->m1_val >> 8) & 0xFF;   // Control Value Bottom

        uint8_t byte_3 = 0b00100000;            // bit 5 = 1 for software enable
        byte_3 |= motor_cmd->m1_control;        // Control Mode 0 = Default, 1 = Torque, 2 = Speed
        byte_3 |= (motor_cmd->m1_mode << 2);    // Motor Mode   0 = Default, 1 = Boost,  2 = Reverse, 3 = Regen

        m1_message[2] = byte_3;
    }

    if(motor_cmd->m2_status == 1){
        m2_message[1] |= motor_cmd->m2_dir << 7;           // Direction
        m2_message[1] = motor_cmd->m2_val & 0xFF;          // Control Value Top
        m2_message[0] = (motor_cmd->m2_val >> 8) & 0xFF;   // Control Value Bottom

        uint8_t byte_3 = 0b00100000;            // bit 5 = 1 for software enable
        byte_3 |= motor_cmd->m2_control;        // Control Mode 0 = Default, 1 = Torque, 2 = Speed
        byte_3 |= motor_cmd->m2_mode << 2;      // Motor Mode   0 = Default, 1 = Boost,  2 = Reverse, 3 = Regen

        m2_message[2] = byte_3;
    }
}
