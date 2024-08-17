/*
 * MotorControlTask.c
 *
 *  Created on: Jul 13, 2024
 *      Author: dominic
 */

#include "MotorControlTask.h"

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
    }
    else {
        // Command not finished
        return 0;
    }

    return 0;
}

/*
    * Parses the buffer into a Motor_cmd
    * 
    * @param buffer: The buffer to parse
    * @param start_index: The index of the buffer to start parsing
    * @param end_index: The index of the buffer to end parsing
    * @param split: 1 if the buffer is split into two parts, 0 if the buffer is not split
    * 
    * @return A Motor_cmd struct containing the parsed data

*/
void ParseMotorCommand(Motor_cmd* motor_cmd, uint8_t* buffer, uint8_t* end_index, uint8_t* last_message) {
    // TODO: Convert uart command to Motor_cmd
    memcpy(buffer, last_message, sizeof(uint8_t)*UART_BUF_LEN);
}
/*
    * Parse Motor_cmd into a CAN message
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
