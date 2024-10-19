/*
 * MotorControlTask.c
 *
 *  Created on: Jul 13, 2024
 *      Author: dominic
 */

#include "MotorControlTask.h"
#include "MotorSafetyTask.h"

Motor_cmd motor_cmd_init(void){
    Motor_cmd motor_cmd;

    motor_cmd.m1_status = 0;
    motor_cmd.m1_control = 0;
    motor_cmd.m1_mode = 0;
    motor_cmd.m1_dir = 0;
    motor_cmd.m1_val = 0;
    
    motor_cmd.m2_status = 0;
    motor_cmd.m2_control = 0;
    motor_cmd.m2_mode = 0;
    motor_cmd.m2_dir = 0;
    motor_cmd.m2_val = 0;

    return motor_cmd;
}

/*
    * Checks if the buffer contains a complete command
    * 
    * @param buffer: The buffer to check
    * @param buffer_index: The index of the buffer to check
    * 
    * @return 0 if the buffer does not contain a complete command, 1 if the buffer contains a complete command
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
    * @param motor_cmd: The Motor_cmd struct to store the parsed data
    * @param buffer: The buffer to parse
    * @param end_index: The index of the buffer to end parsing
    * @param last_message: The last message received
    * 
    * @return A Motor_cmd struct containing the parsed data

*/
uint8_t ParseMotorCommand(Motor_cmd* motor_cmd, uint8_t* buffer, uint8_t* last_message, uint8_t* adc_log_enable, Motor_cmd* last_motor_cmd, UART_HandleTypeDef* huart) {
    for(int i = 0; i < UART_BUF_LEN; i++){
        last_message[i] = buffer[i];
    }

    char motor[5] = {0};
    char option[20] = {0};
    int16_t number = {0};

    // format: m[id] [option] [number]
    sscanf((char *)last_message, "%s %s %hd", motor, option, &number);

    char update_msg[50] = {0};

    // Motor 1
    if (strcmp(motor, "m1") == 0){
        // Status
        if(strcmp(option, "on") == 0){
            motor_cmd->m1_status = 1;
            sprintf(update_msg, "Motor 1 Status: %d -> %d\r\n", last_motor_cmd->m1_status, motor_cmd->m1_status);
        } 
        else if(strcmp(option, "off") == 0){
            motor_cmd->m1_status = 0;
            sprintf(update_msg, "Motor 1 Status: %d -> %d\r\n", last_motor_cmd->m1_status, motor_cmd->m1_status);
        } 
        
        // Control Mode and Value
        else if(strcmp(option, "torque") == 0){
            if (number > 100){
                number = 100;
            } else if(number < -100){
                number = -100;
            }

            motor_cmd->m1_val = number;
            motor_cmd->m1_control = 1;

            if(number < 0){
                motor_cmd->m1_dir = 1;
            } else {
                motor_cmd->m1_dir = 0;
            }

            sprintf(update_msg, "Motor 1 Torque: %d%% -> %d%%\r\n", last_motor_cmd->m1_val, motor_cmd->m1_val);

        }
        else if(strcmp(option, "speed") == 0){
            if(number > 10000){
                number = 10000;
            } else if(number < -10000){
                number = -10000;
            }

            motor_cmd->m1_val = number;
            motor_cmd->m1_control = 2;
            if (number < 0){
                motor_cmd->m1_dir = 1;
            } else {
                motor_cmd->m1_dir = 0;
            }
            sprintf(update_msg, "Motor 1 Speed: %d RPM -> %d RPM\r\n", last_motor_cmd->m1_val/10, motor_cmd->m1_val/10);
        } 
        
        // Motor Mode
        else if(strcmp(option, "default") == 0){
            motor_cmd->m1_mode = 0;
            sprintf(update_msg, "Motor 1 Mode: %d -> %d\r\n", last_motor_cmd->m1_mode, motor_cmd->m1_mode);
        } else if(strcmp(option, "boost") == 0){
            motor_cmd->m1_mode = 1;
            sprintf(update_msg, "Motor 1 Mode: %d -> %d\r\n", last_motor_cmd->m1_mode, motor_cmd->m1_mode);
        } else if(strcmp(option, "reverse") == 0){
            motor_cmd->m1_mode = 2;
            sprintf(update_msg, "Motor 1 Mode: %d -> %d\r\n", last_motor_cmd->m1_mode, motor_cmd->m1_mode);
        } else if(strcmp(option, "regen") == 0){
            motor_cmd->m1_mode = 3;
            sprintf(update_msg, "Motor 1 Mode: %d -> %d\r\n", last_motor_cmd->m1_mode, motor_cmd->m1_mode);
        }

        else {
            // unsupported option
            return -3;
        }
    }

    // Motor 2
    else if(strcmp(motor, "m2") == 0){
        // Status
        if(strcmp(option, "on") == 0) {
            motor_cmd->m2_status = 1;
            sprintf(update_msg, "Motor 2 Status: %d -> %d\r\n", last_motor_cmd->m2_status, motor_cmd->m2_status);
        } 
        else if(strcmp(option, "off") == 0){
            motor_cmd->m2_status = 0;
            sprintf(update_msg, "Motor 2 Status: %d -> %d\r\n", last_motor_cmd->m2_status, motor_cmd->m2_status);
        } 
        
        // Control Mode and Value
        else if(strcmp(option, "torque") == 0){
            if (number > 100){
                number = 100;
            } else if(number < -100){
                number = -100;
            }
            motor_cmd->m2_val = number;
            motor_cmd->m2_control = 1;
            if(number < 0){
                motor_cmd->m2_dir = 1;
            } else {
                motor_cmd->m2_dir = 0;
            }
            sprintf(update_msg, "Motor 2 Torque: %d%% -> %d%%\r\n", last_motor_cmd->m2_val, motor_cmd->m2_val);
        }
        else if(strcmp(option, "speed") == 0){
            if(number > 10000){
                number = 10000;
            } else if(number < -10000){
                number = -10000;
            }

            motor_cmd->m2_val = number;
            motor_cmd->m2_control = 2;
            if (number < 0){
                motor_cmd->m2_dir = 1;
            } else {
                motor_cmd->m2_dir = 0;
            }
            sprintf(update_msg, "Motor 2 Speed: %d RPM -> %d RPM\r\n", last_motor_cmd->m2_val/10, motor_cmd->m2_val/10);
        }

        // Motor Mode
        else if(strcmp(option, "default") == 0){
            motor_cmd->m2_mode = 0;
            sprintf(update_msg, "Motor 2 Mode: %d -> %d\r\n", last_motor_cmd->m2_mode, motor_cmd->m2_mode);
        } else if(strcmp(option, "boost") == 0){
            motor_cmd->m2_mode = 1;
            sprintf(update_msg, "Motor 2 Mode: %d -> %d\r\n", last_motor_cmd->m2_mode, motor_cmd->m2_mode);
        } else if(strcmp(option, "reverse") == 0){
            motor_cmd->m2_mode = 2;
            sprintf(update_msg, "Motor 2 Mode: %d -> %d\r\n", last_motor_cmd->m2_mode, motor_cmd->m2_mode);
        } else if(strcmp(option, "regen") == 0){
            motor_cmd->m2_mode = 3;
            sprintf(update_msg, "Motor 2 Mode: %d -> %d\r\n", last_motor_cmd->m2_mode, motor_cmd->m2_mode);
        }

        else { // unsupported option
            return -3;
        }

    } 
    
    else if (strcmp(motor, "send") == 0){ // Send Command to Motors
        SendMotorCommand(motor_cmd, last_motor_cmd);
    } 
    
    else if (strcmp(motor, "print") == 0){ // Print to UART Terminal
    	if(strcmp(option, "all") == 0){
    		// Print both commands
    		char msg[20] = {0};
    		sprintf(msg, "Last Sent Command\r\n");
    		HAL_UART_Transmit(huart, (uint8_t*)msg, strlen(msg), 100);
    		PrintMotorCommand(huart, last_motor_cmd);

    		sprintf(msg, "Current Command\r\n");
    		HAL_UART_Transmit(huart, (uint8_t*)msg, strlen(msg), 100);
    		PrintMotorCommand(huart, motor_cmd);

    	} else if(strcmp(option, "last") == 0){
    		// print last motor_cmd
    		PrintMotorCommand(huart, last_motor_cmd);

    	} else if(strcmp(option, "") == 0){
    		// Print current command
    		PrintMotorCommand(huart, motor_cmd);
    	} else {
            // unsupported option
            return -3;
        }

    } 
    
    else if(strcmp(motor, "logadc") == 0){ // ADC Log
    	if(strcmp(option, "on") == 0){
    		*adc_log_enable = 1;
            sprintf(update_msg, "ADC Logging %s\r\n", option);

    	} else if(strcmp(option, "off") == 0){
    		*adc_log_enable = 0;
            sprintf(update_msg, "ADC Logging %s\r\n", option);

    	} else { // unsupported option
    		return -3;
    	}
    } 
    
    else { // unsupported motor
        return -2;
    }
    
    HAL_UART_Transmit(huart, (uint8_t*)update_msg, strlen(update_msg), 100);

    return 1;
}

/*
    * Parse Motor_cmd into a CAN message
    * Set the motor command using CAN
    * 
    * @param motor_cmd: The motor command to send
    * @param last_motor_cmd: save the previous command
*/
void SendMotorCommand(Motor_cmd* motor_cmd, Motor_cmd* last_motor_cmd) {
    // Copy the current motor command to the last motor command
    memcpy(last_motor_cmd, motor_cmd, sizeof(Motor_cmd));
    
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

/*
    * Print the Motor_cmd via UART
    * 
    * @param huart: UART Handler
    * @param motor_cmd: The Motor_cmd to print
*/
void PrintMotorCommand(UART_HandleTypeDef* huart, Motor_cmd* motor_cmd) {
    char message[100] = {0};

    sprintf(message, "Motor: 1\r\n Status: %d\r\n Control: %d\r\n Mode: %d\r\n Dir: %d\r\n Val: %d\r\n\r\n", motor_cmd->m1_status, motor_cmd->m1_control, motor_cmd->m1_mode, motor_cmd->m1_dir, motor_cmd->m1_val);
    HAL_UART_Transmit(huart, (uint8_t*)message, strlen(message), 100);

    sprintf(message, "Motor: 2\r\n Status: %d\r\n Control: %d\r\n Mode: %d\r\n Dir: %d\r\n Val: %d\r\n\r\n", motor_cmd->m2_status, motor_cmd->m2_control, motor_cmd->m2_mode, motor_cmd->m2_dir, motor_cmd->m2_val);
    HAL_UART_Transmit(huart, (uint8_t*)message, strlen(message), 100);
}
