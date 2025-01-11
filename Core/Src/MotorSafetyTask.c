/*
 * MotorSafetyTask.c
 *
 *  Created on: Sep 21, 2024
 *      Author: dominic (inspired by Omar)
 */

#include "MotorSafetyTask.h"
#include "main.h"
#include "CAN.h"   // Include the CAN header for message receiving


typedef struct {
    uint32_t ID;
    uint8_t DLC;
    uint8_t data[8];
} CANMessage;

// Queue to hold CAN messages
volatile uint8_t canMessageFlag = 0; // 0: No new message, 1: New message available
CANMessage receivedCANMessage;
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

// Function to receive and handle CAN messages (ISR or main loop context)
void receiveCANMessageHandler(CANPeripheral* peripheral) {
    uint32_t messageID;
    uint8_t DLC;
    uint8_t data[8];

    // Receive CAN message from the hardware
    receiveCANMessage(0, &messageID, &DLC, data, peripheral); // Assuming channel 0

    // Fill the global CANMessage structure with the received data
    receivedCANMessage.ID = messageID;
    receivedCANMessage.DLC = DLC;
    memcpy(receivedCANMessage.data, data, DLC);

    // Set the flag to indicate a new message is available
    canMessageFlag = 1;
}


uint8_t motorSafetyTask(uint16_t* dma_adc_buf){

	    // Attempt to receive a message from the CAN queue
	if (canMessageFlag ==1) {
		//clear the flag to say message is being processed
		canMessageFlag = 0;
		switch (receivedMessage.ID) {
			case (CAN_TX_ADDRESS + 0):   //need to find can_tx_address
				int16_t control_level = (receivedMessage.data[0] << 8) | receivedMessage.data[1]; // Signed 16-bit (Bytes 1-2)
				uint8_t control_mode = (receivedMessage.data[2] & 0b00000011);    // Bits 0-1: Control Mode (00 = TORQUE, 01 = SPEED)
				uint8_t motor_mode = (receivedMessage.data[2] >> 2) & 0b00000111;  // Bits 2-4: Motor Mode (Normal, Boost, Reverse, etc.)
				uint8_t sw_enable = (receivedMessage.data[2] >> 5) & 0b00000001;   // Bit 5: Software Enable (0 - DISABLED, 1 - ENABLED)
				uint8_t motor_state = (receivedMessage.data[2] >> 6) & 0b00000001; // Bit 6: Motor State (0 - IDLE, 1 - RUN)
				uint8_t debug_mode = (receivedMessage.data[2] >> 7) & 0b00000001;  // Bit 7: Debug Mode (0 - Normal, 1 - Debug)

				// Parse the remaining bytes for motor torque, rpm, and temperature
				int16_t motor_torque = (receivedMessage.data[3] << 8) | receivedMessage.data[4]; // Signed 16-bit (Bytes 4-5)
				int16_t motor_rpm = (receivedMessage.data[5] << 8) | data[6];    // Signed 16-bit (Bytes 6-7)
				int8_t motor_temp = receivedMessage.data[7];                     // Signed 8-bit (Byte 8)

				// Output parsed values for the first message
				printf("Control Level: %d\n", control_level);
				printf("Control Mode: %d\n", control_mode);
				printf("Motor Mode: %d\n", motor_mode);
				printf("Software Enable: %d\n", sw_enable);
				printf("Motor State: %d\n", motor_state);
				printf("Debug Mode: %d\n", debug_mode);
				printf("Motor Torque: %d Nm\n", motor_torque);
				printf("Motor RPM: %d (0.1 RPM resolution)\n", motor_rpm);
				printf("Motor Temp: %d °C\n", motor_temp);
				break;

			case (CAN_TX_ADDRESS + 1):
				// Process motor power and position message
				int16_t inv_peak_cur = (receivedMessage.data[0] << 8) | receivedMessage.data[1]; // Signed 16-bit
				int16_t motor_power = (receivedMessage.data[2] << 8) | receivedMessage.data[3];  // Signed 16-bit
				uint16_t abs_position = (receivedMessage.data[4] << 8) | data[5]; // Unsigned 16-bit

				printf("Inverter Peak Current: %d A\n", inv_peak_cur);
				printf("Motor Power: %d W\n", motor_power);
				printf("Motor Position: %u (1/2 degrees)\n", abs_position);
				break;

			// More cases for other messages



		   case (CAN_TX_ADDRESS + 2):  // Assuming this is the message ID for the warning code message
				// Combine the bytes into a 64-bit unsigned integer (warning_code)
				uint64_t warning_code = 0;
				warning_code |= ((uint64_t)receivedMessage.data[0] << 56);
				warning_code |= ((uint64_t)receivedMessage.data[1] << 48);
				warning_code |= ((uint64_t)receivedMessage.data[2] << 40);
				warning_code |= ((uint64_t)receivedMessage.data[3] << 32);
				warning_code |= ((uint64_t)receivedMessage.data[4] << 24);
				warning_code |= ((uint64_t)receivedMessage.data[5] << 16);
				warning_code |= ((uint64_t)receivedMessage.data[6] << 8);
				warning_code |= (uint64_t)receivedMessage.data[7];

				// Check if the warning code is non-zero
				if (warning_code != 0) {
					// Print a generic error or warning message (you could expand this with specific details later)
					printf("Warning/Error Detected: Warning Code = %llu\n", warning_code);
				}
				break;
		   case (CAN_TX_ADDRESS + 3):  // Assuming this is the message ID for the error code message
				   // Combine the bytes into a 64-bit unsigned integer (error_code)
				   uint64_t error_code = 0;
				   error_code |= ((uint64_t)receivedMessage.data[0] << 56);
				   error_code |= ((uint64_t)receivedMessage.data[1] << 48);
				   error_code |= ((uint64_t)receivedMessage.data[2] << 40);
				   error_code |= ((uint64_t)receivedMessage.data[3] << 32);
				   error_code |= ((uint64_t)receivedMessage.data[4] << 24);
				   error_code |= ((uint64_t)receivedMessage.data[5] << 16);
				   error_code |= ((uint64_t)receivedMessage.data[6] << 8);
				   error_code |= (uint64_t)receivedMessage.data[7];

				   // Handle the Initialization error (specific error code = 0)

				   printf("Error Detected: Error Code = %llu\n", error_code);

				   break;
		   default:
					// Handle unknown or unexpected message ID
					printf("Unknown message ID: %x\n", receivedMessage.ID);
					break;

		}
	}
    // recv CAN

    // get expected val from formula

    // parse data

    // compare values with error

    // return error or not
	return 1;
}
