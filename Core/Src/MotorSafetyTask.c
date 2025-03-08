/*
 * MotorSafetyTask.c
 *
 *  Created on: Sep 21, 2024
 *      Author: dominic (inspired by Omar)
 */

#include "MotorSafetyTask.h"
#include "main.h"
#include "CAN.h"   // Include the CAN header for message receiving


#define CAN_TX_ADDRESS 0x500
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
void sendADCValues(UART_HandleTypeDef* huart, DMA_HandleTypeDef* hdma, uint16_t* dma_adc_buf,uint8_t* dma_uart_buf, uint8_t enable){
	if(enable){
        uint16_t adc_vals[8] = {0};
        
        memcpy(adc_vals, dma_adc_buf, 8*sizeof(uint16_t));

		sprintf((char*)dma_uart_buf, "9999 %d %d %d %d %d %d %d %d 8888\r\n",
                adc_vals[0], adc_vals[1], adc_vals[2], adc_vals[3], adc_vals[4], adc_vals[5], adc_vals[6], adc_vals[7]);

        HAL_UART_DMAStop(huart);
        HAL_UART_Transmit_DMA(huart, dma_uart_buf, strlen((char*)dma_uart_buf));
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
double predict_dc_current(double x, double y, double x_mean, double x_std, double y_mean, double y_std, double coefficients[]) {
    // Normalize the input values
    double x_norm = (x - x_mean) / x_std;
    double y_norm = (y - y_mean) / y_std;

    // Calculate each term of the polynomial function
    double z = coefficients[0] +
               coefficients[1] * x_norm +
               coefficients[2] * y_norm +
               coefficients[3] * pow(x_norm, 2) +
               coefficients[4] * pow(y_norm, 2) +
               coefficients[5] * x_norm * y_norm +
               coefficients[6] * pow(x_norm, 3) +
               coefficients[7] * pow(y_norm, 3) +
               coefficients[8] * pow(x_norm, 2) * y_norm +
               coefficients[9] * x_norm * pow(y_norm, 2) +
               coefficients[10] * pow(x_norm, 4) +
               coefficients[11] * pow(y_norm, 4) +
               coefficients[12] * pow(x_norm, 3) * y_norm +
               coefficients[13] * pow(x_norm, 2) * pow(y_norm, 2) +
               coefficients[14] * x_norm * pow(y_norm, 3) +
               coefficients[15] * pow(x_norm, 5) +
               coefficients[16] * pow(y_norm, 5) +
               coefficients[17] * pow(x_norm, 4) * y_norm +
               coefficients[18] * pow(x_norm, 3) * pow(y_norm, 2) +
               coefficients[19] * pow(x_norm, 2) * pow(y_norm, 3) +
               coefficients[20] * x_norm * pow(y_norm, 4) +
               coefficients[21] * pow(x_norm, 6) +
               coefficients[22] * pow(y_norm, 6);

    return z;
}

uint8_t checkADCValues(const uint16_t* dma_adc_buf) {
    // Get data Torque, Speed data from CAN
	int16_t motor_rpm = 0, motor_torque = 0, inv_peak_cur = 0;

	motorSafetyTask(dma_adc_buf, &motor_rpm, &motor_torque, &inv_peak_cur);

    // Get expected current, voltage, torque, speed from formula

    // Get actual current, voltage, torque, speed from ADC values
	double x_mean = 72.63720930232559;    //update this value from the dataset being stored
	double x_std = 42.52505156747794;    //update this value from the dataset being stored
	double y_mean = 469.7674418604651;  //update this value from the dataset being stored
	double y_std = 300.12391527727823; //update this value from the dataset being stored

	    // Coefficients from the model
	double coefficients[] = {
	        8.99081610e+01,  5.41331468e+01,  5.42552952e+01,  3.52967650e+00,
	        -1.39293173e+00,  3.12836964e+01, -3.00255167e-01,  1.45315850e+00,
	        1.91373226e+00,  2.22332187e+00, -1.06146310e-01,  2.61885344e+00,
	        1.89991053e-01,  1.15873486e+00,  1.74216668e+00,  2.55780027e-01,
	        1.91376837e-01, -1.32832021e-01, -6.02793556e-02,  4.50207064e-01,
	        1.49791194e-02, -6.62227062e-02, -5.02206451e-01
	    };

	    // Example inputs
	    // Get the predicted current
	double predicted_current = predict_dc_current(motor_torque, motor_rpm, x_mean, x_std, y_mean, y_std, coefficients);

	    // Print the result

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


uint8_t motorSafetyTask(uint16_t* dma_adc_buf, int16_t *motor_rpm, int16_t *motor_torque, int16_t *inv_peak_cur){

	    // Attempt to receive a message from the CAN queue
	if (canMessageFlag ==1) {
		//clear the flag to say message is being processed
		canMessageFlag = 0;
		switch (receivedCANMessage.ID) {
			case (CAN_TX_ADDRESS + 0):   //need to find can_tx_address
				int16_t control_level = (receivedCANMessage.data[0] << 8) | receivedCANMessage.data[1]; // Signed 16-bit (Bytes 1-2)
				uint8_t control_mode = (receivedCANMessage.data[2] & 0b00000011);    // Bits 0-1: Control Mode (00 = TORQUE, 01 = SPEED)
				uint8_t motor_mode = (receivedCANMessage.data[2] >> 2) & 0b00000111;  // Bits 2-4: Motor Mode (Normal, Boost, Reverse, etc.)
				uint8_t sw_enable = (receivedCANMessage.data[2] >> 5) & 0b00000001;   // Bit 5: Software Enable (0 - DISABLED, 1 - ENABLED)
				uint8_t motor_state = (receivedCANMessage.data[2] >> 6) & 0b00000001; // Bit 6: Motor State (0 - IDLE, 1 - RUN)
				uint8_t debug_mode = (receivedCANMessage.data[2] >> 7) & 0b00000001;  // Bit 7: Debug Mode (0 - Normal, 1 - Debug)

				// Parse the remaining bytes for motor torque, rpm, and temperature
				*motor_torque = (receivedCANMessage.data[3] << 8) | receivedCANMessage.data[4]; // Signed 16-bit (Bytes 4-5)
				*motor_rpm = (receivedCANMessage.data[5] << 8) | receivedCANMessage.data[6];    // Signed 16-bit (Bytes 6-7)
				int8_t motor_temp = receivedCANMessage.data[7];                     // Signed 8-bit (Byte 8)

				// Output parsed values for the first message
				printf("Control Level: %d\n", control_level);
				printf("Control Mode: %d\n", control_mode);
				printf("Motor Mode: %d\n", motor_mode);
				printf("Software Enable: %d\n", sw_enable);
				printf("Motor State: %d\n", motor_state);
				printf("Debug Mode: %d\n", debug_mode);
				//printf("Motor Torque: %d Nm\n", motor_torque);
				//printf("Motor RPM: %d (0.1 RPM resolution)\n", motor_rpm);
				printf("Motor Temp: %d °C\n", motor_temp);
				break;

			case (CAN_TX_ADDRESS + 1):
				// Process motor power and position message
				*inv_peak_cur = (receivedCANMessage.data[0] << 8) | receivedCANMessage.data[1]; // Signed 16-bit
				int16_t motor_power = (receivedCANMessage.data[2] << 8) | receivedCANMessage.data[3];  // Signed 16-bit
				uint16_t abs_position = (receivedCANMessage.data[4] << 8) | receivedCANMessage.data[5]; // Unsigned 16-bit

				//printf("Inverter Peak Current: %d A\n", inv_peak_cur);
				printf("Motor Power: %d W\n", motor_power);
				printf("Motor Position: %u (1/2 degrees)\n", abs_position);
				break;

			// More cases for other messages



		   case (CAN_TX_ADDRESS + 2):  // Assuming this is the message ID for the warning code message
				// Combine the bytes into a 64-bit unsigned integer (warning_code)
				uint64_t warning_code = 0;
				warning_code |= ((uint64_t)receivedCANMessage.data[0] << 56);
				warning_code |= ((uint64_t)receivedCANMessage.data[1] << 48);
				warning_code |= ((uint64_t)receivedCANMessage.data[2] << 40);
				warning_code |= ((uint64_t)receivedCANMessage.data[3] << 32);
				warning_code |= ((uint64_t)receivedCANMessage.data[4] << 24);
				warning_code |= ((uint64_t)receivedCANMessage.data[5] << 16);
				warning_code |= ((uint64_t)receivedCANMessage.data[6] << 8);
				warning_code |= (uint64_t)receivedCANMessage.data[7];

				// Check if the warning code is non-zero
				if (warning_code != 0) {
					// Print a generic error or warning message (you could expand this with specific details later)
					printf("Warning/Error Detected: Warning Code = %llu\n", warning_code);
				}
				break;
		   case (CAN_TX_ADDRESS + 3):  // Assuming this is the message ID for the error code message
				   // Combine the bytes into a 64-bit unsigned integer (error_code)
				   uint64_t error_code = 0;
				   error_code |= ((uint64_t)receivedCANMessage.data[0] << 56);
				   error_code |= ((uint64_t)receivedCANMessage.data[1] << 48);
				   error_code |= ((uint64_t)receivedCANMessage.data[2] << 40);
				   error_code |= ((uint64_t)receivedCANMessage.data[3] << 32);
				   error_code |= ((uint64_t)receivedCANMessage.data[4] << 24);
				   error_code |= ((uint64_t)receivedCANMessage.data[5] << 16);
				   error_code |= ((uint64_t)receivedCANMessage.data[6] << 8);
				   error_code |= (uint64_t)receivedCANMessage.data[7];

				   // Handle the Initialization error (specific error code = 0)

				   printf("Error Detected: Error Code = %llu\n", error_code);

				   break;
		   default:
					// Handle unknown or unexpected message ID
					//printf("Unknown message ID: %x\n", receivedCANMessage.ID);
					break;

		}
	}
    // recv CAN

    // get expected val from formula

    // parse data

    // compare values with error

    // return error or not
	return 0;
}
