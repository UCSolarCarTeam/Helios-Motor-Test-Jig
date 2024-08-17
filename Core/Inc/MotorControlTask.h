/*
 * MotorControlTask.h
 *
 *  Created on: Jul 13, 2024
 *      Author: dominic
 */

#include "stdint.h"
#include "main.h"

#ifndef INC_MOTORCONTROLTASK_H_
#define INC_MOTORCONTROLTASK_H_

typedef struct {
    uint8_t m1_status;  // 0 = off, 1 = on
    uint8_t m1_control;	// 0 = RPM, 1 = Torque
    uint8_t m1_mode;	// 0 - normal
    					// 1 - boost
    					// 2 - reverse (in analog control)
    					// 3 - regeneration
    uint8_t m1_dir;		// 0 = Forward, 1 = Reverse
    uint32_t m1_val;    // Value of RPM or Torque
    
    uint8_t m2_status;
    uint8_t m2_control;
    uint8_t m2_mode;
    uint8_t m2_dir;
    uint32_t m2_val;
} Motor_cmd;


uint8_t CheckBuffer(uint8_t* buffer, uint8_t* buffer_index);

void ParseMotorCommand(Motor_cmd* motor_cmd, uint8_t* buffer, uint8_t* end_index, uint8_t* last_message);

void SendMotorCommand(Motor_cmd* motor_cmd);

#endif /* INC_MOTORCONTROLTASK_H_ */
