/*
 * MotorControlTask.h
 *
 *  Created on: Jul 13, 2024
 *      Author: dominic
 */

#ifndef INC_MOTORCONTROLTASK_H_
#define INC_MOTORCONTROLTASK_H_

struct Motor_cmd {
    uint8_t m1_status;  // 0 = off, 1 = on
    uint8_t m1_mode;    // 0 = RPM, 1 = Torque
    uint8_t m1_regen;   // 0 = accel, 1 = regenerative
    uint8_t m1_dir;     // 0 = forward, 1 = reverse
    uint32_t m1_val;    // Value of RPM or Torque
    
    uint8_t m2_status;
    uint8_t m2_mode;
    uint8_t m2_regen;
    uint8_t m2_dir;
    uint32_t m2_val;
};

#endif /* INC_MOTORCONTROLTASK_H_ */
