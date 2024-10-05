/*
 * MotorSafetyTask.h
 *
 *  Created on: Sep 21, 2024
 *      Author: dominic
 */

#ifndef INC_MOTORSAFETYTASK_H_
#define INC_MOTORSAFETYTASK_H_

#include "main.h"
#include "stdint.h"
#include <stdio.h>
#include <string.h>

#include "MotorControlTask.h"

void MotorSafetyTask(void);

void getSafetyLimits(Motor_cmd* motor_cmd);

#endif /* INC_MOTORSAFETYTASK_H_ */
