//
// Created by 14717 on 2025/9/7.
//

#ifndef INFANTRY_01_MOTOR_CONTROL_H
#define INFANTRY_01_MOTOR_CONTROL_H

#include "struct_typedef.h"

extern struct motor_device *motor_device_list[7];

extern void motor_device_list_init(void);

int16_t motor_turn(struct motor_device *motor, fp32 speed);

int16_t motor_turn_speed(struct motor_device *motor, fp32 speed);

#endif //INFANTRY_01_MOTOR_CONTROL_H