//
// Created by 14717 on 2025/8/27.
//

#ifndef INFANTRY_01_MOTOR_H
#define INFANTRY_01_MOTOR_H

#include "../../Application/struct_typedef.h"

struct motor_device {
    char *name;
    uint8_t id;
    void (*init)(struct motor_device *motor);
    void (*update)(struct motor_device *motor);
    void (*get_measure)(struct motor_device *motor, uint8_t *data);
    void (*set_speed)(struct motor_device *motor, int16_t speed);
    void (*set_current)(struct motor_device *motor, int16_t current);
    void (*set_angle)(struct motor_device *motor, int16_t angle);
    void (*set_ecd)(struct motor_device *motor, int16_t ecd);
    int16_t (*get_ecd)(struct motor_device *motor);
    int16_t (*get_speed)(struct motor_device *motor);
    fp32 (*get_angle_speed)(struct motor_device *motor);
    int16_t (*get_current)(struct motor_device *motor);
    fp32 (*get_angle)(struct motor_device *motor);
    int16_t (*get_temperature)(struct motor_device *motor);
    int16_t (*given_current)(struct motor_device *motor);
    void *motor_data;
};

struct motor_device *motor_get_device(const char *name);

#endif //INFANTRY_01_MOTOR_H