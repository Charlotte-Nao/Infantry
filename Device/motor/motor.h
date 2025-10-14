//
// Created by 14717 on 2025/8/27.
//

#ifndef INFANTRY_01_MOTOR_H
#define INFANTRY_01_MOTOR_H

#include "../../Application/struct_typedef.h"
#include "../../Algorithm/pid/pid.h"

struct motor_device {
    char *name;
    uint8_t id;
    void (*init)(struct motor_device *motor, struct PID *pid1, struct PID *pid2);
    void (*update)(struct motor_device *motor);
    void (*get_measure)(struct motor_device *motor, uint8_t *data);
    void (*set_speed)(struct motor_device *motor, fp32 speed);
    void (*set_current)(struct motor_device *motor, int16_t current);
    void (*set_angle)(struct motor_device *motor, fp32 angle);
    void (*set_ecd)(struct motor_device *motor, int16_t ecd);
    int16_t (*get_ecd)(struct motor_device *motor);
    int16_t (*get_speed)(struct motor_device *motor);
    fp32 (*get_angle_speed)(struct motor_device *motor);
    int16_t (*get_current)(struct motor_device *motor);
    fp32 (*get_angle)(struct motor_device *motor);
    int16_t (*get_temperature)(struct motor_device *motor);
    int16_t (*given_current)(struct motor_device *motor);
    int16_t (*given_voltage)(struct motor_device *motor);
    void *motor_data;
};

struct motor_device *motor_get_device(const char *name);

#endif //INFANTRY_01_MOTOR_H