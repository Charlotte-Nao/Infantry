//
// Created by 14717 on 2025/8/27.
//

#include "Main.h"

#include "cmsis_os.h"

#include "../Application/can_control_motor.h"
#include "../Application/LED.h"


#include "../../Devices/motor/motor.h"
#include <math.h>

#include "Control.h"
#include "../Application/LED.h"

typedef enum
{
    CHASIS0 = 0,
    CHASIS1 = 1,
    CHASIS2 = 2,
    CHASIS3 = 3,
    YAW = 4,
    PITCH = 5,
    SHOOT = 6,
} channel;

void main_task(void const *argument) {
    motor_device_list_init();
    can_filter_init();

    while (1) {
        motor_device_list[YAW]->set_angle(motor_device_list[YAW],
                                          (motor_device_list[YAW]->get_ecd(motor_device_list[YAW]) + yaw_direction * 10)
                                          / 8192);
        motor_device_list[YAW]->update(motor_device_list[YAW]);
        motor_device_list[PITCH]->set_angle(motor_device_list[PITCH],
                                            (motor_device_list[PITCH]->get_ecd(motor_device_list[PITCH]) +
                                             pitch_direction * 10) / 8192);
        motor_device_list[PITCH]->update(motor_device_list[PITCH]);
        motor_device_list[SHOOT]->set_current(motor_device_list[SHOOT], shoot_speed * 2);
        CAN_cmd_gimbal(motor_device_list[YAW]->given_current(motor_device_list[YAW]),
                       motor_device_list[PITCH]->given_current(motor_device_list[PITCH]),
                       motor_device_list[SHOOT]->given_current(motor_device_list[SHOOT]),
                       0);
        CAN_cmd_chassis(0,0,0,0);
        LED_RED_Toggle();
        vTaskDelay(1);
    }
}
