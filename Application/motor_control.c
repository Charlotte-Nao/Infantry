//
// Created by 14717 on 2025/9/7.
//

#include "motor_control.h"
#include "main.h"
#include "../Driver/can/can.h"
#include "../Device//motor/motor.h"
#include <math.h>

#define pi (fp32)M_PI

struct motor_device *motor_device_list[7];

struct PID *pid1_yaw;
struct PID *pid2_yaw;

struct PID *pid1_pitch;
struct PID *pid2_pitch;

struct PID *pid2_3508_0;
struct PID *pid2_3508_1;
struct PID *pid2_3508_2;
struct PID *pid2_3508_3;

struct PID *pid2_2006;

/**********************************************************************************************************************/
/*电机及其PID初始化*/
void motor_device_list_init(void)
{
    //pid1_yaw没用上
    pid1_yaw = pid_get_device("motor_yaw_PID1");
    const fp32 pid1_para_yaw[3] = {0, 0, 0};
    pid1_yaw->init(pid1_yaw, pid1_para_yaw, 0, 0);

    pid2_yaw = pid_get_device("motor_yaw_PID2");
    const fp32 pid2_para_yaw[3] = {300000, 0, 0};
    pid2_yaw->init(pid2_yaw, pid2_para_yaw, 25000, 500);

    pid1_pitch = pid_get_device("motor_pitch_PID1");
    const fp32 pid1_para_pitch[3] = {0, 0, 0};
    pid1_pitch->init(pid1_pitch, pid1_para_pitch, pi / 180, 0);

    pid2_pitch = pid_get_device("motor_pitch_PID2");
    const fp32 pid2_para_pitch[3] = {300000, 0, 0};
    pid2_pitch->init(pid2_pitch, pid2_para_pitch, 25000, 0);

    pid2_2006 = pid_get_device("motor_2006_PID2");
    const fp32 pid2_para_2006[3] = {3000000, 100,-1600000};
    pid2_2006->init(pid2_2006, pid2_para_2006, 10000, 500);

    const fp32 pid2_para_3508[3] = {3000000, 100,-1600000};

    pid2_3508_0 = pid_get_device("motor_3508_0_PID2");
    pid2_3508_0->init(pid2_3508_0, pid2_para_3508, 5000, 500);

    pid2_3508_1 = pid_get_device("motor_3508_1_PID2");
    pid2_3508_1->init(pid2_3508_1, pid2_para_3508, 5000, 500);

    pid2_3508_2 = pid_get_device("motor_3508_2_PID2");
    pid2_3508_2->init(pid2_3508_2, pid2_para_3508, 5000, 500);

    pid2_3508_3 = pid_get_device("motor_3508_3_PID2");
    pid2_3508_3->init(pid2_3508_3, pid2_para_3508, 5000, 500);

    motor_device_list[0] = motor_get_device("chassis_motor1");
    motor_device_list[0]->init(motor_device_list[0], NULL, pid2_3508_0);

    motor_device_list[1] = motor_get_device("chassis_motor2");
    motor_device_list[1]->init(motor_device_list[1], NULL, pid2_3508_1);

    motor_device_list[2] = motor_get_device("chassis_motor3");
    motor_device_list[2]->init(motor_device_list[2], NULL, pid2_3508_2);

    motor_device_list[3] = motor_get_device("chassis_motor4");
    motor_device_list[3]->init(motor_device_list[3], NULL, pid2_3508_3);

    motor_device_list[4] = motor_get_device("gimbal_motor_yaw");
    motor_device_list[4]->init(motor_device_list[4], pid1_yaw, pid2_yaw);

    motor_device_list[5] = motor_get_device("gimbal_motor_pitch");
    motor_device_list[5]->init(motor_device_list[5], pid1_pitch, pid2_pitch);

    motor_device_list[6] = motor_get_device("trigger_motor");
    motor_device_list[6]->init(motor_device_list[6], NULL, pid2_2006);
}

/**********************************************************************************************************************/
/*速度环位置环双环控制（云台PITCH用）*/
int16_t motor_turn(struct motor_device *motor, fp32 speed) {
    const fp32 pos = motor->get_angle(motor);
    motor->set_angle(motor, pos + speed);
    motor->update(motor);
    return  motor->given_voltage(motor);
}

/**********************************************************************************************************************/
/*速度环单环控制（底盘、拨弹用）*/
int16_t motor_turn_speed(struct motor_device *motor, fp32 speed) {
    motor->set_speed(motor, speed);
    motor->update(motor);
    return  motor->given_current(motor);
}
