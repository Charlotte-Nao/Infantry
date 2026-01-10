#include "control_task.h"
#include "cmsis_os.h"
#include "../Components/motor/motor.h"
#include "../Application/robot_global.h"
#include "stdio.h"

void control_task_func(void const * argument) {
    while (robot_ctrl.monitor.sensor_ready == 0) { osDelay(10); }
    osDelay(1000);
    Motor_System_PowerOn_Init();

    struct motor_device* pitch   = motor_get_device("J4310_PITCH");
    struct motor_device* yaw     = motor_get_device("GM6020_YAW");
    struct motor_device* shoot_l = motor_get_device("M3508_SHOOT_L");
    struct motor_device* shoot_r = motor_get_device("M3508_SHOOT_R");
    struct motor_device* stir_m  = motor_get_device("M2006_TRIGGER");
    struct motor_device* chassis[4];
    for(int i=0; i<4; i++) {
        char name[25]; sprintf(name, "M3508_CHASSIS_%d", i+1);
        chassis[i] = motor_get_device(name);
    }

    while (1) {
        // 云台使能
        if (robot_ctrl.gimbal_mode == GIMBAL_RELAX) {
            pitch->send_disable_cmd(pitch);
            yaw->send_disable_cmd(yaw);
        } else {
            pitch->send_enable_cmd(pitch);
            yaw->send_enable_cmd(yaw);
        }

        // 发射使能 (解耦)
        if (robot_ctrl.shoot_mode == SHOOT_STOP) {
            shoot_l->send_disable_cmd(shoot_l);
            shoot_r->send_disable_cmd(shoot_r);
            stir_m->send_disable_cmd(stir_m);
        } else {
            shoot_l->send_enable_cmd(shoot_l);
            shoot_r->send_enable_cmd(shoot_r);
            stir_m->send_enable_cmd(stir_m);
        }

        // 底盘使能
        if (robot_ctrl.chassis_mode == CHASSIS_RELAX) {
            for(int i=0; i<4; i++) if(chassis[i]) chassis[i]->send_disable_cmd(chassis[i]);
        } else {
            for(int i=0; i<4; i++) if(chassis[i]) chassis[i]->send_enable_cmd(chassis[i]);
        }

        Motor_All_Update();
        DJI_Motor_Send_CAN1_Group(&hcan1);
        DJI_Motor_Send_CAN2_Group(&hcan2);
        pitch->send_ctrl_cmd(pitch);

        osDelay(1);
    }
}