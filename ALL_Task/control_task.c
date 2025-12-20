#include "control_task.h"
#include "cmsis_os.h"
#include "../Components/motor/motor.h"
#include "../Application/robot_global.h"
#include "../Bsp/LED/bsp_LED.h"

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
    chassis[0] = motor_get_device("M3508_CHASSIS_1");
    chassis[1] = motor_get_device("M3508_CHASSIS_2");
    chassis[2] = motor_get_device("M3508_CHASSIS_3");
    chassis[3] = motor_get_device("M3508_CHASSIS_4");

    if (!pitch || !yaw || !chassis[0] || !stir_m) {
        while(1) { osDelay(100); } // 核心电机缺失则挂起
    }

    while (1) {
        /* --- 云台物理使能 --- */
        if (robot_ctrl.gimbal_mode == GIMBAL_RELAX) {
            pitch->send_disable_cmd(pitch);
            yaw->send_disable_cmd(yaw);
        } else {
            pitch->send_enable_cmd(pitch);
            yaw->send_enable_cmd(yaw);
        }

        /* --- 发射物理使能 (含摩擦轮和拨弹轮) --- */
        if (robot_ctrl.shoot_mode == SHOOT_STOP || robot_ctrl.gimbal_mode == GIMBAL_RELAX) {
            shoot_l->send_disable_cmd(shoot_l);
            shoot_r->send_disable_cmd(shoot_r);
            stir_m->send_disable_cmd(stir_m);
        } else {
            shoot_l->send_enable_cmd(shoot_l);
            shoot_r->send_enable_cmd(shoot_r);
            stir_m->send_enable_cmd(stir_m);
        }

        /* --- 底盘物理使能 --- */
        if (robot_ctrl.chassis_mode == CHASSIS_RELAX) {
            for(int i=0; i<4; i++) chassis[i]->send_disable_cmd(chassis[i]);
        } else {
            for(int i=0; i<4; i++) chassis[i]->send_enable_cmd(chassis[i]);
        }

        /* --- 物理下发 --- */
        Motor_All_Update();               // 计算 PID
        DJI_Motor_Send_CAN1_Group(&hcan1); // 发送 CAN1 (底盘+Yaw+拨弹)
        DJI_Motor_Send_CAN2_Group(&hcan2); // 发送 CAN2 (摩擦轮)
        pitch->send_ctrl_cmd(pitch);       // 发送达妙 MIT 帧

        osDelay(1);
    }
}